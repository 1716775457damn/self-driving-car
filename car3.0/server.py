# 导入所需模块
from flask import Flask, jsonify, send_from_directory
from flask_cors import CORS
import socket
import threading
import json
import logging
import time
import os
import math
import numpy as np
import torch
from queue import Queue, Empty, Full
import torch.nn as nn
import torch.optim as optim
from collections import deque
import atexit
import sys
import signal
import psutil

# 初始化GPU
try:
    if torch.cuda.is_available():
        USE_GPU = True
        DEVICE = torch.device('cuda')
        print("GPU加速已启用 - 使用设备:", torch.cuda.get_device_name(0))
    else:
        USE_GPU = False
        DEVICE = torch.device('cpu')
        print("未找到可用的GPU，使用CPU模式")
except Exception as e:
    USE_GPU = False
    DEVICE = torch.device('cpu')
    print(f"GPU初始化失败: {e}，使用CPU模式")

# 配置日志
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# 初始化Flask应用
app = Flask(__name__, static_folder='static')
CORS(app, resources={r"/*": {"origins": "*"}}, supports_credentials=True)

class KalmanFilter:
    def __init__(self, process_variance=1e-4, measurement_variance=1e-2):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimate = 0.0
        self.estimate_error = 1.0
        
    def update(self, measurement):
        prediction_error = self.estimate_error + self.process_variance
        kalman_gain = prediction_error / (prediction_error + self.measurement_variance)
        self.estimate = self.estimate + kalman_gain * (measurement - self.estimate)
        self.estimate_error = (1 - kalman_gain) * prediction_error
        return self.estimate

class MotionPredictor(nn.Module):
    def __init__(self, input_size=9, hidden_size=32, num_layers=2, output_size=9):
        super(MotionPredictor, self).__init__()
        self.hidden_size = hidden_size
        self.num_layers = num_layers
        
        self.lstm = nn.LSTM(input_size, hidden_size, num_layers, batch_first=True)
        self.fc = nn.Linear(hidden_size, output_size)
        
    def forward(self, x):
        h0 = torch.zeros(self.num_layers, x.size(0), self.hidden_size).to(x.device)
        c0 = torch.zeros(self.num_layers, x.size(0), self.hidden_size).to(x.device)
        
        out, _ = self.lstm(x, (h0, c0))
        out = self.fc(out[:, -1, :])
        return out

class MPUProcessor:
    def __init__(self):
        self.last_time = time.time()
        self.velocity = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.orientation = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.gravity = {'x': 0.0, 'y': 0.0, 'z': 1.0}
        self.alpha = 0.8
        
        # 初始化LSTM模型
        self.sequence_length = 10
        self.motion_predictor = MotionPredictor().to(DEVICE)
        self.optimizer = optim.Adam(self.motion_predictor.parameters(), lr=0.001)
        self.criterion = nn.MSELoss()
        
        # 数据缓冲区
        self.data_buffer = deque(maxlen=self.sequence_length)
        self.prediction_buffer = deque(maxlen=5)  # 存储最近5个预测结果
        
        # 添加卡尔曼滤波器
        self.kalman_filters = {
            'ax': KalmanFilter(process_variance=1e-4, measurement_variance=1e-2),
            'ay': KalmanFilter(process_variance=1e-4, measurement_variance=1e-2),
            'az': KalmanFilter(process_variance=1e-4, measurement_variance=1e-2),
            'gx': KalmanFilter(process_variance=1e-4, measurement_variance=1e-2),
            'gy': KalmanFilter(process_variance=1e-4, measurement_variance=1e-2),
            'gz': KalmanFilter(process_variance=1e-4, measurement_variance=1e-2),
            'vx': KalmanFilter(process_variance=1e-3, measurement_variance=1e-1),
            'vy': KalmanFilter(process_variance=1e-3, measurement_variance=1e-1),
            'vz': KalmanFilter(process_variance=1e-3, measurement_variance=1e-1),
            'px': KalmanFilter(process_variance=1e-3, measurement_variance=1e-1),
            'py': KalmanFilter(process_variance=1e-3, measurement_variance=1e-1),
            'pz': KalmanFilter(process_variance=1e-3, measurement_variance=1e-1)
        }
        
        self.movement_threshold = 0.05
        self.static_samples = 0
        self.static_threshold = 10
        
        logger.info("MPU处理器初始化完成")

    def prepare_sequence(self, data_list):
        """将传感器数据转换为模型输入序列"""
        sequence = []
        # 转换deque为列表
        buffer_list = list(self.data_buffer)
        
        # 获取最近的sequence_length个数据
        recent_data = buffer_list[-min(self.sequence_length, len(buffer_list)):]
        
        # 构建序列
        for d in recent_data:
            features = [
                d['acceleration']['x'], d['acceleration']['y'], d['acceleration']['z'],
                d['gyro']['x'], d['gyro']['y'], d['gyro']['z'],
                d['orientation']['x'], d['orientation']['y'], d['orientation']['z']
            ]
            sequence.append(features)
        
        # 如果数据不足，用第一个数据填充
        if sequence:
            while len(sequence) < self.sequence_length:
                sequence.insert(0, sequence[0])
        else:
            # 如果没有数据，创建零序列
            zero_features = [0.0] * 9  # 9个特征全部设为0
            sequence = [zero_features] * self.sequence_length
        
        return torch.FloatTensor([sequence]).to(DEVICE)

    def update_model(self, current_data, predicted_data):
        """更新LSTM模型"""
        if len(self.data_buffer) > 0:
            self.motion_predictor.train()
            
            # 准备输入序列
            x = self.prepare_sequence(self.data_buffer)
            
            # 准备目标数据
            y = torch.FloatTensor([[
                current_data['acceleration']['x'], current_data['acceleration']['y'], current_data['acceleration']['z'],
                current_data['gyro']['x'], current_data['gyro']['y'], current_data['gyro']['z'],
                current_data['orientation']['x'], current_data['orientation']['y'], current_data['orientation']['z']
            ]]).to(DEVICE)
            
            # 训练步骤
            self.optimizer.zero_grad()
            output = self.motion_predictor(x)
            loss = self.criterion(output, y)
            loss.backward()
            self.optimizer.step()

    def predict_motion(self):
        """使用LSTM模型预测下一个状态"""
        if len(self.data_buffer) == 0:
            return None
            
        self.motion_predictor.eval()
        with torch.no_grad():
            x = self.prepare_sequence(self.data_buffer)
            prediction = self.motion_predictor(x)
            return prediction.cpu().numpy()[0]

    def is_static(self, linear_acc):
        """检测设备是否静止"""
        magnitude = math.sqrt(linear_acc['x']**2 + linear_acc['y']**2 + linear_acc['z']**2)
        if magnitude < self.movement_threshold:
            self.static_samples += 1
        else:
            self.static_samples = 0
        return self.static_samples > self.static_threshold

    def process_data(self, data):
        try:
            current_time = time.time()
            dt = min(current_time - self.last_time, 0.1)
            self.last_time = current_time

            # 应用卡尔曼滤波
            accel = {
                'x': self.kalman_filters['ax'].update(data['ax']),
                'y': self.kalman_filters['ay'].update(data['ay']),
                'z': self.kalman_filters['az'].update(data['az'])
            }

            gyro = {
                'x': self.kalman_filters['gx'].update(data['gx']),
                'y': self.kalman_filters['gy'].update(data['gy']),
                'z': self.kalman_filters['gz'].update(data['gz'])
            }

            # 更新重力估计
            self.gravity['x'] = self.alpha * self.gravity['x'] + (1 - self.alpha) * accel['x']
            self.gravity['y'] = self.alpha * self.gravity['y'] + (1 - self.alpha) * accel['y']
            self.gravity['z'] = self.alpha * self.gravity['z'] + (1 - self.alpha) * accel['z']

            # 计算线性加速度
            linear_acceleration = {
                'x': accel['x'] - self.gravity['x'],
                'y': accel['y'] - self.gravity['y'],
                'z': accel['z'] - self.gravity['z']
            }

            # 更新姿态角
            for axis in ['x', 'y', 'z']:
                self.orientation[axis] += gyro[axis] * dt
                self.orientation[axis] = ((self.orientation[axis] + 180) % 360) - 180

            # 准备当前数据
            current_data = {
                'acceleration': accel,
                'linear_acceleration': linear_acceleration,
                'gyro': gyro,
                'orientation': self.orientation.copy(),
                'velocity': self.velocity.copy(),
                'position': self.position.copy()
            }

            # 添加到数据缓冲区
            self.data_buffer.append(current_data)

            try:
                # 使用LSTM预测下一个状态
                predicted_motion = self.predict_motion()
                if predicted_motion is not None:
                    # 更新模型
                    self.update_model(current_data, predicted_motion)
                    
                    # 使用预测结果和实际数据的加权平均
                    weight = 0.3  # 预测权重
                    if not self.is_static(linear_acceleration):
                        # 更新速度和位置（结合预测）
                        for i, axis in enumerate(['x', 'y', 'z']):
                            # 预测加速度
                            pred_accel = predicted_motion[i]
                            # 结合预测和实际加速度
                            combined_accel = weight * pred_accel + (1 - weight) * linear_acceleration[axis]
                            
                            # 更新速度
                            new_velocity = self.velocity[axis] + combined_accel * dt
                            self.velocity[axis] = self.kalman_filters[f'v{axis}'].update(new_velocity)
                            self.velocity[axis] = max(min(self.velocity[axis], 5.0), -5.0)
                            
                            # 更新位置
                            new_position = self.position[axis] + self.velocity[axis] * dt
                            self.position[axis] = self.kalman_filters[f'p{axis}'].update(new_position)
                    else:
                        # 静止状态处理
                        for axis in ['x', 'y', 'z']:
                            self.velocity[axis] *= 0.95
                            if abs(self.velocity[axis]) < 0.01:
                                self.velocity[axis] = 0
            except Exception as e:
                logger.error(f"LSTM预测出错: {e}", exc_info=True)
                # 如果LSTM预测失败，使用传统方法更新
                if not self.is_static(linear_acceleration):
                    for axis in ['x', 'y', 'z']:
                        new_velocity = self.velocity[axis] + linear_acceleration[axis] * dt
                        self.velocity[axis] = self.kalman_filters[f'v{axis}'].update(new_velocity)
                        self.velocity[axis] = max(min(self.velocity[axis], 5.0), -5.0)
                        
                        new_position = self.position[axis] + self.velocity[axis] * dt
                        self.position[axis] = self.kalman_filters[f'p{axis}'].update(new_position)
                else:
                    for axis in ['x', 'y', 'z']:
                        self.velocity[axis] *= 0.95
                        if abs(self.velocity[axis]) < 0.01:
                            self.velocity[axis] = 0

            return {
                'acceleration': accel,
                'linear_acceleration': linear_acceleration,
                'gyro': gyro,
                'orientation': self.orientation.copy(),
                'velocity': self.velocity.copy(),
                'position': self.position.copy()
            }

        except Exception as e:
            logger.error(f"处理MPU数据时出错: {e}", exc_info=True)
            return None

class TCPServer:
    def __init__(self, host='0.0.0.0', port=8080):
        self.host = host
        self.port = port
        self.server_socket = None
        self.client_socket = None
        self.client_address = None
        self.is_running = False
        self.connection_status = False
        self.mpu_processor = MPUProcessor()
        self.data_queue = Queue(maxsize=1000)
        self.last_processed_data = None
        self.data_lock = threading.Lock()
        
        logger.info(f"TCP服务器初始化完成 - 主机: {host}, 端口: {port}")

    def start(self):
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(1)
            self.is_running = True
            logger.info(f"TCP服务器启动在 {self.host}:{self.port}")

            while self.is_running:
                try:
                    logger.debug("等待客户端连接...")
                    self.client_socket, self.client_address = self.server_socket.accept()
                    self.client_socket.settimeout(5.0)
                    self.connection_status = True
                    logger.info(f"ESP32已连接: {self.client_address}")
                    self.handle_client()
                except socket.timeout:
                    logger.warning("连接超时")
                    continue
                except Exception as e:
                    logger.error(f"处理客户端连接时出错: {e}", exc_info=True)
                    self.connection_status = False
                    if self.client_socket:
                        self.client_socket.close()
        except Exception as e:
            logger.error(f"TCP服务器启动错误: {e}", exc_info=True)
        finally:
            self.stop()

    def handle_client(self):
        buffer = ""
        while self.is_running and self.client_socket:
            try:
                data = self.client_socket.recv(1024).decode('utf-8')
                if not data:
                    logger.warning("客户端断开连接")
                    self.connection_status = False
                    break

                buffer += data
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    self.process_data(line.strip())

            except socket.timeout:
                logger.debug("等待数据超时")
                continue
            except Exception as e:
                logger.error(f"处理数据时出错: {e}", exc_info=True)
                self.connection_status = False
                break

        if self.client_socket:
            self.client_socket.close()
            self.connection_status = False
            logger.info("ESP32断开连接")

    def process_data(self, data):
        try:
            if data.startswith('m:'):
                values = data[2:].split(',')
                if len(values) == 6:
                    raw_data = {
                        'ax': float(values[0]),
                        'ay': float(values[1]),
                        'az': float(values[2]),
                        'gx': float(values[3]),
                        'gy': float(values[4]),
                        'gz': float(values[5])
                    }
                    
                    processed_data = self.mpu_processor.process_data(raw_data)
                    if processed_data:
                        with self.data_lock:
                            self.last_processed_data = processed_data
                    
        except Exception as e:
            logger.error(f"处理MPU数据时出错: {data} - {e}", exc_info=True)

    def get_mpu_data(self):
        try:
            with self.data_lock:
                if self.last_processed_data is None:
                    return {
                        'acceleration': {'x': 0, 'y': 0, 'z': 0},
                        'gyro': {'x': 0, 'y': 0, 'z': 0},
                        'orientation': {'x': 0, 'y': 0, 'z': 0},
                        'velocity': {'x': 0, 'y': 0, 'z': 0},
                        'position': {'x': 0, 'y': 0, 'z': 0},
                        'connection_status': self.connection_status
                    }
                return {**self.last_processed_data, 'connection_status': self.connection_status}
        except Exception as e:
            logger.error(f"获取MPU数据时出错: {e}", exc_info=True)
            return None

    def get_connection_info(self):
        return {
            'is_running': self.is_running,
            'connection_status': self.connection_status,
            'client_address': self.client_address,
            'queue_size': self.data_queue.qsize()
        }

    def stop(self):
        self.is_running = False
        self.connection_status = False
        if self.client_socket:
            self.client_socket.close()
        if self.server_socket:
            self.server_socket.close()
        logger.info("TCP服务器已停止")

# 全局变量
tcp_server = None

def create_tcp_server():
    """创建并初始化TCP服务器"""
    global tcp_server
    try:
        logger.info("创建TCP服务器实例...")
        tcp_server = TCPServer(host='0.0.0.0', port=8080)
        
        logger.info("启动TCP服务器线程...")
        server_thread = threading.Thread(target=tcp_server.start)
        server_thread.daemon = True
        server_thread.start()
        
        time.sleep(1)
        connection_info = tcp_server.get_connection_info()
        logger.info(f"TCP服务器状态: {connection_info}")
        
        return tcp_server
    except Exception as e:
        logger.error(f"创建TCP服务器失败: {e}", exc_info=True)
        return None

# 初始化TCP服务器
tcp_server = create_tcp_server()

@app.route('/')
def index():
    return send_from_directory('static', 'index.html')

@app.route('/gyro')
def get_gyro_data():
    try:
        if tcp_server is None:
            logger.error("TCP服务器未初始化")
            return jsonify({
                'error': 'TCP server not initialized',
                'connection_status': False
            })

        logger.debug(f"收到/gyro请求，当前连接状态: {tcp_server.connection_status}")
        data = tcp_server.get_mpu_data()
        if not data:
            logger.warning("无法获取MPU数据")
            return jsonify({
                'error': 'No MPU data available',
                'connection_status': tcp_server.connection_status,
                'acceleration': {'x': 0, 'y': 0, 'z': 0},
                'linear_acceleration': {'x': 0, 'y': 0, 'z': 0},
                'gyro': {'x': 0, 'y': 0, 'z': 0},
                'orientation': {'x': 0, 'y': 0, 'z': 0},
                'velocity': {'x': 0, 'y': 0, 'z': 0},
                'position': {'x': 0, 'y': 0, 'z': 0}
            })
        
        response = jsonify(data)
        response.headers.add('Access-Control-Allow-Origin', '*')
        response.headers.add('Access-Control-Allow-Headers', 'Content-Type')
        response.headers.add('Access-Control-Allow-Methods', 'GET')
        logger.debug(f"返回数据: {data}")
        return response
    except Exception as e:
        logger.error(f"处理/gyro请求时出错: {str(e)}", exc_info=True)
        return jsonify({
            'error': str(e),
            'connection_status': False
        })

if __name__ == '__main__':
    try:
        app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
    except Exception as e:
        logger.error(f"服务器启动错误: {e}", exc_info=True)
    finally:
        if tcp_server:
            tcp_server.stop()