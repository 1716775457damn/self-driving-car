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

app = Flask(__name__, static_folder='static')
CORS(app)

# 自定义日志过滤器
class DuplicateFilter(logging.Filter):
    def __init__(self, interval=1):
        self.last_log = {}
        self.interval = interval
        
    def filter(self, record):
        current_time = time.time()
        msg = record.getMessage()
        if msg in self.last_log:
            if current_time - self.last_log[msg] < self.interval:
                    return False
        self.last_log[msg] = current_time
        return True

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)
logger.addFilter(DuplicateFilter())

# 为Werkzeug日志添加过滤器
werkzeug_logger = logging.getLogger('werkzeug')
werkzeug_logger.addFilter(DuplicateFilter())

class KalmanFilter:
    def __init__(self, process_variance=1e-4, measurement_variance=1e-2):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimate = 0.0
        self.estimate_error = 1.0
        
    def update(self, measurement):
        # 预测步骤
        prediction_error = self.estimate_error + self.process_variance
        
        # 更新步骤
        kalman_gain = prediction_error / (prediction_error + self.measurement_variance)
        self.estimate = self.estimate + kalman_gain * (measurement - self.estimate)
        self.estimate_error = (1 - kalman_gain) * prediction_error
        
        return self.estimate

class MPUProcessor:
    def __init__(self):
        self.last_time = time.time()
        self.velocity = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.orientation = {'x': 0.0, 'y': 0.0, 'z': 0.0}  # 欧拉角
        self.last_gyro = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.last_accel = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        
        # 卡尔曼滤波器
        self.kalman_filters = {
            'ax': KalmanFilter(process_variance=1e-3, measurement_variance=1e-3),
            'ay': KalmanFilter(process_variance=1e-3, measurement_variance=1e-3),
            'az': KalmanFilter(process_variance=1e-3, measurement_variance=1e-3),
            'gx': KalmanFilter(process_variance=1e-4, measurement_variance=1e-4),
            'gy': KalmanFilter(process_variance=1e-4, measurement_variance=1e-4),
            'gz': KalmanFilter(process_variance=1e-4, measurement_variance=1e-4)
        }
        
        # 滤波器参数
        self.alpha = 0.2  # 加速度低通滤波系数
        self.beta = 0.8   # 角速度互补滤波系数
        
    def process_data(self, data):
        current_time = time.time()
        dt = min(current_time - self.last_time, 0.1)  # 限制最大时间步长
        self.last_time = current_time
        
        # 提取加速度和角速度数据
        accel = {
            'x': data['ax'],
            'y': data['ay'],
            'z': data['az']
        }
        gyro = {
            'x': data['gx'],
            'y': data['gy'],
            'z': data['gz']
        }
        
        # 滤波处理
        filtered_accel = {}
        filtered_gyro = {}
        
        # 加速度滤波
        for axis in ['x', 'y', 'z']:
            # 卡尔曼滤波
            filtered_accel[axis] = self.kalman_filters[f'a{axis}'].update(accel[axis])
            # 低通滤波
            filtered_accel[axis] = self.alpha * filtered_accel[axis] + (1 - self.alpha) * self.last_accel[axis]
            
        # 角速度滤波
        for axis in ['x', 'y', 'z']:
            # 卡尔曼滤波
            filtered_gyro[axis] = self.kalman_filters[f'g{axis}'].update(gyro[axis])
            # 互补滤波
            filtered_gyro[axis] = self.beta * filtered_gyro[axis] + (1 - self.beta) * self.last_gyro[axis]
        
        # 更新上一次的值
        self.last_accel = filtered_accel.copy()
        self.last_gyro = filtered_gyro.copy()
        
        # 移除重力加速度影响
        gravity_compensation = 9.81 * math.cos(self.orientation['x'])
        filtered_accel['z'] -= gravity_compensation
        
        # 计算姿态（积分角速度）
        for axis in ['x', 'y', 'z']:
            self.orientation[axis] += filtered_gyro[axis] * dt
            # 限制角度范围在-180到180度之间
            self.orientation[axis] = ((self.orientation[axis] + 180) % 360) - 180
        
        # 根据姿态转换加速度到全局坐标系
        global_accel = self._transform_to_global(filtered_accel)
        
        # 计算速度（积分加速度）
        for axis in ['x', 'y', 'z']:
            # 应用死区
            if abs(global_accel[axis]) < 0.05:
                global_accel[axis] = 0
            
            # 积分计算速度
            self.velocity[axis] += global_accel[axis] * dt
            
            # 应用阻尼
            damping = 0.98
            self.velocity[axis] *= damping
            
            # 速度死区
            if abs(self.velocity[axis]) < 0.02:
                self.velocity[axis] = 0
        
        # 计算位置（积分速度）
        for axis in ['x', 'y', 'z']:
            self.position[axis] += self.velocity[axis] * dt
            
            # 位置限制
            position_limit = 10.0
            self.position[axis] = max(min(self.position[axis], position_limit), -position_limit)
        
        return {
            'acceleration': filtered_accel,
            'gyro': filtered_gyro,
            'orientation': self.orientation,
            'velocity': self.velocity,
            'position': self.position
        }
        
    def _transform_to_global(self, local_accel):
        """将局部坐标系的加速度转换到全局坐标系"""
        # 获取欧拉角（弧度）
        roll = math.radians(self.orientation['x'])
        pitch = math.radians(self.orientation['y'])
        yaw = math.radians(self.orientation['z'])
        
        # 创建旋转矩阵
        Rx = np.array([
            [1, 0, 0],
            [0, math.cos(roll), -math.sin(roll)],
            [0, math.sin(roll), math.cos(roll)]
        ])
        
        Ry = np.array([
            [math.cos(pitch), 0, math.sin(pitch)],
            [0, 1, 0],
            [-math.sin(pitch), 0, math.cos(pitch)]
        ])
        
        Rz = np.array([
            [math.cos(yaw), -math.sin(yaw), 0],
            [math.sin(yaw), math.cos(yaw), 0],
            [0, 0, 1]
        ])
        
        # 合成旋转矩阵
        R = Rz @ Ry @ Rx
        
        # 转换加速度
        local_vec = np.array([local_accel['x'], local_accel['y'], local_accel['z']])
        global_vec = R @ local_vec
        
        return {
            'x': global_vec[0],
            'y': global_vec[1],
            'z': global_vec[2]
        }

class TCPServer:
    def __init__(self, host='0.0.0.0', port=8080):
        self.host = host
        self.port = port
        self.server_socket = None
        self.client_socket = None
        self.client_address = None
        self.is_running = False
        self.mpu_data = {'x': 0, 'y': 0, 'z': 0}
        self.last_update = 0
        self.lock = threading.Lock()
        self.mpu_processor = MPUProcessor()  # 添加MPU处理器

    def start(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(1)
        self.is_running = True
        logger.info(f"TCP服务器启动在 {self.host}:{self.port}")

        while self.is_running:
            try:
                self.client_socket, self.client_address = self.server_socket.accept()
                logger.info(f"ESP32已连接: {self.client_address}")
                self.handle_client()
            except Exception as e:
                logger.error(f"处理客户连接时出错: {e}")
                if self.client_socket:
                    self.client_socket.close()

    def handle_client(self):
        buffer = ""
        while self.is_running and self.client_socket:
            try:
                data = self.client_socket.recv(1024).decode('utf-8')
                if not data:
                    break

                buffer += data
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    self.process_data(line.strip())

            except Exception as e:
                logger.error(f"处理数据时出错: {e}")
                break

        if self.client_socket:
            self.client_socket.close()
            logger.info("ESP32断开连接")

    def process_data(self, data):
        try:
            if data.startswith('m:'):
                values = data[2:].split(',')
                if len(values) == 6:  # 现在我们期望6个值：ax,ay,az,gx,gy,gz
                    with self.lock:
                        raw_data = {
                            'ax': float(values[0]),
                            'ay': float(values[1]),
                            'az': float(values[2]),
                            'gx': float(values[3]),
                            'gy': float(values[4]),
                            'gz': float(values[5])
                        }
                        # 打印接收到的原始数据
                        logger.info(f"接收到MPU数据: {raw_data}")
                        
                        # 处理MPU数据
                        processed_data = self.mpu_processor.process_data(raw_data)
                        self.mpu_data = processed_data
                        self.last_update = time.time()
                        
                        # 打印处理后的数据
                        logger.info(f"处理后的数据: {processed_data}")
                else:
                    logger.warning(f"MPU数据格式错误: {data}")
            else:
                logger.warning(f"未知数据格式: {data}")
        except Exception as e:
            logger.error(f"处理MPU数据时出错: {data} - {e}")

    def get_mpu_data(self):
        with self.lock:
            return self.mpu_data.copy()

    def stop(self):
        self.is_running = False
        if self.client_socket:
            self.client_socket.close()
        if self.server_socket:
            self.server_socket.close()

# 创建TCP服务器实例
tcp_server = TCPServer()

# 在新线程中启动TCP服务器
server_thread = threading.Thread(target=tcp_server.start)
server_thread.daemon = True
server_thread.start()

@app.route('/')
def index():
    return send_from_directory('static', 'index.html')

@app.route('/gyro')
def get_gyro_data():
    if not tcp_server.mpu_data:
        logger.warning("没有可用的MPU数据")
        return jsonify({'error': 'No MPU data available'})
    try:
        data = tcp_server.mpu_data
        # 打印发送到前端的数据
        logger.info(f"发送到前端的数据: {data}")
        return jsonify(data)  # 直接返回已处理的数据
    except Exception as e:
        logger.error(f"获取MPU数据时出错: {e}")
        return jsonify({'error': str(e)})

if __name__ == '__main__':
    try:
        app.run(host='0.0.0.0', port=80, debug=False)
    finally:
        tcp_server.stop()