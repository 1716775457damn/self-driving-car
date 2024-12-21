from flask import Flask, request, send_file, jsonify
import time
import logging

# 创建一个自定义的日志过滤器
class DuplicateFilter(logging.Filter):
    def __init__(self):
        super().__init__()
        self.last_log = {}
        self.repeat_count = {}
        
    def filter(self, record):
        # 获取日志内容和参数
        log_content = record.msg
        if record.args:
            log_content = log_content % record.args
            
        log_key = (record.module, record.levelno, log_content)
        current_time = time.time()
        
        # 检查是否是重复日志
        if log_key in self.last_log:
            last_time, last_content = self.last_log[log_key]
            # 如果日志内容相同且时间间隔小于1秒
            if log_content == last_content and (current_time - last_time) < 1:
                self.repeat_count[log_key] = self.repeat_count.get(log_key, 0) + 1
                # 如果连续重复超过3次，不打印
                if self.repeat_count[log_key] >= 3:
                    return False
            else:
                # 如果有累积的重复日志，打印汇总信息
                if log_key in self.repeat_count and self.repeat_count[log_key] > 3:
                    record.msg = f"{log_content} (重复 {self.repeat_count[log_key]} 次)"
                    record.args = None
                self.repeat_count[log_key] = 0
        
        self.last_log[log_key] = (current_time, log_content)
        return True

# 配置日志
logger = logging.getLogger(__name__)
# 移除所有现有的处理器
for handler in logger.handlers[:]:
    logger.removeHandler(handler)
# 移除所有现有的过滤器
for filter in logger.filters[:]:
    logger.removeFilter(filter)

logger.setLevel(logging.INFO)

# 创建控制台处理器
console_handler = logging.StreamHandler()
console_handler.setLevel(logging.INFO)

# 设置日志格式
formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
console_handler.setFormatter(formatter)

# 添加过滤器
duplicate_filter = DuplicateFilter()
console_handler.addFilter(duplicate_filter)

# 添加处理器到日志记录器
logger.addHandler(console_handler)

# 防止日志输出重复
logger.propagate = False

class PIDController:
    def __init__(self, kp, ki, kd, output_min, output_max, sample_time=0.1, max_change_rate=200):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.sample_time = sample_time
        self.max_change_rate = max_change_rate  # 每秒最大变化率
        
        self.last_time = None
        self.last_error = 0
        self.integral = 0
        self.last_output = 0
        
    def compute(self, setpoint, current_value):
        current_time = time.time()
        
        if self.last_time is None:
            self.last_time = current_time
            self.last_output = current_value
            return current_value
            
        # 计算时间差
        dt = current_time - self.last_time
        if dt < self.sample_time:
            return self.last_output
            
        # 计算误差
        error = setpoint - current_value
        
        # 计算积分项
        self.integral += error * dt
        # 积分限幅，防止积分饱和
        self.integral = max(self.output_min/self.ki, min(self.output_max/self.ki, self.integral))
        
        # 计算微分项
        derivative = (error - self.last_error) / dt if dt > 0 else 0
        
        # 计算输出
        output = (
            self.kp * error +  # 比例项
            self.ki * self.integral +  # 积分项
            self.kd * derivative  # 微分项
        )
        
        # 限制变化率
        max_change = self.max_change_rate * dt
        output = max(self.last_output - max_change, 
                    min(self.last_output + max_change, output))
        
        # 输出限幅
        output = max(self.output_min, min(self.output_max, output))
        
        # 更新状态
        self.last_error = error
        self.last_time = current_time
        self.last_output = output
        
        return output
        
    def reset(self):
        self.last_time = None
        self.last_error = 0
        self.integral = 0
        self.last_output = 0

# 添加连接状态跟踪
class ESP32State:
    def __init__(self):
        self.last_seen = 0
        self.request_count = 0
        self.last_command_sent = None
        self.last_status = "未连接"
    
    def update_seen(self):
        self.last_seen = time.time()
        self.request_count += 1
        if self.is_connected():
            if self.last_status != "已连接":
                self.last_status = "已连接"
                logger.info("ESP32 已连接")
        
    def is_connected(self):
        return (time.time() - self.last_seen) < 2
    
    def get_status(self):
        connected = self.is_connected()
        if not connected and self.last_status != "未连接":
            self.last_status = "未连接"
            logger.warning("ESP32 连接丢失")
        return {
            "connected": connected,
            "last_seen": self.last_seen,
            "request_count": self.request_count,
            "last_command": self.last_command_sent,
            "status": self.last_status
        }

app = Flask(__name__)

# 存储最新的控制值
current_throttle = 0
current_steering = 0
target_throttle = 0
target_steering = 0
last_command_time = time.time()
esp32_last_seen = 0

# 创建 PID 控制器
# 参数顺序：Kp, Ki, Kd, 最小输出, 最大输出, 采样时间, 最大变化率
throttle_pid = PIDController(
    kp=3.0,           # 增加比例系数，提高响应速度
    ki=0.2,           # 增加积分系数，减少稳态误差
    kd=0.05,          # 保持较小的微分系数，防止过度震荡
    output_min=-100,
    output_max=100,
    sample_time=0.05, # 减小采样时间，提高响应速度
    max_change_rate=400  # 增大变化率限制，允许更快的响应
)

steering_pid = PIDController(
    kp=2.5,           # 转向可以稍微温和一些
    ki=0.1,
    kd=0.02,
    output_min=-100,
    output_max=100,
    sample_time=0.05,
    max_change_rate=500  # 转向需要更快的响应
)

# 创建 ESP32 状态跟踪器
esp32_state = ESP32State()

@app.route('/')
def index():
    return send_file('index.html')

@app.route('/control')
def control():
    global current_throttle, current_steering, target_throttle, target_steering, last_command_time
    
    try:
        # 从前端获取目标值
        target_throttle = float(request.args.get('throttle', 0))
        target_steering = float(request.args.get('steering', 0))
        
        # 记录前端请求
        logger.info(f"收到前端控制请求 - 目标油门: {target_throttle}, 目标转向: {target_steering}")
        
        # ESP32 连接状态检查
        if not esp32_state.is_connected():
            logger.warning("ESP32 未连接，控制命令可能无效")
        
        # 确保值在有效范围内
        target_throttle = max(-100, min(100, target_throttle))
        target_steering = max(-100, min(100, target_steering))
        
        # 添加死区处理
        deadzone = 5  # 5% 的死区
        if abs(target_throttle) < deadzone:
            target_throttle = 0
        if abs(target_steering) < deadzone:
            target_steering = 0
            
        # 直接使用目标值
        current_throttle = target_throttle
        current_steering = target_steering
            
        last_command_time = time.time()
        
        # 构建发送给 ESP32 的控制命令
        control_command = f"c:{current_throttle:.1f},{current_steering:.1f}"
        logger.info(f"生成控制命令: {control_command}")
        
        return control_command
        
    except Exception as e:
        logger.error(f"控制端点错误: {str(e)}")
        return "Error", 500

@app.route('/esp32')
def esp32_endpoint():
    """
    ESP32 请求数据的端点
    """
    global current_throttle, current_steering, last_command_time
    
    try:
        esp32_state.update_seen()
        logger.info(f"ESP32 请求状态 - 计数: {esp32_state.request_count}")
        
        if time.time() - last_command_time > 1:
            throttle_pid.reset()
            steering_pid.reset()
            current_throttle = 0
            current_steering = 0
        
        control_command = f"c:{current_throttle:.1f},{current_steering:.1f}"
        esp32_state.last_command_sent = control_command
        
        logger.info(f"发送控制命令到 ESP32: {control_command}")
        return control_command
        
    except Exception as e:
        logger.error(f"ESP32 端点错误: {str(e)}")
        return "Error", 500

@app.route('/status')
def get_status():
    """
    获取系统状态
    """
    try:
        current_time = time.time()
        status = {
            'esp32_state': {
                'connected': esp32_state.is_connected(),
                'last_seen': round(esp32_state.last_seen, 2),
                'last_seen_seconds_ago': round(current_time - esp32_state.last_seen, 2),
                'request_count': esp32_state.request_count,
                'last_command': esp32_state.last_command_sent,
                'status': esp32_state.last_status
            },
            'last_command': {
                'current_throttle': round(float(current_throttle), 2),
                'current_steering': round(float(current_steering), 2),
                'target_throttle': round(float(target_throttle), 2),
                'target_steering': round(float(target_steering), 2)
            },
            'pid_state': {
                'throttle': {
                    'kp': round(float(throttle_pid.kp), 2),
                    'ki': round(float(throttle_pid.ki), 2),
                    'kd': round(float(throttle_pid.kd), 2),
                    'integral': round(float(throttle_pid.integral), 2)
                },
                'steering': {
                    'kp': round(float(steering_pid.kp), 2),
                    'ki': round(float(steering_pid.ki), 2),
                    'kd': round(float(steering_pid.kd), 2),
                    'integral': round(float(steering_pid.integral), 2)
                }
            },
            'time': round(current_time, 2)
        }
        logger.info(f"系统状态: {status}")
        return jsonify(status)
    except Exception as e:
        logger.error(f"获取状态时出错: {str(e)}")
        return jsonify({
            'error': str(e),
            'status': 'error'
        }), 500

@app.route('/pid/config', methods=['POST'])
def configure_pid():
    """
    动态配置 PID 参数
    """
    try:
        data = request.get_json()
        if 'throttle' in data:
            t = data['throttle']
            throttle_pid.kp = float(t.get('kp', throttle_pid.kp))
            throttle_pid.ki = float(t.get('ki', throttle_pid.ki))
            throttle_pid.kd = float(t.get('kd', throttle_pid.kd))
            
        if 'steering' in data:
            s = data['steering']
            steering_pid.kp = float(s.get('kp', steering_pid.kp))
            steering_pid.ki = float(s.get('ki', steering_pid.ki))
            steering_pid.kd = float(s.get('kd', steering_pid.kd))
            
        return {"status": "success"}
    except Exception as e:
        return {"status": "error", "message": str(e)}, 400

if __name__ == '__main__':
    logger.info("服务器启动")
    app.run(host='0.0.0.0', port=8080, debug=True)