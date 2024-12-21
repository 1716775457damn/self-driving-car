#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <stdio.h>
#include <esp_task_wdt.h>

/**
 * 舵机控制相关
 */
class LedcServo
{
public:
  float freq = 50;
  int resolution = 8;
  float pwmBaseScale;
  float pwmMin;
  float pwmMax;
  int channel;
  int scale = 1;
  void setup(float freq, int resolution, int channel);
  /* 0 < scale <= 1 */
  void setScale(float scale);
  void attachPin(int pin);
  void write(float value, float min, float max);
};
void LedcServo::setup(float f, int r, int c)
{
  this->freq = f;
  this->resolution = r;
  this->pwmBaseScale = this->freq * pow(2, this->resolution) / 1000;
  this->pwmMin = 1 * this->pwmBaseScale;
  this->pwmMax = 2 * this->pwmBaseScale;
  this->channel = c;
  ledcSetup(this->channel, this->freq, this->resolution);
}
void LedcServo::setScale(float s)
{
  if (s <= 0)
    throw "s 不能小于等于0";
  if (s > 1)
    throw "s 不能大于1";

  this->scale = s;
  this->pwmMin = (1.5 - s * 0.5) * this->pwmBaseScale;
  this->pwmMax = (1.5 + s * 0.5) * this->pwmBaseScale;
}
void LedcServo::attachPin(int p)
{
  ledcAttachPin(p, this->channel);
}
void LedcServo::write(float v, float min, float max)
{
  float newV = v;
  if (max > min)
  {
    if (v > max)
      newV = max;
    if (v < min)
      newV = min;
  }
  else
  {
    if (v > min)
      newV = min;
    if (v < max)
      newV = max;
  }
  ledcWrite(this->channel, map(newV, min, max, this->pwmMin, this->pwmMax));
}

float sendMin = -100;
float sendMax = 100;
float sendHaf = 0;

float rxC1 = sendHaf;
float rxC2 = sendHaf;

// 设置为-1
float lastRxC1 = rxC1;
float lastRxC2 = rxC2;

LedcServo rxC1Servo;
LedcServo rxC1ServoHaf;
LedcServo rxC2Servo;
LedcServo rxC2ServoHaf;

/**
 * 接收信息的web server 监听80端口
 */
// WebServer server(80);

unsigned long timeNow = 0;
unsigned long lastDataTickTime = 0;

int LED_BUILTIN = 2;
bool ledShow = false;
int ledLoopTick = -1;

// 新增：定义你的 WiFi 网络信息
const char *ssid = "ovo";
const char *password = "twx20051";

// 新增：定义你电脑上运行的服务的 IP 地址和端口
const char *serverName = "192.168.50.254"; // 例如: "192.168.1.100"
const uint16_t serverPort = 8080;          // 你电脑上服务的端口

// void handleRoot()
// {
//   String c = server.arg("c");
//   // Serial.println(c.c_str());
//   sscanf(c.c_str(), "c:%f,%f", &rxC1, &rxC2);
//   // Serial.println(rxC1);
//   // Serial.println(rxC2);
//   lastDataTickTime = millis();
//   server.send(200, "text/plain", "success");
// }

// void registerEvent()
// {
//   server.on("/", handleRoot);
//
//   server.enableCORS();
//   server.begin();
//   Serial.println("HTTP server started");
// }

// 在文件开头添加电压监控相关定义
#define VOLTAGE_PIN 34  // 用于监控电压的 ADC 引脚
#define MIN_VOLTAGE 4.5 // 最小工作电压（根据实际情况调整）

// 添加重启计数和时间记录
RTC_DATA_ATTR int bootCount = 0;  // 存储在RTC内存中的重启计数
unsigned long lastResetTime = 0;   // 上次重置的时间

// 添加电调初始化和软启动相关定义
#define ESC_INIT_DELAY 2000  // 电调初始化延时
#define SOFT_START_STEPS 20   // 软启动步数
#define WDT_TIMEOUT 5        // 增加看门狗超时时间到5秒

// 在文件开头定义串口引脚
#define DEBUG_TX 17  // TX 引脚
#define DEBUG_RX 16  // RX 引脚（如果不需要接收可以不用）

// 在文件开头添加串口定义
#define SERIAL_DEBUG Serial2    // 使用 Serial2 作为调试串口
#define DEBUG_TX 17            // TX 引脚
#define DEBUG_RX 16            // RX 引脚

void setup()
{
  // 先初始化调试串口
  SERIAL_DEBUG.begin(115200, SERIAL_8N1, DEBUG_RX, DEBUG_TX);
  delay(100);  // 等待串口稳定
  
  // 发送一些测试数据
  SERIAL_DEBUG.println("\n\n");
  SERIAL_DEBUG.println("=================");
  SERIAL_DEBUG.println("ESP32 Starting...");
  SERIAL_DEBUG.println("=================");
  
  // 测试串口是否工作
  for(int i = 0; i < 10; i++) {
    SERIAL_DEBUG.printf("Test output %d\n", i);
    delay(100);
  }
  
  // 记录启动信息
  bootCount++;
  SERIAL_DEBUG.printf("启动次数: %d\n", bootCount);
  
  // 配置看门狗，增加超时时间
  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);
  
  // 初始化舵机时添加延时和错误检查
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // 添加错误检查的舵机初始化
  bool servoInitSuccess = true;
  
  try {
    // 电调初始化过程
    SERIAL_DEBUG.println("开始电调初始化...");
    
    // 先将所有舵机设置到中点位置
    rxC1Servo.setup(100, 10, 8);
    rxC1Servo.attachPin(23);
    rxC1Servo.setScale(1);
    rxC1Servo.write(0, -1, 1);
    
    rxC1ServoHaf.setup(100, 10, 9);
    rxC1ServoHaf.attachPin(22);
    rxC1ServoHaf.setScale(0.3);
    rxC1ServoHaf.write(0, -1, 1);
    
    // 等待电调启动
    SERIAL_DEBUG.println("等待电调启动...");
    delay(ESC_INIT_DELAY);
    
    // 软启动测试
    SERIAL_DEBUG.println("执行电调软启动测试...");
    for(int i = 0; i < SOFT_START_STEPS; i++) {
        float testValue = (float)i / SOFT_START_STEPS * 20.0;  // 最大测试到20%油门
        rxC1Servo.write(testValue, sendMax, sendMin);
        rxC1ServoHaf.write(testValue, sendMax, sendMin);
        delay(100);
        esp_task_wdt_reset();  // 重置看门狗
    }
    
    // 回到中点
    rxC1Servo.write(0, -1, 1);
    rxC1ServoHaf.write(0, -1, 1);
    
    // 初始化方向舵机
    rxC2Servo.setup(500, 10, 10);
    rxC2Servo.attachPin(26);
    rxC2Servo.setScale(0.7);
    rxC2Servo.write(0, -1, 1);
    
    rxC2ServoHaf.setup(500, 10, 11);
    rxC2ServoHaf.attachPin(25);
    rxC2ServoHaf.setScale(0.3);
    rxC2ServoHaf.write(0, -1, 1);
  } catch (...) {
    servoInitSuccess = false;
    SERIAL_DEBUG.println("舵机初始化失败！");
  }

  // WiFi连接添加超时和重试机制
  SERIAL_DEBUG.print("正在连接WiFi: ");
  SERIAL_DEBUG.println(ssid);
  
  WiFi.begin(ssid, password);
  int wifiRetries = 0;
  while (WiFi.status() != WL_CONNECTED && wifiRetries < 20) {
    delay(500);
    SERIAL_DEBUG.print(".");
    wifiRetries++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    SERIAL_DEBUG.println("\nWiFi连接成功");
    SERIAL_DEBUG.print("IP地址: ");
    SERIAL_DEBUG.println(WiFi.localIP());
  } else {
    SERIAL_DEBUG.println("\nWiFi连接失败，准备重启...");
    ESP.restart();
  }

  if (MDNS.begin("esp32-car")) {
    SERIAL_DEBUG.println("MDNS响应器启动成功");
  }

  // 初始化完成后重置看门狗计时器
  esp_task_wdt_reset();
  lastResetTime = millis();
}

// float mappedThrottle = 0;
// float mappedSteering = 0;
// float deadzone = 10;

// 添加电压检测函数
float getVoltage() {
    // 读取 ADC 值并转换为实际电压
    // 注意：需要根据你的分压电路调整计算公式
    int rawValue = analogRead(VOLTAGE_PIN);
    float voltage = (rawValue * 3.3 * 2) / 4095.0;  // 假设使用分压器，实际比例需要调整
    return voltage;
}

void updateServo()
{
  try {
    static float lastSafeRxC1 = 0;  // 记录上一次安全的油门值
    float maxOutput = 80.0;         // 最大输出限制
    float maxChange = 5.0;          // 每次最大变化量（降低）
    
    if (lastRxC1 != rxC1) {
      float limitedRxC1 = constrain(rxC1, -maxOutput, maxOutput);
      
      // 计算变化量
      float change = limitedRxC1 - lastSafeRxC1;
      // 限制变化率
      if (abs(change) > maxChange) {
        limitedRxC1 = lastSafeRxC1 + (change > 0 ? maxChange : -maxChange);
      }
      
      // 软启动保护
      if (abs(limitedRxC1) > 20 && abs(lastSafeRxC1) < 10) {
        // 如果从低油门突然到高油门，强制使用渐变
        limitedRxC1 = lastSafeRxC1 + (limitedRxC1 > 0 ? maxChange : -maxChange);
      }
      
      SERIAL_DEBUG.printf("更新油门舵机: %.2f -> %.2f (限制后: %.2f)\n", 
                   lastRxC1, rxC1, limitedRxC1);
      
      // 分两步更新油门，减少电流冲击
      rxC1Servo.write(limitedRxC1, sendMax, sendMin);
      delay(5);
      rxC1ServoHaf.write(limitedRxC1, sendMax, sendMin);
      
      lastSafeRxC1 = limitedRxC1;
      lastRxC1 = rxC1;
    }
    
    if (lastRxC2 != rxC2) {
      float limitedRxC2 = constrain(rxC2, -maxOutput, maxOutput);
      float maxChange = 10.0;
      limitedRxC2 = constrain(limitedRxC2, lastRxC2 - maxChange, lastRxC2 + maxChange);
      
      rxC2Servo.write(limitedRxC2, sendMax, sendMin);
      delay(5);
      rxC2ServoHaf.write(limitedRxC2, sendMax, sendMin);
      lastRxC2 = limitedRxC2;
    }
  } catch (...) {
    SERIAL_DEBUG.println("舵机更新出错！");
  }
}

// 添加一个串口测试函数
void testSerial() {
    static unsigned long lastTestTime = 0;
    if(millis() - lastTestTime > 1000) {  // 每秒发送一次测试数据
        lastTestTime = millis();
        SERIAL_DEBUG.printf("Uptime: %lu ms\n", millis());
    }
}

void loop()
{
  // 在 loop 开始时调用测试函数
  testSerial();
  
  // 定期喂狗
  if (millis() - lastResetTime >= 1000) {
    esp_task_wdt_reset();
    lastResetTime = millis();
  }

  timeNow = millis();

  // 监控电压
  float voltage = getVoltage();
  if (voltage < MIN_VOLTAGE) {
      SERIAL_DEBUG.printf("警告：电压过低 %.2fV\n", voltage);
  }

  // 每隔一段时间向服务器发送数据并接收指令
  if (timeNow - lastDataTickTime > 100)
  {
    lastDataTickTime = timeNow;

    // 添加电压监控日志
    SERIAL_DEBUG.printf("当前电压: %.2fV\n", voltage);

    // 检查 WiFi 连接状态
    if (WiFi.status() != WL_CONNECTED) {
        SERIAL_DEBUG.println("WiFi 连接断开，尝试重连...");
        WiFi.reconnect();
        delay(100);  // 给一些重连的时间
        return;
    }

    WiFiClient client;
    if (client.connect(serverName, serverPort))
    {
      SERIAL_DEBUG.println("\n正在连接服务器...");

      // 发送数据到服务器
      String request = String("GET /esp32?status=ok&voltage=") + voltage + " HTTP/1.1\r\n" +
                      "Host: " + serverName + "\r\n" +
                      "Connection: close\r\n\r\n";
      
      client.print(request);
      SERIAL_DEBUG.println("已发送请求");

      // 等待服务器响应
      unsigned long timeout = millis();
      while (client.connected() && !client.available())
      {
        if (millis() - timeout > 1000)
        {
          SERIAL_DEBUG.println("服务器响应超时！");
          client.stop();
          return;
        }
      }

      // 读取并处理响应
      bool headerEnd = false;
      String response = "";
      
      while (client.available())
      {
        String line = client.readStringUntil('\n');
        line.trim();
        
        if (!headerEnd)
        {
          if (line.length() == 0)
          {
            headerEnd = true;
            SERIAL_DEBUG.println("HTTP 头部处理完成");
          }
          continue;
        }
        
        if (line.length() > 0)
        {
          response = line;
          SERIAL_DEBUG.print("收到控制命令: ");
          SERIAL_DEBUG.println(response);
          
          if (response.startsWith("c:"))
          {
            float newRxC1, newRxC2;
            if (sscanf(response.substring(2).c_str(), "%f,%f", &newRxC1, &newRxC2) == 2)
            {
              SERIAL_DEBUG.printf("解到的值 - 油门: %.2f, 转向: %.2f\n", newRxC1, newRxC2);
              
              // 只有当值发生变化时才更新
              if (abs(newRxC1 - rxC1) > 0.1 || abs(newRxC2 - rxC2) > 0.1)
              {
                rxC1 = newRxC1;
                rxC2 = newRxC2;
                updateServo();
                SERIAL_DEBUG.println("舵机值已更新");
              }
            }
            else
            {
              SERIAL_DEBUG.println("命令格式错误！");
            }
          }
        }
      }
      
      client.stop();
      SERIAL_DEBUG.println("连接已关闭");
    }
    else
    {
      SERIAL_DEBUG.println("无法连接到服务器！");
    }
  }

  // 超时处理添加渐变
  if (timeNow - lastDataTickTime > 1000)
  {
    if (rxC1 != sendHaf || rxC2 != sendHaf)
    {
      SERIAL_DEBUG.println("连接超时，舵机渐变归中");
      // 渐变回中
      float step = 5.0;  // 每次变化的步长
      if (rxC1 > sendHaf) rxC1 -= step;
      if (rxC1 < sendHaf) rxC1 += step;
      if (rxC2 > sendHaf) rxC2 -= step;
      if (rxC2 < sendHaf) rxC2 += step;
      
      updateServo();
    }

    // LED 闪烁逻辑
    ledLoopTick++;
    if (ledLoopTick >= 50)
    {
      ledLoopTick = 0;
      ledShow = !ledShow;
      digitalWrite(LED_BUILTIN, ledShow ? HIGH : LOW);
    }
  }
  else
  {
    if (!ledShow)
    {
      ledShow = true;
      digitalWrite(LED_BUILTIN, HIGH);
    }
  }

  delay(5);  // 保持较小的延时
}
