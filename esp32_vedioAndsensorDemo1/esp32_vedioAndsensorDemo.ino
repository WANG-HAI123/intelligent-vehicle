#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiClient.h>
#include <base64.h>
// 传感器相关库
#include <NewPing.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>

// ========================== 1. 核心同步对象（互斥锁/队列）==========================
SemaphoreHandle_t imgBufferMutex;    // 图像缓冲区互斥锁（保护imgBuffer）
SemaphoreHandle_t serialTxMutex;     // 串口发送互斥锁（保护OpenMVSerial发送）
HardwareSerial OpenMVSerial(2);      // 全局串口2（与OpenMV通信，统一实例避免冲突）
volatile QueueHandle_t imgQueue;     // 图像数据队列（串口接收→Web推送解耦）

// ========================== 2. 基础配置（WiFi、引脚、参数）==========================
// WiFi配置
const char* ssid = "iQOO Neo9 Pro";
const char* password = "wang123456.";

// Web服务器配置
WebServer server(80);
WiFiClient webClients[5];  // 最多支持5个Web客户端
int clientCount = 0;
String boundary = "frameboundary";  // MJPEG流边界标识

// 串口2（与OpenMV通信）配置
#define RX_PIN 16   
#define TX_PIN 17
#define BAUD_RATE 115200
#define SERIAL_RX_BUFFER_SIZE 3072  // 增大串口接收缓冲区（适配图像数据）

// 超声波模块（HC-SR04）配置
#define TRIG_PIN 2    
#define ECHO_PIN 4    
#define MAX_DISTANCE 400  // 最大测距（cm）
#define OBSTACLE_THRESHOLD 15  // 障碍判定阈值（<15cm视为有障碍）

// 传感器→OpenMV数据协议
#define SENSOR_FRAME_HEADER 0xAA  // 帧头
#define SENSOR_FRAME_FOOTER 0x55  // 帧尾
#define CMD_ULTRASONIC 0x01       // 超声障碍指令（1=有障碍，0=无障碍）
#define CMD_COLOR_ID 0x02         // 颜色ID指令（1=黑色，0=白色，2=未知）
#define COLOR_BLACK 1             // 黑色标识
#define COLOR_WHITE 0             // 白色标识
#define COLOR_UNKNOWN 2           // 未知颜色标识

// TCS34725颜色传感器配置（I2C：SDA=21, SCL=22）
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// ========================== 3. 图像相关全局变量 ===========================
const char FRAME_HEADER[] = "IMG_START";
const char FRAME_FOOTER[] = "IMG_END";
const size_t HEADER_LEN = strlen(FRAME_HEADER);
const size_t FOOTER_LEN = strlen(FRAME_FOOTER);
#define QUEUE_LEN 3               // 图像队列长度（避免溢出）
#define MAX_IMG_SIZE 4096         // 最大图像尺寸（适配OpenMV输出）
volatile uint8_t imgBuffer[MAX_IMG_SIZE];  // 图像缓冲区
volatile uint8_t receivedChecksum;         // 接收的校验和（OpenMV发送）
volatile bool isBufferBusy = false;        // 缓冲区占用标志（避免读写冲突）

// ========================== 4. 工具函数 ===========================
// 计算校验和（与OpenMV一致：sum % 256）
uint8_t calcChecksum(const uint8_t* data, size_t len) {
  uint32_t sum = 0;
  for (size_t i = 0; i < len; i++) sum += data[i];
  return sum % 256;
}

// ========================== 5. Web图像流功能 ===========================
// HTML页面（仅保留图像监控）
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
  <meta charset="UTF-8"> 
  <title>ESP32 图像监控</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial; text-align: center; margin: 0; padding: 20px; background-color: #f0f0f0; }
    h1 { color: #333; }
    #imageContainer { margin: 20px auto; padding: 10px; background-color: white; border-radius: 8px; box-shadow: 0 0 10px rgba(0,0,0,0.1); }
    #liveImage { max-width: 100%; max-height: 600px; border: 1px solid #ddd; }
    #status { color: #666; margin-top: 20px; }
  </style>
</head>
<body>
  <h1>ESP32 实时图像监控</h1>
  <div id="imageContainer">
    <img id="liveImage" src="/stream" alt="实时图像">
  </div>
  <div id="status">连接状态：正常</div>
</body>
</html>
)rawliteral";

// 发送MJPEG图像流到Web客户端
void sendImageToWebClients(uint8_t* imageData, size_t imageSize) {
  static char headerBuffer[256];  // 静态缓冲区避免动态内存分配
  
  // 预构建MJPEG帧头（必须符合协议）
  snprintf(headerBuffer, sizeof(headerBuffer),
           "--%s\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
           boundary.c_str(), imageSize);
  
  // 清理断开的客户端，避免资源泄漏
  for (int i = 0; i < clientCount; ) {
    WiFiClient& client = webClients[i];
    
    if (!client || !client.connected()) {
      client.stop();
      // 用最后一个客户端覆盖当前位置（压缩数组）
      if (i < clientCount - 1) {
        webClients[i] = webClients[clientCount - 1];
      }
      clientCount--;
      Serial.printf("清理断开客户端，剩余: %d\n", clientCount);
      continue;
    }
    
    // 发送帧头+图像数据+帧尾
    bool success = true;
    success &= (client.print(headerBuffer) > 0);
    success &= (client.write(imageData, imageSize) == imageSize);
    success &= (client.print("\r\n") > 0);
    
    if (!success) {
      client.stop();
      if (i < clientCount - 1) {
        webClients[i] = webClients[clientCount - 1];
      }
      clientCount--;
      Serial.printf("发送失败清理客户端，剩余: %d\n", clientCount);
    } else {
      i++;  // 仅成功客户端递增索引
    }
  }
}

// 处理Web图像流请求（/stream接口）
void handleStream() {
  // 限制客户端数量（避免CPU/内存过载）
  if (clientCount >= 5) {
    server.send(503, "text/plain", "客户端数量已达上限（最多5个）");
    return;
  }

  // 发送MJPEG流协议头（浏览器解析关键）
  WiFiClient client = server.client();
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: multipart/x-mixed-replace; boundary=" + boundary);
  client.println("Access-Control-Allow-Origin: *");  // 允许跨域（方便前端调用）
  client.println("Connection: close");
  client.println();  // 协议头结束标志（必须空行）

  // 新增客户端到列表
  webClients[clientCount] = client;
  clientCount++;
  Serial.printf("新的流客户端连接，当前连接数：%d\n", clientCount);
}

// 初始化Web服务器（仅保留核心接口）
void setupWebServer() {
  // 主页路由（返回图像监控页面）
  server.on("/", HTTP_GET, []() {
    server.send_P(200, "text/html", index_html);
  });
  // 图像流路由（核心实时流接口）
  server.on("/stream", HTTP_GET, handleStream);
  // 404路由（无效请求处理）
  server.onNotFound([]() {
    server.send(404, "text/plain", "Not Found");
  });

  server.begin();
  Serial.println("Web服务器已启动，端口 80");
}

// ========================== 6. WiFi连接函数 ===========================
void connectToWiFi() {
  Serial.printf("正在连接WiFi：%s ", ssid);
  WiFi.begin(ssid, password);

  // 等待连接（超时自动重试，实际项目可加超时退出）
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // 连接成功反馈
  Serial.println("\nWiFi连接成功！");
  Serial.print("ESP32 IP地址：");
  Serial.println(WiFi.localIP());  // 访问该IP即可查看图像监控
}

// ========================== 7. FreeRTOS任务（功能解耦，保证实时性）==========================
// 任务1：串口接收OpenMV图像数据（高优先级，避免数据丢失）
void serialRxTask(void* param) {
  // 初始化串口2（仅初始化一次，全局复用）
  OpenMVSerial.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
  OpenMVSerial.setRxBufferSize(SERIAL_RX_BUFFER_SIZE);
  Serial.println("串口2（OpenMV通信）初始化完成，等待图像数据...");

  // 状态机（处理图像帧的完整接收）
  enum { WAIT_HEADER, GET_IMG_SIZE, GET_CHECKSUM, GET_IMG_DATA, WAIT_FOOTER } state = WAIT_HEADER;
  uint8_t headerMatchCnt = 0;    // 帧头匹配计数器
  uint8_t footerMatchCnt = 0;    // 帧尾匹配计数器
  uint32_t imgSize = 0;          // 图像尺寸（4字节，OpenMV发送）
  uint32_t imgDataCnt = 0;       // 已接收图像数据字节数
  uint32_t footerWaitTimeout = 0;// 帧尾等待超时计数器

  while (1) {
    // 有数据时处理（非阻塞读取）
    while (OpenMVSerial.available() > 0) {
      uint8_t byte = OpenMVSerial.read();
      footerWaitTimeout = 0;  // 重置超时计数器

      switch (state) {
        // 等待图像帧头（"IMG_START"）
        case WAIT_HEADER:
          if (byte == FRAME_HEADER[headerMatchCnt]) {
            headerMatchCnt++;
            if (headerMatchCnt == HEADER_LEN) {
              // 获取缓冲区锁，标记占用
              if (xSemaphoreTake(imgBufferMutex, portMAX_DELAY) == pdTRUE) {
                state = GET_IMG_SIZE;
                headerMatchCnt = 0;
                imgSize = 0;
                imgDataCnt = 0;
                isBufferBusy = true;
                Serial.printf("\n检测到图像帧头，准备接收...\n");
                xSemaphoreGive(imgBufferMutex);  // 立即释放（仅状态切换需保护）
              }
            }
          } else {
            headerMatchCnt = 0;  // 匹配失败，重置计数器
          }
          break;

        // 获取图像尺寸（4字节，大端序）
        case GET_IMG_SIZE:
          imgSize = (imgSize << 8) | byte;  // 拼接4字节尺寸
          imgDataCnt++;
          if (imgDataCnt == 4) {
            imgDataCnt = 0;
            // 校验图像尺寸（避免缓冲区溢出）
            if (imgSize > MAX_IMG_SIZE || imgSize == 0) {
              Serial.printf("图像尺寸异常（%u字节），丢弃当前帧！\n", imgSize);
              if (xSemaphoreTake(imgBufferMutex, portMAX_DELAY) == pdTRUE) {
                state = WAIT_HEADER;
                isBufferBusy = false;
                xSemaphoreGive(imgBufferMutex);
              }
            } else {
              state = GET_CHECKSUM;
              Serial.printf("图像大小：%u字节，准备接收校验和...\n", imgSize);
            }
          }
          break;

        // 获取校验和（1字节）
        case GET_CHECKSUM:
          receivedChecksum = byte;
          state = GET_IMG_DATA;
          Serial.printf("校验和：0x%02X，开始接收图像数据...\n", receivedChecksum);
          break;

        // 接收图像数据（填充缓冲区）
        case GET_IMG_DATA:
          if (xSemaphoreTake(imgBufferMutex, portMAX_DELAY) == pdTRUE) {
            imgBuffer[imgDataCnt++] = byte;  // 写入缓冲区
            xSemaphoreGive(imgBufferMutex);
            
            // 数据接收完成，切换到等待帧尾
            if (imgDataCnt == imgSize) {
              state = WAIT_FOOTER;
              footerMatchCnt = 0;
              Serial.printf("图像数据接收完成（%u字节），等待帧尾...\n", imgSize);
            }
          }
          break;

        // 等待图像帧尾（"IMG_END"）
        case WAIT_FOOTER:
          if (byte == FRAME_FOOTER[footerMatchCnt]) {
            footerMatchCnt++;
            if (footerMatchCnt == FOOTER_LEN) {
              // 帧接收完整，通知数据处理任务
              if (xSemaphoreTake(imgBufferMutex, portMAX_DELAY) == pdTRUE) {
                state = WAIT_HEADER;
                isBufferBusy = false;
                xQueueSend((QueueHandle_t)imgQueue, &imgSize, portMAX_DELAY);  // 发送尺寸到队列
                Serial.printf("帧尾匹配成功！已将图像放入处理队列\n");
                xSemaphoreGive(imgBufferMutex);
              }
            }
          } else {
            footerMatchCnt = (byte == FRAME_FOOTER[0]) ? 1 : 0;  // 部分匹配时保留进度
          }
          break;
      }
    }

    // 帧尾等待超时处理（防止状态机卡死）
    if (state == WAIT_FOOTER) {
      footerWaitTimeout++;
      if (footerWaitTimeout > 500) {  // 0.1ms*500=50ms超时
        Serial.println("帧尾等待超时，丢弃当前帧！");
        if (xSemaphoreTake(imgBufferMutex, portMAX_DELAY) == pdTRUE) {
          state = WAIT_HEADER;
          isBufferBusy = false;
          xSemaphoreGive(imgBufferMutex);
        }
        footerWaitTimeout = 0;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(0.1));  // 让出CPU，降低占用
  }
}

// 任务2：图像数据处理+Web推送（中优先级，解耦接收与推送）
void dataProcTask(void* param) {
  uint32_t imgSize;         // 从队列接收的图像尺寸
  uint8_t localChecksum;    // 本地计算的校验和

  while (1) {
    // 等待队列中的图像数据（阻塞直到有数据）
    if (xQueueReceive((QueueHandle_t)imgQueue, &imgSize, portMAX_DELAY) == pdPASS) {
      // 读取缓冲区并校验数据完整性
      if (xSemaphoreTake(imgBufferMutex, portMAX_DELAY) == pdTRUE) {
        localChecksum = calcChecksum((uint8_t*)imgBuffer, imgSize);
        
        // 校验和匹配（确保数据未损坏）
        if (localChecksum == receivedChecksum) {
          Serial.printf("校验和验证成功！（接收：0x%02X，计算：0x%02X）\n", 
                        receivedChecksum, localChecksum);
          
          // 有Web客户端时推送图像
          if (clientCount > 0) {
            sendImageToWebClients((uint8_t*)imgBuffer, imgSize);
            Serial.printf("已将图像发送到 %u 个Web客户端\n", clientCount);
          } else {
            Serial.println("没有连接的Web客户端，跳过图像发送");
          }
        } else {
          Serial.printf("校验和验证失败！（接收：0x%02X，计算：0x%02X）\n", 
                        receivedChecksum, localChecksum);
        }
        
        xSemaphoreGive(imgBufferMutex);
      }
    }
  }
}

// 任务3：超声波测距+颜色识别+数据发送（低优先级，不影响图像核心功能）
void sensorTask(void* param) {
  // 初始化传感器
  NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);  // 超声波模块
  if (!tcs.begin()) {  // 颜色传感器初始化（失败则卡死提示）
    Serial.println("无法找到TCS34725传感器!请检查接线(SDA=21, SCL=22)");
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
  }
  Serial.println("TCS34725颜色传感器初始化成功");
  Serial.println("----------------------------------------");

  // 传感器数据变量
  uint16_t r, g, b, c;  // TCS34725原始数据（R/G/B/清除值）
  float red, green, blue;  // 归一化RGB值（消除环境光影响）
  unsigned int distance;    // 超声波测距结果（cm）
  uint8_t colorID = COLOR_UNKNOWN;  // 颜色标识
  uint8_t isBlock = 0;              // 障碍标识（1=有障碍，0=无障碍）

  while (1) {
    // 1. 超声波测距（0表示超出测量范围）
    distance = sonar.ping_cm();
    // 障碍判定（距离有效且小于阈值视为有障碍）
    isBlock = (distance > 0 && distance < OBSTACLE_THRESHOLD) ? 1 : 0;

    // 2. 颜色传感器数据读取与处理
    tcs.getRawData(&r, &g, &b, &c);
    // 归一化RGB（避免环境光干扰，处理c=0的除零异常）
    if (c == 0) {
      red = green = blue = 0.0f;
    } else {
      red = (float)r / c;
      green = (float)g / c;
      blue = (float)b / c;
    }

    // 3. 颜色识别（基于归一化值，阈值可根据实际场景调整）
    if (r < 80 && g < 60 && b < 55) {  // 低亮度→黑色
      colorID = COLOR_BLACK;
    } else if (r > 150 && g > 140 && b > 100) {  // 高亮度→白色
      colorID = COLOR_WHITE;
    } else {  // 其他颜色→未知
      colorID = COLOR_UNKNOWN;
    }

    // // 4. 打印传感器调试信息
    // Serial.print("[传感器数据] ");
    // if (distance == 0) {
    //   Serial.print("距离：超出测量范围");
    // } else {
    //   Serial.printf("距离：%u cm（障碍：%s）", distance, isBlock ? "是" : "否");
    // }
    // Serial.printf(" | 颜色：%s（归一化RGB：%.2f, %.2f, %.2f）\n",
    //               colorID == COLOR_BLACK ? "黑色" : (colorID == COLOR_WHITE ? "白色" : "未知"),
    //               red, green, blue);

    // 5. 向OpenMV发送传感器数据（串口发送互斥锁保护）
    if (xSemaphoreTake(serialTxMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      // 发送障碍数据帧（4字节：帧头+指令+数据+帧尾）
      uint8_t ultrasonicFrame[4] = {SENSOR_FRAME_HEADER, CMD_ULTRASONIC, isBlock, SENSOR_FRAME_FOOTER};
      OpenMVSerial.write(ultrasonicFrame, sizeof(ultrasonicFrame));

      // 发送颜色数据帧（4字节：帧头+指令+数据+帧尾）
      uint8_t colorFrame[4] = {SENSOR_FRAME_HEADER, CMD_COLOR_ID, colorID, SENSOR_FRAME_FOOTER};
      OpenMVSerial.write(colorFrame, sizeof(colorFrame));

      xSemaphoreGive(serialTxMutex);
      // 调试日志（可选，可注释减少串口占用）
      Serial.printf("[发OpenMV] 障碍：%d | 颜色：%d\n", isBlock, colorID);
    } else {
      Serial.println("[警告] 获取串口发送锁失败，跳过OpenMV数据发送");
    }
    Serial.println();

    vTaskDelay(pdMS_TO_TICKS(100));  // 100ms采样一次（传感器无需高频）
  }
}

// ========================== 8. 初始化与主循环 ===========================
void setup() {
  // 初始化调试串口（用于打印日志）
  Serial.begin(115200);
  while (!Serial) delay(10);  // 等待串口监视器连接
  Serial.println("ESP32 图像监控+传感器系统初始化...");
  Wire.begin();  // 初始化I2C（颜色传感器依赖）

  // 1. 创建同步对象（互斥锁+队列）
  imgBufferMutex = xSemaphoreCreateMutex();
  serialTxMutex = xSemaphoreCreateMutex();
  imgQueue = xQueueCreate(QUEUE_LEN, sizeof(uint32_t));
  // 检查同步对象创建结果（失败则卡死提示）
  if (imgBufferMutex == NULL || serialTxMutex == NULL || imgQueue == NULL) {
    Serial.println("同步对象（互斥锁/队列）创建失败！程序终止");
    while (1);
  }

  // 2. 连接WiFi与初始化Web服务器
  connectToWiFi();
  setupWebServer();

  // 3. 创建FreeRTOS任务（优先级：串口接收 > 数据处理 > 传感器）
  // 任务1：串口接收（高优先级3，避免图像数据丢失）
  xTaskCreate(
    serialRxTask,    // 任务函数
    "SerialRxTask",  // 任务名称
    8192,            // 栈大小（字节，图像处理需较大栈）
    NULL,            // 传递参数
    3,               // 优先级（0-24，越高越优先）
    NULL
  );

  // 任务2：数据处理（中优先级2，平衡实时性与资源）
  xTaskCreate(
    dataProcTask,    // 任务函数
    "DataProcTask",  // 任务名称
    8192,            // 栈大小
    NULL,            // 传递参数
    2,               // 优先级
    NULL
  );

  // 任务3：传感器（低优先级1，不抢占核心功能）
  xTaskCreate(
    sensorTask,      // 任务函数
    "SensorTask",    // 任务名称
    4096,            // 栈大小
    NULL,            // 传递参数
    1,               // 优先级
    NULL
  );
}

void loop() {
  // 处理Web客户端请求（非阻塞，与FreeRTOS任务兼容）
  server.handleClient();
  vTaskDelay(pdMS_TO_TICKS(1));  // 让出CPU，避免主循环占用过高
}