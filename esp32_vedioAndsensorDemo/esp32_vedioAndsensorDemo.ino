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

// ========================== 1. 基础配置（WiFi、引脚、参数）==========================
// WiFi配置
const char* ssid = "iQOO Neo9 Pro";
const char* password = "wang123456.";

// Web服务器配置（仅保留图像流功能）
WebServer server(80);
WiFiClient webClients[5];  // 最多支持5个Web客户端
int clientCount = 0;
String boundary = "frameboundary";  // MJPEG流边界标识

// 串口2（与OpenMV通信）配置
#define RX_PIN 16   
#define TX_PIN 17
#define BAUD_RATE 115200
#define SERIAL_RX_BUFFER_SIZE 3072  // 增大串口接收缓冲区

// 超声波模块（HC-SR04）配置
#define TRIG_PIN 2    // 触发引脚
#define ECHO_PIN 4    // 回声引脚
#define MAX_DISTANCE 400  // 最大测距（cm）

// TCS34725颜色传感器配置（I2C）
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// ========================== 2. 图像相关全局变量 ===========================
const char FRAME_HEADER[] = "IMG_START";
const char FRAME_FOOTER[] = "IMG_END";
const size_t HEADER_LEN = strlen(FRAME_HEADER);
const size_t FOOTER_LEN = strlen(FRAME_FOOTER);
#define QUEUE_LEN 3
#define MAX_IMG_SIZE 4096
volatile QueueHandle_t imgQueue;  // 图像数据队列
volatile uint8_t imgBuffer[MAX_IMG_SIZE];  // 图像缓冲区
volatile uint8_t receivedChecksum;  // 接收的校验和
volatile bool isBufferBusy = false;  // 缓冲区占用标志

// ========================== 3. 工具函数 ===========================
// 计算校验和（与OpenMV一致：sum % 256）
uint8_t calcChecksum(const uint8_t* data, size_t len) {
  uint32_t sum = 0;
  for (size_t i = 0; i < len; i++) sum += data[i];
  return sum % 256;
}

// ========================== 4. Web图像流功能 ===========================
// HTML页面（仅保留图像监控，移除传感器数据显示）
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
  // 构建MJPEG帧头（严格遵循流格式）
  String header = "--" + boundary + "\r\n";
  header += "Content-Type: image/jpeg\r\n";
  header += "Content-Length: " + String(imageSize) + "\r\n";
  header += "Cache-Control: no-cache\r\n";
  header += "Pragma: no-cache\r\n";
  header += "\r\n";  // 帧头与图像数据的分隔行（必须保留）

  // 遍历客户端，移除断开/写入失败的连接
  for (int i = 0; i < clientCount; ) {
    if (webClients[i] && webClients[i].connected()) {
      // 依次发送帧头、图像数据、帧尾
      bool headerOk = webClients[i].print(header);
      bool dataOk = webClients[i].write(imageData, imageSize);
      bool endOk = webClients[i].print("\r\n");

      if (headerOk && dataOk && endOk) {
        i++;  // 全部发送成功，保留客户端
      } else {
        // 发送失败，移除客户端（用最后一个客户端覆盖当前位置）
        webClients[i] = webClients[clientCount - 1];
        clientCount--;
        Serial.printf("客户端 %d 写入失败，已移除\n", i);
      }
    } else {
      // 客户端断开，直接移除
      webClients[i] = webClients[clientCount - 1];
      clientCount--;
      Serial.printf("客户端 %d 已断开，已移除\n", i);
    }
  }
}

// 处理Web图像流请求
void handleStream() {
  // 限制客户端数量（避免资源耗尽）
  if (clientCount >= 5) {
    server.send(503, "text/plain", "客户端数量已达上限（最多5个）");
    return;
  }

  // 发送MJPEG流协议头（必须正确，否则浏览器无法解析）
  WiFiClient client = server.client();
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: multipart/x-mixed-replace; boundary=" + boundary);
  client.println("Access-Control-Allow-Origin: *");
  client.println("Connection: close");
  client.println();  // 协议头结束标志

  // 将新客户端加入列表
  webClients[clientCount] = client;
  clientCount++;
  Serial.printf("新的流客户端连接，当前连接数：%d\n", clientCount);
}

// 初始化Web服务器（仅保留图像主页和流接口）
void setupWebServer() {
  // 主页路由（返回图像监控页面）
  server.on("/", HTTP_GET, []() {
    server.send_P(200, "text/html", index_html);
  });
  // 图像流路由（提供实时图像）
  server.on("/stream", HTTP_GET, handleStream);
  // 404路由（无效请求处理）
  server.onNotFound([]() {
    server.send(404, "text/plain", "Not Found");
  });

  // 启动服务器
  server.begin();
  Serial.println("Web服务器已启动，端口 80");
}

// ========================== 5. WiFi连接函数 ===========================
void connectToWiFi() {
  Serial.printf("正在连接WiFi：%s ", ssid);
  WiFi.begin(ssid, password);

  // 等待连接成功
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // 连接成功后打印信息
  Serial.println("\nWiFi连接成功！");
  Serial.print("ESP32 IP地址：");
  Serial.println(WiFi.localIP());  // 访问该IP即可查看图像监控
}

// ========================== 6. FreeRTOS任务（功能拆分，保证实时性）==========================
// 任务1：串口接收OpenMV图像数据（独立任务，避免阻塞Web和传感器）
void serialRxTask(void* param) {
  // 初始化串口2（与OpenMV通信）
  HardwareSerial Serial2(2);
  Serial2.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial2.setRxBufferSize(SERIAL_RX_BUFFER_SIZE);  // 增大缓冲区，避免数据丢失
  Serial.println("串口2（OpenMV通信）初始化完成，等待图像数据...");

  // 状态机变量（处理帧结构：帧头→图像大小→校验和→图像数据→帧尾）
  enum { WAIT_HEADER, GET_IMG_SIZE, GET_CHECKSUM, GET_IMG_DATA, WAIT_FOOTER } state = WAIT_HEADER;
  uint8_t headerMatchCnt = 0;    // 帧头匹配计数器
  uint8_t footerMatchCnt = 0;    // 帧尾匹配计数器
  uint32_t imgSize = 0;          // 图像大小（4字节大端模式）
  uint32_t imgDataCnt = 0;       // 已接收图像字节数
  uint32_t footerWaitTimeout = 0;// 帧尾等待超时计数器

  while (1) {
    // 持续读取串口数据（有数据时处理）
    while (Serial2.available() > 0) {
      uint8_t byte = Serial2.read();
      footerWaitTimeout = 0;  // 重置超时计数器

      switch (state) {
        // 状态1：匹配帧头（IMG_START）
        case WAIT_HEADER:
          if (byte == FRAME_HEADER[headerMatchCnt]) {
            headerMatchCnt++;
            // 帧头完全匹配，切换到读取图像大小状态
            if (headerMatchCnt == HEADER_LEN) {
              state = GET_IMG_SIZE;
              headerMatchCnt = 0;
              imgSize = 0;
              imgDataCnt = 0;
              isBufferBusy = true;  // 锁定缓冲区
              Serial.printf("\n检测到图像帧头，准备接收...\n");
            }
          } else {
            headerMatchCnt = 0;  // 匹配失败，重置计数器
          }
          break;

        // 状态2：读取图像大小（4字节，大端模式：高位在前）
        case GET_IMG_SIZE:
          imgSize = (imgSize << 8) | byte;  // 拼接4字节大小
          imgDataCnt++;
          if (imgDataCnt == 4) {
            imgDataCnt = 0;
            // 检查图像大小是否超出缓冲区上限
            if (imgSize > MAX_IMG_SIZE) {
              Serial.printf("图像过大（%u字节），丢弃当前帧！\n", imgSize);
              state = WAIT_HEADER;
              isBufferBusy = false;  // 解锁缓冲区
            } else {
              state = GET_CHECKSUM;
              Serial.printf("图像大小：%u字节，准备接收校验和...\n", imgSize);
            }
          }
          break;

        // 状态3：读取校验和（1字节）
        case GET_CHECKSUM:
          receivedChecksum = byte;
          state = GET_IMG_DATA;
          Serial.printf("校验和：0x%02X，开始接收图像数据...\n", receivedChecksum);
          break;

        // 状态4：读取图像数据
        case GET_IMG_DATA:
          imgBuffer[imgDataCnt++] = byte;
          // 数据接收完成，切换到等待帧尾状态
          if (imgDataCnt == imgSize) {
            state = WAIT_FOOTER;
            footerMatchCnt = 0;
            Serial.printf("图像数据接收完成（%u字节），等待帧尾...\n", imgSize);
          }
          break;

        // 状态5：匹配帧尾（IMG_END）
        case WAIT_FOOTER:
          if (byte == FRAME_FOOTER[footerMatchCnt]) {
            footerMatchCnt++;
            // 帧尾完全匹配，将图像大小放入队列，等待处理
            if (footerMatchCnt == FOOTER_LEN) {
              state = WAIT_HEADER;
              isBufferBusy = false;  // 解锁缓冲区
              xQueueSend((QueueHandle_t)imgQueue, &imgSize, portMAX_DELAY);
              Serial.printf("帧尾匹配成功！已将图像放入处理队列\n");
            }
          } else {
            // 部分匹配，重置计数器（仅保留首字符匹配的情况）
            footerMatchCnt = (byte == FRAME_FOOTER[0]) ? 1 : 0;
          }
          break;
      }
    }

    // 帧尾等待超时处理（防止因数据丢失导致状态机卡死）
    if (state == WAIT_FOOTER) {
      footerWaitTimeout++;
      if (footerWaitTimeout > 500) {  // 超时阈值（约500ms）
        Serial.println("帧尾等待超时，丢弃当前帧！");
        state = WAIT_HEADER;
        isBufferBusy = false;
        footerWaitTimeout = 0;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(0.1));  // 让出CPU，降低占用率
  }
}

// 任务2：图像数据处理+Web推送（独立任务，与串口接收解耦）
void dataProcTask(void* param) {
  uint32_t imgSize;  // 存储从队列获取的图像大小
  while (1) {
    // 等待队列中的图像数据（阻塞直到有数据）
    if (xQueueReceive((QueueHandle_t)imgQueue, &imgSize, portMAX_DELAY) == pdPASS) {
      // 等待缓冲区解锁（避免与串口接收冲突）
      while (isBufferBusy) vTaskDelay(pdMS_TO_TICKS(1));

      // 校验和验证（确保图像数据完整）
      uint8_t localChecksum = calcChecksum((uint8_t*)imgBuffer, imgSize);
      if (localChecksum == receivedChecksum) {
        Serial.printf("校验和验证成功！（接收：0x%02X，计算：0x%02X）\n", receivedChecksum, localChecksum);
        
        // 有Web客户端时，推送图像流
        if (clientCount > 0) {
          sendImageToWebClients((uint8_t*)imgBuffer, imgSize);
          Serial.printf("已将图像发送到 %u 个Web客户端\n", clientCount);
        } else {
          Serial.println("没有连接的Web客户端，跳过图像发送");
        }
      } else {
        Serial.printf("校验和验证失败！（接收：0x%02X，计算：0x%02X）\n", receivedChecksum, localChecksum);
      }
    }
  }
}

// 任务3：超声波测距+颜色识别
void sensorTask(void* param) {
  // 初始化超声波模块
  NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);
  // 初始化颜色传感器
  if (!tcs.begin()) {
    Serial.println("无法找到TCS34725传感器!请检查接线(SDA=21, SCL=22)");
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));  // 卡死等待排查
  }
  Serial.println("TCS34725颜色传感器初始化成功");
  Serial.println("----------------------------------------");

  // 传感器数据变量
  uint16_t r, g, b, c;  // TCS34725原始数据（R/G/B/清除值）
  float red, green, blue;  // 归一化RGB值（消除环境光影响）
  unsigned int distance;  // 超声波测距结果

  while (1) {
    // 1. 读取超声波距离
    distance = sonar.ping_cm();

    // 2. 读取颜色数据；无效时置空
    tcs.getRawData(&r, &g, &b, &c);  // 读取RGB原始值
    // 归一化RGB（c为清除值，避免环境光干扰；c=0时置0防止除零）
    red = (float)r / c;
    green = (float)g / c;
    blue = (float)b / c;

    // 3. 串口输出传感器结果
    Serial.print("[传感器数据] ");
    // 输出距离
    if (distance == 0) {
      Serial.print("距离：超出测量范围");
    } else {
      Serial.printf("距离：%u cm", distance);
    }
    // 输出颜色
    Serial.printf(" | RGB原始值:(%u, %u, %u)", r, g, b);
  
    Serial.println();

    vTaskDelay(pdMS_TO_TICKS(10));  
  }
}

// ========================== 7. 初始化与主循环 ===========================
void setup() {
  // 初始化调试串口（用于打印日志）
  Serial.begin(115200);
  while (!Serial) delay(10);  // 等待串口就绪
  Serial.println("ESP32 图像监控+传感器测试初始化...");
  Wire.begin();

  // 1. 连接WiFi
  connectToWiFi();

  // 2. 初始化Web服务器
  setupWebServer();

  // 3. 创建图像数据队列（用于串口接收与数据处理任务间通信）
  imgQueue = xQueueCreate(QUEUE_LEN, sizeof(uint32_t));
  if (imgQueue == NULL) {
    Serial.println("图像队列创建失败！程序终止");
    while (1);  // 队列创建失败，卡死等待排查
  }

  // 4. 创建FreeRTOS任务（任务优先级：串口接收 > 数据处理 > 传感器）
  // 串口接收任务：优先级3（最高，避免数据丢失）
  xTaskCreate(
    serialRxTask,    // 任务函数
    "SerialRxTask",  // 任务名称
    8192,            // 栈大小（字节）
    NULL,            // 传递参数
    3,               // 优先级（0-24，越高越优先）
    NULL
  );

  // 数据处理任务：优先级2
  xTaskCreate(
    dataProcTask,    // 任务函数
    "DataProcTask",  // 任务名称
    8192,            // 栈大小
    NULL,            // 传递参数
    2,               // 优先级
    NULL
  );

  // 传感器任务：优先级1（最低，不影响图像功能）
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
  // 处理Web客户端请求
  server.handleClient();
  vTaskDelay(pdMS_TO_TICKS(1));  // 让出CPU，避免占用过高
}