#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiClient.h>
#include <base64.h>

// WiFi配置
const char* ssid = "iQOO Neo9 Pro";
const char* password = "wang123456.";

// Web服务器配置
WebServer server(80);
WiFiClient webClients[5];  // 最多支持5个客户端
int clientCount = 0;

// 1. 串口配置
#define RX_PIN 16   
#define TX_PIN 17
#define BAUD_RATE 115200
#define SERIAL_RX_BUFFER_SIZE 3072  // 增大串口接收缓冲区

// 2. 帧定义（与OpenMV完全一致）
const char FRAME_HEADER[] = "IMG_START";
const char FRAME_FOOTER[] = "IMG_END";
const size_t HEADER_LEN = strlen(FRAME_HEADER);
const size_t FOOTER_LEN = strlen(FRAME_FOOTER);

// 3. 队列与缓冲区配置
#define QUEUE_LEN 3
#define MAX_IMG_SIZE 4096

// 4. 全局变量（加volatile确保多任务可见性）
volatile QueueHandle_t imgQueue;
volatile uint8_t imgBuffer[MAX_IMG_SIZE];
volatile uint8_t receivedChecksum;  // 接收的校验和
volatile bool isBufferBusy = false; // 缓冲区锁
String boundary = "frameboundary";  // 用于MJPEG流的边界标识

// 计算校验和（与OpenMV完全一致：sum % 256）
uint8_t calcChecksum(const uint8_t* data, size_t len) {
  uint32_t sum = 0;
  for (size_t i = 0; i < len; i++) sum += data[i];
  return sum % 256;
}

// HTML页面内容
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
    #status { color: #666; margin-top: 20px; padding: 10px; background-color: #e9f7fe; border-radius: 4px; }
  </style>
</head>
<body>
  <h1>ESP32 实时图像监控</h1>
  <div id="imageContainer">
    <img id="liveImage" src="/stream" alt="实时图像">
  </div>
  <div id="status">连接中...</div>
</body>
</html>
)rawliteral";

// 发送图像到所有连接的客户端（使用 MJPEG 流）
void sendImageToWebClients(uint8_t* imageData, size_t imageSize) {
  // 为 MJPEG 流准备头部（严格格式）
  String header = "--" + boundary + "\r\n";
  header += "Content-Type: image/jpeg\r\n";
  header += "Content-Length: " + String(imageSize) + "\r\n";
  header += "Cache-Control: no-cache\r\n";
  header += "Pragma: no-cache\r\n";
  header += "\r\n";  // 头部与图像数据的分隔行（必须有）
  
  // 遍历所有客户端，确保数据写入成功
  for (int i = 0; i < clientCount; ) {
    if (webClients[i] && webClients[i].connected()) {
      // 先发送头部
      bool headerOk = webClients[i].print(header);
      // 再发送图像数据
      bool dataOk = webClients[i].write(imageData, imageSize);
      // 最后发送帧结束行
      bool endOk = webClients[i].print("\r\n");
      
      if (headerOk && dataOk && endOk) {
        i++; // 只有全部写入成功，才保留客户端
      } else {
        // 写入失败，移除客户端
        webClients[i] = webClients[clientCount - 1];
        clientCount--;
        Serial.printf("客户端 %d 写入失败，已移除\n", i);
      }
    } else {
      // 客户端断开，移除
      webClients[i] = webClients[clientCount - 1];
      clientCount--;
      Serial.printf("客户端 %d 已断开，已移除\n", i);
    }
  }
}

// -------------------------- 串口接收任务 --------------------------
void serialRxTask(void* param) {
  // 初始化串口2，增大接收缓冲区
  HardwareSerial Serial2(2);
  Serial2.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial2.setRxBufferSize(SERIAL_RX_BUFFER_SIZE);
  Serial.println("串口2初始化完成，等待OpenMV数据...");

  // 接收状态机变量
  enum { WAIT_HEADER, GET_IMG_SIZE, GET_CHECKSUM, GET_IMG_DATA, WAIT_FOOTER } state = WAIT_HEADER;
  uint8_t headerMatchCnt = 0;    // 帧头匹配计数
  uint8_t footerMatchCnt = 0;    // 帧尾匹配计数
  uint32_t imgSize = 0;          // 图像大小（4字节大端）
  uint32_t imgDataCnt = 0;       // 已接收图像字节数
  uint32_t footerWaitTimeout = 0;// 帧尾等待超时

  while (1) {
    // 持续读取串口数据
    while (Serial2.available() > 0) {
      uint8_t byte = Serial2.read();
      footerWaitTimeout = 0;

      switch (state) {
        // 状态1：匹配帧头（IMG_START）
        case WAIT_HEADER:
          if (byte == FRAME_HEADER[headerMatchCnt]) {
            headerMatchCnt++;
            if (headerMatchCnt == HEADER_LEN) {
              state = GET_IMG_SIZE;
              headerMatchCnt = 0;
              imgSize = 0;
              imgDataCnt = 0;
              isBufferBusy = true;
              Serial.printf("\n检测到帧头，准备接收图像...\n");
            }
          } else {
            headerMatchCnt = 0;
          }
          break;

        // 状态2：接收图像大小（4字节，大端模式）
        case GET_IMG_SIZE:
          imgSize = (imgSize << 8) | byte;
          imgDataCnt++;
          if (imgDataCnt == 4) {
            imgDataCnt = 0;
            if (imgSize > MAX_IMG_SIZE) {
              Serial.printf("图像过大（%u字节），丢弃！\n", imgSize);
              state = WAIT_HEADER;
              isBufferBusy = false;
            } else {
              state = GET_CHECKSUM;
              Serial.printf("图像大小：%u字节，准备接收校验和...\n", imgSize);
            }
          }
          break;

        // 状态3：接收校验和（1字节）
        case GET_CHECKSUM:
          receivedChecksum = byte;
          state = GET_IMG_DATA;
          Serial.printf("校验和：0x%02X，开始接收图像数据...\n", receivedChecksum);
          break;

        // 状态4：接收图像数据
        case GET_IMG_DATA:
          imgBuffer[imgDataCnt++] = byte;
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
            if (footerMatchCnt == FOOTER_LEN) {
              state = WAIT_HEADER;
              isBufferBusy = false;
              xQueueSend((QueueHandle_t)imgQueue, &imgSize, portMAX_DELAY);
              Serial.printf("帧尾匹配成功！已将图像放入处理队列\n");
            }
          } else {
            footerMatchCnt = (byte == FRAME_FOOTER[0]) ? 1 : 0;
          }
          break;
      }
    }

    // 帧尾等待超时处理
    if (state == WAIT_FOOTER) {
      footerWaitTimeout++;
      if (footerWaitTimeout > 500) {
        Serial.println("帧尾等待超时，丢弃当前帧！");
        state = WAIT_HEADER;
        isBufferBusy = false;
        footerWaitTimeout = 0;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(0.1));
  }
}

// -------------------------- 数据处理与Web推送任务 --------------------------
void dataProcTask(void* param) {
  uint32_t imgSize;
  while (1) {
    // 等待队列通知
    if (xQueueReceive((QueueHandle_t)imgQueue, &imgSize, portMAX_DELAY) == pdPASS) {
      // 等待缓冲区解锁
      while (isBufferBusy) vTaskDelay(pdMS_TO_TICKS(1));

      Serial.println("\n开始验证图像数据...");
      // 计算本地校验和并对比
      uint8_t localChecksum = calcChecksum((uint8_t*)imgBuffer, imgSize);
      if (localChecksum == receivedChecksum) {
        Serial.printf("校验和验证成功！（接收：0x%02X，计算：0x%02X）\n", receivedChecksum, localChecksum);
        
        // 发送图像到Web客户端
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

// 修改handleStream函数
void handleStream() {
  WiFiClient client = server.client();
  
  // 检查是否已达到最大客户端数
  if (clientCount >= 5) {
    server.send(503, "text/plain", "客户端数量已达上限");
    return;
  }
  
  // 发送正确的MJPEG流头部
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: multipart/x-mixed-replace; boundary=" + boundary);
  client.println("Access-Control-Allow-Origin: *");
  client.println("Connection: close");
  client.println();
  
  // 将客户端添加到列表
  webClients[clientCount] = client;
  clientCount++;
  
  Serial.printf("新的流客户端连接，当前连接数：%d\n", clientCount);
  
}

// Web服务器处理函数
void setupWebServer() {
  // 路由设置
  server.on("/", HTTP_GET, [](){
    server.send_P(200, "text/html", index_html);
  });
  
  server.on("/stream", HTTP_GET, handleStream);

  server.onNotFound([](){
    server.send(404, "text/plain", "Not found");
  });

  // 启动服务器
  server.begin();
  Serial.println("Web服务器已启动，端口 80");
}

// -------------------------- WiFi连接函数 --------------------------
void connectToWiFi() {
  Serial.printf("正在连接到 %s ", ssid);
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("");
  Serial.println("WiFi连接成功！");
  Serial.print("IP地址: ");
  Serial.println(WiFi.localIP());
}

// -------------------------- 初始化 --------------------------
void setup() {
  // 初始化调试串口
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("ESP32 图像接收器与Web监控初始化...");

  // 连接WiFi
  connectToWiFi();

  // 配置Web服务器
  setupWebServer();

  // 创建队列
  imgQueue = xQueueCreate(QUEUE_LEN, sizeof(uint32_t));
  if (imgQueue == NULL) {
    Serial.println("队列创建失败！");
    while (1);
  }

  // 创建FreeRTOS任务
  xTaskCreate(
    serialRxTask,    // 串口接收任务
    "SerialRx",      // 任务名称
    8192,            // 栈大小
    NULL,            // 传递参数
    3,               // 优先级
    NULL
  );

  xTaskCreate(
    dataProcTask,    // 数据处理任务
    "DataProc",      // 任务名称
    8192,            // 栈大小
    NULL,            // 传递参数
    2,               // 优先级
    NULL
  );

}

void loop() {
  // 处理Web请求
  server.handleClient();
  vTaskDelay(pdMS_TO_TICKS(1));
}
