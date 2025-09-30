#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiClient.h>
#include <base64.h>
#include <NewPing.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>

// 联合体用于float与字节数组转换
union FloatByteUnion {
  float f;
  uint8_t b[4];
};

// ========================== 1. 核心同步对象 ===========================
SemaphoreHandle_t imgBufferMutex;    // 图像缓冲区互斥锁
SemaphoreHandle_t pidDataMutex;      // PID数据互斥锁
HardwareSerial OpenMVSerial(2);      // 串口2（与OpenMV通信）
volatile QueueHandle_t imgQueue;     // 图像数据队列
volatile QueueHandle_t motorCmdQueue;// 电机命令队列

// ========================== 2. 基础配置 ===========================
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
#define SERIAL_RX_BUFFER_SIZE 4096  // 增大串口接收缓冲区

// 超声波模块（HC-SR04）配置
#define TRIG_PIN 2    
#define ECHO_PIN 4    
#define MAX_DISTANCE 400  // 最大测距（cm）
#define OBSTACLE_THRESHOLD 30  // 障碍判定阈值（<25cm视为有障碍）
#define OBSTACLE_CONFIRM 2     // 连续2次检测到障碍才确认（防抖）

// 颜色传感器配置
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
#define COLOR_BLACK 1     // 有效路径（黑线）
#define COLOR_WHITE 0     // 边界（白线）
#define COLOR_UNKNOWN 2   // 未知颜色
#define COLOR_STABLE 2    // 连续2次识别一致才确认（防抖）

// 电机配置
#define AIN1 27  // 右电机方向1
#define AIN2 26  // 右电机方向2
#define PWMA 25  // 右电机PWM
#define BIN1 12  // 左电机方向1
#define BIN2 13  // 左电机方向2
#define PWMB 14  // 左电机PWM
#define BASE_SPEED 80    // 巡线基础速度（0-255）

// 图像相关配置
const char FRAME_HEADER[] = "IMG_START";
const char FRAME_FOOTER[] = "IMG_END";
const size_t HEADER_LEN = strlen(FRAME_HEADER);
const size_t FOOTER_LEN = strlen(FRAME_FOOTER);
#define QUEUE_LEN 3               // 图像队列长度
#define MAX_IMG_SIZE 4096         // 最大图像尺寸
volatile uint8_t imgBuffer[MAX_IMG_SIZE];  // 图像缓冲区
volatile uint8_t receivedChecksum;         // 接收的校验和
volatile bool isBufferBusy = false;        // 缓冲区占用标志

// PID数据与帧结构配置
const char DATA_SEPARATOR[] = "||";            // 分隔符
const size_t SEPARATOR_LEN = strlen(DATA_SEPARATOR);
uint8_t g_pidRawData[16] = {0};                 // PID原始字节数据

// ========================== 3. 全局数据结构 ===========================
// OpenMV PID数据（解析后存储）
typedef struct {
  float rho_err;      // 巡线偏差（正=偏右，负=偏左）
  float theta_err;    // 巡线角度偏差
  float rho_output;   // 偏差PID输出（用于速度差）
  float theta_output; // 角度PID输出（用于转向）
  bool isValid;       // PID数据是否有效
} PIDData;
PIDData g_pidData = {0.0f, 0.0f, 0.0f, 0.0f, false};

// 传感器状态（防抖后）
typedef struct {
  bool hasObstacle;   // 是否有障碍（超声波）
  uint8_t color;      // 颜色（颜色传感器）
} SensorState;
SensorState g_sensorState = {false, COLOR_UNKNOWN};

// 电机命令
typedef enum {
  CMD_BRAKE, CMD_FORWARD, CMD_BACKWARD, CMD_LEFT, CMD_RIGHT
} MotorCmdType;
typedef struct {
  MotorCmdType cmd;
  uint8_t leftSpeed;  // 左电机速度
  uint8_t rightSpeed; // 右电机速度
} MotorCmd;

// 超声波传感器实例
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);

// ========================== 4. 工具函数 ===========================
// 计算校验和（sum % 256）
uint8_t calcChecksum(const uint8_t* data, size_t len) {
  uint32_t sum = 0;
  for (size_t i = 0; i < len; i++) sum += data[i];
  return sum % 256;
}

// 解析PID数据
void parsePIDData(uint8_t* rawData, PIDData* pidData) {
  FloatByteUnion u;
  
  // 解析rho_err（0-3字节）
  memcpy(u.b, rawData + 0, 4);
  pidData->rho_err = u.f;
  
  // 解析theta_err（4-7字节）
  memcpy(u.b, rawData + 4, 4);
  pidData->theta_err = u.f;
  
  // 解析rho_output（8-11字节）
  memcpy(u.b, rawData + 8, 4);
  pidData->rho_output = u.f;
  
  // 解析theta_output（12-15字节）
  memcpy(u.b, rawData + 12, 4);
  pidData->theta_output = u.f;
}

// 计算电机速度（基础速度 + PID补偿）
void calcMotorSpeed(int16_t* leftSpeed, int16_t* rightSpeed) {
  xSemaphoreTake(pidDataMutex, portMAX_DELAY);

  // 计算左右电机速度（基础速度 ± 补偿）
  *leftSpeed = BASE_SPEED + g_pidData.rho_output;
  *rightSpeed = BASE_SPEED - g_pidData.rho_output;
  
  // 限制速度范围（0-255）
  *leftSpeed = constrain(*leftSpeed, 0, 255);
  *rightSpeed = constrain(*rightSpeed, 0, 255);
  xSemaphoreGive(pidDataMutex);
}

// 传感器数据防抖（连续n次一致才更新状态）
template <typename T>
bool debounce(T current, T* last, uint8_t* count, uint8_t threshold) {
  if (current == *last) {
    *count = (*count < threshold) ? (*count + 1) : *count;
    return (*count >= threshold);
  } else {
    *last = current;
    *count = 1;
    return false;
  }
}

// ========================== 5. Web图像流功能 ===========================
// HTML页面
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
  <meta charset="UTF-8"> 
  <title>ESP32 图像监控与电机控制</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial; text-align: center; margin: 0; padding: 20px; background-color: #f0f0f0; }
    h1 { color: #333; }
    #imageContainer { margin: 20px auto; padding: 10px; background-color: white; border-radius: 8px; box-shadow: 0 0 10px rgba(0,0,0,0.1); }
    #liveImage { max-width: 100%; max-height: 600px; border: 1px solid #ddd; }
    #status { color: #666; margin-top: 20px; padding: 10px; background-color: white; border-radius: 8px; display: inline-block; }
  </style>
</head>
<body>
  <h1>ESP32 实时图像与传感器监控</h1>
  <div id="imageContainer">
    <img id="liveImage" src="/stream" alt="实时图像">
  </div>
  <div id="status">连接状态：正常 | 系统就绪</div>
</body>
</html>
)rawliteral";

// 发送MJPEG图像流到Web客户端
void sendImageToWebClients(uint8_t* imageData, size_t imageSize) {
  static char headerBuffer[256];
  
  snprintf(headerBuffer, sizeof(headerBuffer),
           "--%s\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
           boundary.c_str(), imageSize);
  
  // 清理断开的客户端
  for (int i = 0; i < clientCount; ) {
    WiFiClient& client = webClients[i];
    
    if (!client || !client.connected()) {
      client.stop();
      if (i < clientCount - 1) {
        webClients[i] = webClients[clientCount - 1];
      }
      clientCount--;
      Serial.printf("清理断开客户端，剩余: %d\n", clientCount);
      continue;
    }
    
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
      i++;
    }
  }
}

// 处理Web图像流请求
void handleStream() {
  if (clientCount >= 5) {
    server.send(503, "text/plain", "客户端数量已达上限（最多5个）");
    return;
  }

  WiFiClient client = server.client();
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: multipart/x-mixed-replace; boundary=" + boundary);
  client.println("Access-Control-Allow-Origin: *");
  client.println("Connection: close");
  client.println();

  webClients[clientCount] = client;
  clientCount++;
  Serial.printf("新的流客户端连接，当前连接数：%d\n", clientCount);
}

// 初始化Web服务器
void setupWebServer() {
  server.on("/", HTTP_GET, []() {
    server.send_P(200, "text/html", index_html);
  });
  server.on("/stream", HTTP_GET, handleStream);
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

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi连接成功！");
  Serial.print("ESP32 IP地址：");
  Serial.println(WiFi.localIP());
}

// ========================== 7. 多传感器融合决策任务 ===========================
// 新增：绕障状态枚举（放在全局数据结构定义处）
typedef enum {
  NORMAL,           // 正常巡线状态
  OBSTACLE_AVOID,    // 障碍绕障状态
  FINISHED             
} SystemState;
SystemState g_systemState = NORMAL;  // 初始状态为正常巡线

//绕障流程计时变量（避免硬编码延迟）
#define AVOID_STOP_DELAY 1000      // 刹车后停顿时间（ms）
#define AVOID_BACK_DELAY 800       // 后退时间（ms）
#define AVOID_TURN_DELAY 1000      // 第一次转向（右转）时间（ms）
#define AVOID_FORWARD_DELAY 700   // 前进绕障时间（ms）
#define AVOID_LEFT_ADJUST_DELAY 500 // 左转调整时间（ms）- 扩大视野找线
#define AVOID_CHECK_DELAY 600      //调整后停留检测时间（ms）
uint32_t avoidStartTime = 0;        // 绕障流程开始时间


void sensorFusionTask(void* param) {
  // 新增：白色区域识别计数器（0:未识别 1:第一次识别 2:第二次识别）
  uint8_t whiteDetectCount = 0;
  uint8_t blackDetectCount = 0;

  
  // 原有防抖变量
  bool lastObstacle = false;
  uint8_t obstacleCount = 0;
  uint8_t lastColor = COLOR_UNKNOWN;
  uint8_t colorCount = 0;
  
  // 原有电机速度变量
  int16_t leftSpeed = 0, rightSpeed = 0;
  MotorCmd cmd;

  while (1) {
    // -------------------------- 第一步：传感器数据采集与防抖 --------------------------
    // 1. 超声波障碍检测（防抖）
    unsigned int distance = sonar.ping_cm();
    bool currentObstacle = (distance > 0 && distance < OBSTACLE_THRESHOLD);
    if (debounce(currentObstacle, &lastObstacle, &obstacleCount, OBSTACLE_CONFIRM)) {
      g_sensorState.hasObstacle = currentObstacle;
    }
    // Serial.printf("g_sensorState.hasObstacle:%d\n", g_sensorState.hasObstacle);

    // 2. 颜色传感器数据（防抖）
    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);
    uint8_t currentColor = COLOR_UNKNOWN;
    Serial.printf("[颜色检测] r:%d, g:%d, b:%d, c%d\n", r, g, b, c);
    if (c > 0) {
      if (r < 80 && g < 60 && b < 20) currentColor = COLOR_BLACK;
      else if (r > 150 && g > 140 && b > 100) currentColor = COLOR_WHITE;
      else currentColor = COLOR_UNKNOWN;
    }
    
    // 颜色稳定变化时更新计数
    // if (debounce(currentColor, &lastColor, &colorCount, COLOR_STABLE)) {
    //   g_sensorState.color = currentColor;
    //   // 当从非白色变为白色时，递增计数器
    //   if (currentColor == COLOR_WHITE && lastColor != COLOR_WHITE) {
    //     whiteDetectCount++;
    //     Serial.printf("[颜色检测] 第%d次识别到白色区域\n", whiteDetectCount);
    //   }
    // }
    g_sensorState.color = currentColor;
    if (currentColor == COLOR_WHITE && lastColor != COLOR_WHITE) {
      whiteDetectCount++;
      // Serial.printf("[临时测试] 第%d次识别到白色区域\n", whiteDetectCount);
    }else if(currentColor == COLOR_BLACK && lastColor != COLOR_BLACK){
      blackDetectCount++;
    }
    lastColor = currentColor; 
    Serial.printf("g_sensorState.color:%d,黑色触碰次数：%d\n", g_sensorState.color,blackDetectCount);

    Serial.printf("g_sensorState.color:%d,白色触碰次数：%d\n", g_sensorState.color,whiteDetectCount);

    // 3. PID数据有效性判断
    xSemaphoreTake(pidDataMutex, portMAX_DELAY);
    bool pidValid = g_pidData.isValid;
    xSemaphoreGive(pidDataMutex);


    // -------------------------- 第二步：系统状态机决策 --------------------------
    // switch (g_systemState) {
    //   // 状态1：正常巡线
    //   case NORMAL:{
    //     if (g_sensorState.hasObstacle) {
    //       if(g_sensorState.color != COLOR_WHITE){
    //         // 检测到障碍→切换到绕障状态，紧急刹车
    //         g_systemState = OBSTACLE_AVOID;
    //         avoidStartTime = millis();
    //       }
    //       cmd.cmd = CMD_BRAKE;
    //       Serial.println("[融合决策] 检测到障碍→进入绕障状态，紧急刹车");
    //     } else {
    //       // 无障→根据白色识别次数处理（关键修改）
    //       if (g_sensorState.color != COLOR_WHITE) {
    //         // 非白色区域，正常巡线
    //         calcMotorSpeed(&leftSpeed, &rightSpeed);
    //         cmd.cmd = CMD_FORWARD;
    //         cmd.leftSpeed = leftSpeed;
    //         cmd.rightSpeed = rightSpeed;
    //         Serial.printf("[融合决策] 正常巡线→左速：%d，右速：%d\n", 
    //                       leftSpeed, rightSpeed);
    //       } else {
    //         // 白色区域，根据识别次数处理
    //         if (whiteDetectCount == 1) {
    //           // 第一次识别白色（起始点），正常发车
    //           calcMotorSpeed(&leftSpeed, &rightSpeed);
    //           cmd.cmd = CMD_FORWARD;
    //           cmd.leftSpeed = leftSpeed;
    //           cmd.rightSpeed = rightSpeed;
    //           Serial.println("[融合决策] 第一次到达起始点，开始发车");
    //         } else if (whiteDetectCount >= 2) {
    //           // 第二次识别白色，停车结束行程
    //           cmd.cmd = CMD_BRAKE;
    //           Serial.println("[融合决策] 第二次到达起始点，结束行程");
    //           g_systemState = FINISHED; 
    //         }
    //       }
    //     }
    //     break;
    //   }

    //   // 状态2：障碍绕障
    //   case OBSTACLE_AVOID:{
    //     uint32_t avoidElapsed = millis() - avoidStartTime;

    //     // 步骤1：刹车后停顿1秒（确认障碍）
    //     if (avoidElapsed < AVOID_STOP_DELAY) {
    //       cmd.cmd = CMD_BRAKE;
    //       Serial.println("[绕障流程] 步骤1：刹车停顿，确认障碍");
    //     }

    //     // 步骤3：右转1.2秒（避开障碍主体）
    //     else if (AVOID_STOP_DELAY < avoidElapsed && avoidElapsed < AVOID_STOP_DELAY + AVOID_TURN_DELAY) {
    //       cmd.cmd = CMD_RIGHT;
    //       cmd.leftSpeed = BASE_SPEED*1.5;    // 左快右停→右转
    //       cmd.rightSpeed = BASE_SPEED*1.5;
    //       Serial.println("[绕障流程] 步骤3：右转，避开障碍物主体");
    //     }

    //     // 步骤4：前进1.5秒（越过障碍物）
    //     else if (AVOID_STOP_DELAY + AVOID_TURN_DELAY < avoidElapsed && 
    //             avoidElapsed < AVOID_STOP_DELAY + AVOID_TURN_DELAY + AVOID_FORWARD_DELAY) {
    //       cmd.cmd = CMD_FORWARD;
    //       cmd.leftSpeed = BASE_SPEED*1.5;
    //       cmd.rightSpeed = BASE_SPEED*1.5;
    //       Serial.println("[绕障流程] 步骤4：前进，越过障碍物");
    //     }

    //     // 步骤5：左转0.8秒（扩大视野，寻找循迹线）
    //     else if (avoidElapsed > AVOID_STOP_DELAY + AVOID_TURN_DELAY + AVOID_FORWARD_DELAY &&
    //       avoidElapsed < AVOID_STOP_DELAY + AVOID_TURN_DELAY + AVOID_FORWARD_DELAY + AVOID_LEFT_ADJUST_DELAY) {
    //       cmd.cmd = CMD_LEFT;
    //       cmd.leftSpeed = BASE_SPEED*1.5;            // 左停右快→左转（调整方向）
    //       cmd.rightSpeed = BASE_SPEED*1.5;
    //       Serial.println("[绕障流程] 步骤5：左转调整，扩大视野找线");
    //     }
    //     else if (avoidElapsed > AVOID_STOP_DELAY + AVOID_TURN_DELAY + AVOID_FORWARD_DELAY + AVOID_LEFT_ADJUST_DELAY && 
    //       avoidElapsed < AVOID_STOP_DELAY + AVOID_TURN_DELAY + AVOID_FORWARD_DELAY*1.5 + AVOID_LEFT_ADJUST_DELAY) {
    //       cmd.cmd = CMD_FORWARD;
    //       cmd.leftSpeed = BASE_SPEED*1.5;
    //       cmd.rightSpeed = BASE_SPEED*1.5;
    //       Serial.println("[绕障流程] 步骤6：继续前进");
    //     }
    //     // // 步骤6：停留0.5秒（给传感器时间检测循迹线）
    //     // else if (avoidElapsed > AVOID_STOP_DELAY + AVOID_TURN_DELAY + AVOID_FORWARD_DELAY + AVOID_LEFT_ADJUST_DELAY&&
    //     //   avoidElapsed < AVOID_STOP_DELAY + AVOID_TURN_DELAY + AVOID_FORWARD_DELAY + AVOID_LEFT_ADJUST_DELAY + AVOID_CHECK_DELAY) {
    //     //   cmd.cmd = CMD_BRAKE;
    //     //   Serial.println("[绕障流程] 步骤6：停留检测，等待传感器识别循迹线");
    //     // }

    //     // 步骤7：绕障完成→恢复正常巡线
    //     else {
    //       g_systemState = NORMAL;
    //       cmd.cmd = CMD_BRAKE;
    //       Serial.println("[绕障流程] 步骤7：绕障完成→恢复正常巡线状态");
    //     }
    //     break;
    //   }
    //   case FINISHED:{
    //     cmd.cmd = CMD_BRAKE;  // 保持刹车状态
    //     Serial.println("[系统状态] 行程已结束");
    //     vTaskDelay(pdMS_TO_TICKS(100)); // 降低循环频率
    //     break;
    //   }
    // }


    // -------------------------- 第三步：发送电机命令 --------------------------
    xQueueSend(motorCmdQueue, &cmd, pdMS_TO_TICKS(10));
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ========================== 8. 电机控制任务（执行命令） ===========================
void motorControlTask(void* param) {
  // 初始化电机引脚
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
    
  // 初始状态：停止
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);

  MotorCmd cmd;
  while (1) {
    if (xQueueReceive(motorCmdQueue, &cmd, portMAX_DELAY) == pdTRUE) {
      switch (cmd.cmd) {
        case CMD_BRAKE:
          // 刹车（两端高电平短路）
          digitalWrite(AIN1, HIGH);
          digitalWrite(AIN2, HIGH);
          digitalWrite(BIN1, HIGH);
          digitalWrite(BIN2, HIGH);
          analogWrite(PWMA, 0);
          analogWrite(PWMB, 0);
          Serial.println("电机已刹车");
          break;

        case CMD_FORWARD:
          // 前进（左右电机正转）
          digitalWrite(AIN1, HIGH);
          digitalWrite(AIN2, LOW);
          digitalWrite(BIN1, HIGH);
          digitalWrite(BIN2, LOW);
          analogWrite(PWMA, cmd.rightSpeed);
          analogWrite(PWMB, cmd.leftSpeed);
          Serial.printf("前进\n");
          break;

        case CMD_BACKWARD:
          // 后退（左右电机反转）
          digitalWrite(AIN1, LOW);
          digitalWrite(AIN2, HIGH);
          digitalWrite(BIN1, LOW);
          digitalWrite(BIN2, HIGH);
          analogWrite(PWMA, cmd.rightSpeed);
          analogWrite(PWMB, cmd.leftSpeed);
          Serial.printf("右转\n");
          break;

        case CMD_LEFT:
          // 左转（左停右转）
          digitalWrite(AIN1, HIGH);
          digitalWrite(AIN2, LOW);
          digitalWrite(BIN1, LOW);
          digitalWrite(BIN2, HIGH);
          analogWrite(PWMA, cmd.rightSpeed);
          analogWrite(PWMB, cmd.leftSpeed);
          Serial.printf("左转\n");
          break;

        case CMD_RIGHT:
          // 右转（右停左转）
          digitalWrite(AIN1, LOW);
          digitalWrite(AIN2, HIGH);
          digitalWrite(BIN1, HIGH);
          digitalWrite(BIN2, LOW);
          analogWrite(PWMA, cmd.rightSpeed);
          analogWrite(PWMB, cmd.leftSpeed);
          // Serial.printf("左速度：%d,右速度：%d\n", cmd.leftSpeed,cmd.rightSpeed);
          break;
      }
    }
  }
}

// ========================== 9. 串口接收OpenMV数据任务 ===========================
void serialRxTask(void* param) {
  OpenMVSerial.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
  OpenMVSerial.setRxBufferSize(SERIAL_RX_BUFFER_SIZE);
  Serial.println("串口2（OpenMV通信）初始化完成，等待图像数据...");

  // 状态机定义
  enum { 
    WAIT_HEADER,    // 等待帧头（IMG_START）
    GET_PID_DATA,   // 接收16字节PID原始数据
    WAIT_SEPARATOR, // 等待分隔符（||）
    GET_IMG_SIZE,   // 接收图像尺寸（4字节）
    GET_CHECKSUM,   // 接收校验和（1字节）
    GET_IMG_DATA,   // 接收图像数据
    WAIT_FOOTER     // 等待帧尾（IMG_END）
  } state = WAIT_HEADER;

  uint8_t headerMatchCnt = 0;    // 帧头匹配计数器
  uint8_t footerMatchCnt = 0;    // 帧尾匹配计数器
  uint8_t sepMatchCnt = 0;       // 分隔符匹配计数器
  uint32_t imgSize = 0;          // 图像尺寸
  uint32_t imgDataCnt = 0;       // 已接收图像数据字节数
  uint32_t pidDataCnt = 0;       // 已接收PID数据字节数
  uint32_t timeoutCnt = 0;       // 超时计数器

  while (1) {
    while (OpenMVSerial.available() > 0) {
      uint8_t byte = OpenMVSerial.read();
      timeoutCnt = 0;

      switch (state) {
        // 1. 等待图像帧头（"IMG_START"）
        case WAIT_HEADER:
          if (byte == FRAME_HEADER[headerMatchCnt]) {
            headerMatchCnt++;
            if (headerMatchCnt == HEADER_LEN) {
              if (xSemaphoreTake(imgBufferMutex, portMAX_DELAY) == pdTRUE) {
                state = GET_PID_DATA;
                headerMatchCnt = 0;
                pidDataCnt = 0;
                sepMatchCnt = 0;
                imgSize = 0;
                imgDataCnt = 0;
                memset((void*)g_pidRawData, 0, 16);
                isBufferBusy = true;
                // Serial.printf("\n[接收] 检测到帧头，开始接收PID数据...\n");
                xSemaphoreGive(imgBufferMutex);
              }
            }
          } else {
            headerMatchCnt = 0;
          }
          break;

        // 2. 接收16字节PID原始数据
        case GET_PID_DATA:
          g_pidRawData[pidDataCnt++] = byte;
          if (pidDataCnt == 16) {
            state = WAIT_SEPARATOR;
            // Serial.printf("[接收] PID原始数据（16字节）接收完成，等待分隔符...\n");
          }
          break;

        // 3. 匹配分隔符（"||"）
        case WAIT_SEPARATOR:
          if (byte == DATA_SEPARATOR[sepMatchCnt]) {
            sepMatchCnt++;
            if (sepMatchCnt == SEPARATOR_LEN) {
              if (xSemaphoreTake(imgBufferMutex, portMAX_DELAY) == pdTRUE) {
                // 解析PID数据并标记为有效
                parsePIDData((uint8_t*)g_pidRawData, (PIDData*)&g_pidData);
                xSemaphoreTake(pidDataMutex, portMAX_DELAY);
                g_pidData.isValid = true;
                xSemaphoreGive(pidDataMutex);
                
                // Serial.printf("[PID解析] rho_err:%.2f | theta_err:%.2f | rho_out:%.2f | theta_out:%.2f\n",
                //               g_pidData.rho_err, g_pidData.theta_err,
                //               g_pidData.rho_output, g_pidData.theta_output);
                state = GET_IMG_SIZE;
                sepMatchCnt = 0;
                xSemaphoreGive(imgBufferMutex);
              }
            }
          } else {
            sepMatchCnt = (byte == DATA_SEPARATOR[0]) ? 1 : 0;
            if (sepMatchCnt == 0) {
              // Serial.println("[警告] 分隔符匹配失败，重置接收状态！");
              state = WAIT_HEADER;
              if (xSemaphoreTake(imgBufferMutex, portMAX_DELAY) == pdTRUE) {
                isBufferBusy = false;
                xSemaphoreGive(imgBufferMutex);
              }
            }
          }
          break;

        // 4. 接收图像尺寸（4字节，大端序）
        case GET_IMG_SIZE:
          imgSize = (imgSize << 8) | byte;
          imgDataCnt++;
          if (imgDataCnt == 4) {
            imgDataCnt = 0;
            if (imgSize > MAX_IMG_SIZE || imgSize == 0) {
              // Serial.printf("[警告] 图像尺寸异常（%u字节），丢弃当前帧！\n", imgSize);
              if (xSemaphoreTake(imgBufferMutex, portMAX_DELAY) == pdTRUE) {
                state = WAIT_HEADER;
                isBufferBusy = false;
                xSemaphoreGive(imgBufferMutex);
              }
            } else {
              state = GET_CHECKSUM;
              // Serial.printf("[接收] 图像大小：%u字节，准备接收校验和...\n", imgSize);
            }
          }
          break;

        // 5. 接收校验和（1字节）
        case GET_CHECKSUM:
          receivedChecksum = byte;
          state = GET_IMG_DATA;
          // Serial.printf("[接收] 校验和：0x%02X，开始接收图像数据...\n", receivedChecksum);
          break;

        // 6. 接收图像数据（填充缓冲区）
        case GET_IMG_DATA:
          if (xSemaphoreTake(imgBufferMutex, portMAX_DELAY) == pdTRUE) {
            imgBuffer[imgDataCnt++] = byte;
            xSemaphoreGive(imgBufferMutex);
            
            if (imgDataCnt == imgSize) {
              state = WAIT_FOOTER;
              footerMatchCnt = 0;
              // Serial.printf("[接收] 图像数据接收完成（%u字节），等待帧尾...\n", imgSize);
            }
          }
          break;

        // 7. 等待图像帧尾（"IMG_END"）
        case WAIT_FOOTER:
          if (byte == FRAME_FOOTER[footerMatchCnt]) {
            footerMatchCnt++;
            if (footerMatchCnt == FOOTER_LEN) {
              if (xSemaphoreTake(imgBufferMutex, portMAX_DELAY) == pdTRUE) {
                state = WAIT_HEADER;
                isBufferBusy = false;
                xQueueSend((QueueHandle_t)imgQueue, &imgSize, portMAX_DELAY);
                // Serial.printf("[接收] 帧尾匹配成功！已将图像放入处理队列\n");
                xSemaphoreGive(imgBufferMutex);
              }
            }
          } else {
            footerMatchCnt = (byte == FRAME_FOOTER[0]) ? 1 : 0;
          }
          break;
      }
    }

    // 超时处理
    timeoutCnt++;
    if (timeoutCnt > 1000) {  // 100ms超时
      // Serial.printf("[警告] 状态机超时（当前状态：%d），重置到帧头等待！\n", state);
      if (xSemaphoreTake(imgBufferMutex, portMAX_DELAY) == pdTRUE) {
        state = WAIT_HEADER;
        isBufferBusy = false;
        xSemaphoreGive(imgBufferMutex);
      }
      timeoutCnt = 0;
      headerMatchCnt = 0;
      footerMatchCnt = 0;
      sepMatchCnt = 0;
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ========================== 10. 图像数据处理+Web推送任务 ===========================
void dataProcTask(void* param) {
  uint32_t imgSize;
  uint8_t localChecksum;

  while (1) {
    if (xQueueReceive((QueueHandle_t)imgQueue, &imgSize, portMAX_DELAY) == pdPASS) {
      if (xSemaphoreTake(imgBufferMutex, portMAX_DELAY) == pdTRUE) {
        localChecksum = calcChecksum((uint8_t*)imgBuffer, imgSize);
        
        if (localChecksum == receivedChecksum) {
          // Serial.printf("校验和验证成功！（接收：0x%02X，计算：0x%02X）\n", 
                        // receivedChecksum, localChecksum);
          
          if (clientCount > 0) {
            sendImageToWebClients((uint8_t*)imgBuffer, imgSize);
            // Serial.printf("已将图像发送到 %u 个Web客户端\n", clientCount);
          } else {
            // Serial.println("没有连接的Web客户端，跳过图像发送");
          }
        } else {
          // Serial.printf("校验和验证失败！（接收：0x%02X，计算：0x%02X）\n", 
                        // receivedChecksum, localChecksum);
        }
        
        xSemaphoreGive(imgBufferMutex);
      }
    }
  }
}

// ========================== 11. 初始化与主循环 ===========================
void setup() {
  // 初始化调试串口
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("ESP32 多传感器融合电机控制系统初始化...");
  Wire.begin();  // 初始化I2C（颜色传感器依赖）

  // 初始化颜色传感器
  if (!tcs.begin()) {
    Serial.println("无法找到TCS34725传感器!请检查接线(SDA=21, SCL=22)");
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
  }
  Serial.println("TCS34725颜色传感器初始化成功");

  // 创建同步对象
  imgBufferMutex = xSemaphoreCreateMutex();
  imgQueue = xQueueCreate(QUEUE_LEN, sizeof(uint32_t));
  pidDataMutex = xSemaphoreCreateMutex();
  motorCmdQueue = xQueueCreate(5, sizeof(MotorCmd));
  
  // 检查同步对象创建结果
  if (imgBufferMutex == NULL || imgQueue == NULL || 
      pidDataMutex == NULL || motorCmdQueue == NULL) {
    Serial.println("同步对象创建失败！程序终止");
    while (1);
  }

  // 连接WiFi与初始化Web服务器
  connectToWiFi();
  setupWebServer();

  // 创建FreeRTOS任务
  // 任务1：串口接收（高优先级3）
  xTaskCreate(
    serialRxTask,    "SerialRxTask",
    8192,  NULL, 3, NULL
  );

  // 任务2：数据处理（中优先级2）
  xTaskCreate(
    dataProcTask,    "DataProcTask",
    8192,  NULL, 2, NULL
  );

  // 任务3：多传感器融合决策（中优先级2）
  xTaskCreate(
    sensorFusionTask,  "SensorFusionTask",
    4096,  NULL, 2, NULL
  );

  // 任务4：电机控制（中优先级2）
  xTaskCreate(
    motorControlTask,  "MotorCtrlTask",
    4096,  NULL, 2, NULL
  );
}

void loop() {
  // 处理Web客户端请求
  server.handleClient();
  vTaskDelay(pdMS_TO_TICKS(1));
}
