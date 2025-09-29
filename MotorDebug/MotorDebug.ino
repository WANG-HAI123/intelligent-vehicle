#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// ========================== 1. 硬件参数配置（必须根据你的小车修改！）==========================
#define WHEEL_DIAMETER 65.0f    // 车轮直径（mm，例：65mm）
#define WHEEL_BASE 150.0f       // 轮距（两车轮中心距离，mm，例：150mm）
#define MOTOR_RPM 150.0f        // 电机转速（转/分钟，例：150RPM，需实际测量）
#define ANGLE_CALIB 1.05f       // 角度校准系数（实际角度偏小则大于1，偏大则小于1）
#define DISTANCE_CALIB 1.08f    // 距离校准系数（实际距离偏小则大于1，偏大则小于1）

// ========================== 2. 电机引脚定义（与硬件一致）==========================
#define AIN1 27  // 左电机方向1
#define AIN2 26  // 左电机方向2
#define PWMA 25  // 左电机PWM
#define BIN1 12  // 右电机方向1
#define BIN2 13  // 右电机方向2
#define PWMB 14  // 右电机PWM

// ========================== 3. 命令类型扩展（新增角度/距离控制）==========================
typedef enum {
  CMD_STOP, CMD_BRAKE, 
  CMD_FORWARD, CMD_BACKWARD, CMD_LEFT, CMD_RIGHT,
  CMD_ANGLE_TURN,  // 新增：指定角度转向（左/右）
  CMD_DIST_MOVE    // 新增：指定距离移动（前/后）
} MotorCmdType;

// 新增：角度控制参数
typedef struct {
  bool isLeftTurn;  // true=左转，false=右转
  float targetAngle;// 目标角度（°，例：90=左转90°）
  uint8_t speed;    // 速度（0-255）
} AngleParam;

// 新增：距离控制参数
typedef struct {
  bool isForward;   // true=前进，false=后退
  float targetDist; // 目标距离（mm，例：200=前进200mm）
  uint8_t speed;    // 速度（0-255）
} DistParam;

// 统一电机命令结构体（兼容所有命令类型）
typedef struct {
  MotorCmdType cmd;
  union {
    uint8_t speed;       // 基础命令（前进/后退/左/右）的速度
    AngleParam angle;    // 角度控制参数
    DistParam dist;      // 距离控制参数
  } param;
} MotorCmd;

// ========================== 4. 全局变量 ===========================
QueueHandle_t motorCmdQueue;  // 电机命令队列


// ========================== 5. 工具函数（计算角度/距离对应的运行时间）==========================
/**
 * 计算转向所需时间（ms）
 * @param targetAngle 目标角度（°）
 * @param speed 电机速度（0-255）
 * @return 运行时间（ms）
 */
uint32_t calcTurnTime(float targetAngle, uint8_t speed) {
  if (speed == 0) return 0;
  // 1. 电机转速与速度的映射（假设速度255对应额定转速MOTOR_RPM）
  float actualRpm = MOTOR_RPM * ((float)speed / 255.0f);
  // 2. 电机每转一圈的时间（ms）
  float rpmToMsPerCircle = 60000.0f / actualRpm;
  // 3. 单侧电机转1圈对应的转向角度（°）
  float anglePerCircle = (PI * WHEEL_DIAMETER / WHEEL_BASE) * 360.0f;
  // 4. 目标角度所需的时间（ms），乘以校准系数
  float targetTime = (targetAngle / anglePerCircle) * rpmToMsPerCircle * ANGLE_CALIB;
  return (uint32_t)targetTime;
}

/**
 * 计算直线移动所需时间（ms）
 * @param targetDist 目标距离（mm）
 * @param speed 电机速度（0-255）
 * @return 运行时间（ms）
 */
uint32_t calcMoveTime(float targetDist, uint8_t speed) {
  if (speed == 0) return 0;
  // 1. 电机转速与速度的映射
  float actualRpm = MOTOR_RPM * ((float)speed / 255.0f);
  // 2. 电机每转一圈的时间（ms）
  float rpmToMsPerCircle = 60000.0f / actualRpm;
  // 3. 车轮每转一圈移动的距离（mm）
  float distPerCircle = PI * WHEEL_DIAMETER;
  // 4. 目标距离所需的时间（ms），乘以校准系数
  float targetTime = (targetDist / distPerCircle) * rpmToMsPerCircle * DISTANCE_CALIB;
  return (uint32_t)targetTime;
}


// ========================== 6. 电机控制任务（扩展角度/距离处理）==========================
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
  TickType_t xLastWakeTime;  // 用于精确延时

  while (1) {
    if (xQueueReceive(motorCmdQueue, &cmd, portMAX_DELAY) == pdTRUE) {
      switch (cmd.cmd) {
        // -------------------------- 原有基础命令（不变）--------------------------
        case CMD_STOP:
          digitalWrite(AIN1, LOW);
          digitalWrite(AIN2, LOW);
          digitalWrite(BIN1, LOW);
          digitalWrite(BIN2, LOW);
          analogWrite(PWMA, 0);
          analogWrite(PWMB, 0);
          Serial.println("电机已停止");
          break;

        case CMD_BRAKE:
          digitalWrite(AIN1, HIGH);
          digitalWrite(AIN2, HIGH);
          digitalWrite(BIN1, HIGH);
          digitalWrite(BIN2, HIGH);
          analogWrite(PWMA, 0);
          analogWrite(PWMB, 0);
          Serial.println("电机已刹车");
          break;

        case CMD_FORWARD:
          digitalWrite(AIN1, HIGH);
          digitalWrite(AIN2, LOW);
          digitalWrite(BIN1, HIGH);
          digitalWrite(BIN2, LOW);
          analogWrite(PWMA, cmd.param.speed);
          analogWrite(PWMB, cmd.param.speed);
          Serial.printf("前进（速度：%d）\n", cmd.param.speed);
          break;

        case CMD_BACKWARD:
          digitalWrite(AIN1, LOW);
          digitalWrite(AIN2, HIGH);
          digitalWrite(BIN1, LOW);
          digitalWrite(BIN2, HIGH);
          analogWrite(PWMA, cmd.param.speed);
          analogWrite(PWMB, cmd.param.speed);
          Serial.printf("后退（速度：%d）\n", cmd.param.speed);
          break;

        case CMD_LEFT:
          digitalWrite(AIN1, LOW);
          digitalWrite(AIN2, LOW);
          digitalWrite(BIN1, LOW);
          digitalWrite(BIN2, HIGH);
          analogWrite(PWMA, 0);
          analogWrite(PWMB, cmd.param.speed);
          Serial.printf("左转（速度：%d）\n", cmd.param.speed);
          break;

        case CMD_RIGHT:
          digitalWrite(AIN1, LOW);
          digitalWrite(AIN2, HIGH);
          digitalWrite(BIN1, LOW);
          digitalWrite(BIN2, LOW);
          analogWrite(PWMA, cmd.param.speed);
          analogWrite(PWMB, 0);
          Serial.printf("右转（速度：%d）\n", cmd.param.speed);
          break;

        // -------------------------- 新增：角度转向命令 --------------------------
        case CMD_ANGLE_TURN: {
          bool isLeft = cmd.param.angle.isLeftTurn;
          float angle = cmd.param.angle.targetAngle;
          uint8_t speed = cmd.param.angle.speed;
          Serial.printf("%s转（目标角度：%.1f°，速度：%d）\n", 
                        isLeft ? "左" : "右", angle, speed);

          // 1. 计算所需运行时间
          uint32_t runTime = calcTurnTime(angle, speed);
          if (runTime == 0) {
            Serial.println("速度为0，取消转向");
            break;
          }
          Serial.printf("预计转向时间：%d ms\n", runTime);

          // 2. 执行转向动作
          if (isLeft) {
            // 左转：右电机反转（与原CMD_LEFT一致）
            digitalWrite(AIN1, LOW);
            digitalWrite(AIN2, LOW);
            digitalWrite(BIN1, LOW);
            digitalWrite(BIN2, HIGH);
            analogWrite(PWMA, 0);
            analogWrite(PWMB, speed);
          } else {
            // 右转：左电机反转（与原CMD_RIGHT一致）
            digitalWrite(AIN1, LOW);
            digitalWrite(AIN2, HIGH);
            digitalWrite(BIN1, LOW);
            digitalWrite(BIN2, LOW);
            analogWrite(PWMA, speed);
            analogWrite(PWMB, 0);
          }

          // 3. 精确延时指定时间（FreeRTOS推荐用vTaskDelayUntil，避免任务调度偏差）
          xLastWakeTime = xTaskGetTickCount();
          vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(runTime));

          // 4. 转向完成后自动停止
          digitalWrite(AIN1, LOW);
          digitalWrite(AIN2, LOW);
          digitalWrite(BIN1, LOW);
          digitalWrite(BIN2, LOW);
          analogWrite(PWMA, 0);
          analogWrite(PWMB, 0);
          Serial.printf("%s转完成（实际时间：%d ms）\n", isLeft ? "左" : "右", runTime);
          break;
        }

        // -------------------------- 新增：距离移动命令 --------------------------
        case CMD_DIST_MOVE: {
          bool isForward = cmd.param.dist.isForward;
          float dist = cmd.param.dist.targetDist;
          uint8_t speed = cmd.param.dist.speed;
          Serial.printf("%s进（目标距离：%.1f mm，速度：%d）\n", 
                        isForward ? "前" : "后", dist, speed);

          // 1. 计算所需运行时间
          uint32_t runTime = calcMoveTime(dist, speed);
          if (runTime == 0) {
            Serial.println("速度为0，取消移动");
            break;
          }
          Serial.printf("预计移动时间：%d ms\n", runTime);

          // 2. 执行移动动作
          if (isForward) {
            // 前进（与原CMD_FORWARD一致）
            digitalWrite(AIN1, HIGH);
            digitalWrite(AIN2, LOW);
            digitalWrite(BIN1, HIGH);
            digitalWrite(BIN2, LOW);
          } else {
            // 后退（与原CMD_BACKWARD一致）
            digitalWrite(AIN1, LOW);
            digitalWrite(AIN2, HIGH);
            digitalWrite(BIN1, LOW);
            digitalWrite(BIN2, HIGH);
          }
          analogWrite(PWMA, speed);
          analogWrite(PWMB, speed);

          // 3. 精确延时指定时间
          xLastWakeTime = xTaskGetTickCount();
          vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(runTime));

          // 4. 移动完成后自动停止
          digitalWrite(AIN1, LOW);
          digitalWrite(AIN2, LOW);
          digitalWrite(BIN1, LOW);
          digitalWrite(BIN2, LOW);
          analogWrite(PWMA, 0);
          analogWrite(PWMB, 0);
          Serial.printf("%s进完成（实际时间：%d ms）\n", isForward ? "前" : "后", runTime);
          break;
        }
      }
    }
  }
}


// ========================== 7. 串口指令处理任务（扩展角度/距离指令解析）==========================
void serialCommandTask(void* param) {
  Serial.println("==================== 电机调试程序（带角度/距离控制） ====================");
  Serial.println("【基础指令】");
  Serial.println("f [速度]    - 前进（例：f 100）");
  Serial.println("b [速度]    - 后退（例：b 100）");
  Serial.println("l [速度]    - 左转（例：l 100）");
  Serial.println("r [速度]    - 右转（例：r 100）");
  Serial.println("s           - 停止");
  Serial.println("k           - 刹车");
  Serial.println("【高级指令】");
  Serial.println("a l [角度] [速度] - 左转指定角度（例：a l 90 100 → 左转90°，速度100）");
  Serial.println("a r [角度] [速度] - 右转指定角度（例：a r 45 80 → 右转45°，速度80）");
  Serial.println("d f [距离] [速度] - 前进指定距离（例：d f 200 100 → 前进200mm，速度100）");
  Serial.println("d b [距离] [速度] - 后退指定距离（例：d b 150 80 → 后退150mm，速度80）");
  Serial.println("======================================================================");
  Serial.println("速度范围：0-255 | 角度范围：0-360° | 距离范围：0-9999mm");

  char cmdBuffer[50];
  int bufferIndex = 0;
  
  while (1) {
    if (Serial.available() > 0) {
      char c = Serial.read();
      Serial.print(c);  // 回显输入字符

      // 处理换行符（指令结束）
      if (c == '\n' || c == '\r') {
        if (bufferIndex > 0) {
          cmdBuffer[bufferIndex] = '\0';
          MotorCmd motorCmd;
          motorCmd.param.speed = 100;  // 默认速度

          // -------------------------- 解析原有基础指令 --------------------------
          if (cmdBuffer[0] == 'f') {
            motorCmd.cmd = CMD_FORWARD;
            sscanf(cmdBuffer, "f %hhu", &motorCmd.param.speed);
          } else if (cmdBuffer[0] == 'b') {
            motorCmd.cmd = CMD_BACKWARD;
            sscanf(cmdBuffer, "b %hhu", &motorCmd.param.speed);
          } else if (cmdBuffer[0] == 'l') {
            motorCmd.cmd = CMD_LEFT;
            sscanf(cmdBuffer, "l %hhu", &motorCmd.param.speed);
          } else if (cmdBuffer[0] == 'r') {
            motorCmd.cmd = CMD_RIGHT;
            sscanf(cmdBuffer, "r %hhu", &motorCmd.param.speed);
          } else if (cmdBuffer[0] == 's') {
            motorCmd.cmd = CMD_STOP;
          } else if (cmdBuffer[0] == 'k') {
            motorCmd.cmd = CMD_BRAKE;
          }

          // -------------------------- 解析新增角度指令（a开头） --------------------------
          else if (cmdBuffer[0] == 'a') {
            motorCmd.cmd = CMD_ANGLE_TURN;
            char dir;
            float angle;
            uint8_t speed;
            // 解析格式：a l 90 100 → 方向l/r，角度，速度
            int parseRet = sscanf(cmdBuffer, "a %c %f %hhu", &dir, &angle, &speed);
            if (parseRet != 3 || (dir != 'l' && dir != 'r') || angle <= 0 || angle > 360) {
              Serial.println("\n角度指令格式错误！正确例：a l 90 100");
              bufferIndex = 0;
              continue;
            }
            // 填充角度参数
            motorCmd.param.angle.isLeftTurn = (dir == 'l');
            motorCmd.param.angle.targetAngle = angle;
            motorCmd.param.angle.speed = constrain(speed, 0, 255);
          }

          // -------------------------- 解析新增距离指令（d开头） --------------------------
          else if (cmdBuffer[0] == 'd') {
            motorCmd.cmd = CMD_DIST_MOVE;
            char dir;
            float dist;
            uint8_t speed;
            // 解析格式：d f 200 100 → 方向f/b，距离，速度
            int parseRet = sscanf(cmdBuffer, "d %c %f %hhu", &dir, &dist, &speed);
            if (parseRet != 3 || (dir != 'f' && dir != 'b') || dist <= 0 || dist > 9999) {
              Serial.println("\n距离指令格式错误！正确例：d f 200 100");
              bufferIndex = 0;
              continue;
            }
            // 填充距离参数
            motorCmd.param.dist.isForward = (dir == 'f');
            motorCmd.param.dist.targetDist = dist;
            motorCmd.param.dist.speed = constrain(speed, 0, 255);
          }

          // -------------------------- 未知指令 --------------------------
          else {
            Serial.println("\n未知指令！请参考指令说明");
            bufferIndex = 0;
            continue;
          }

          // 限制速度范围（所有命令通用）
          if (motorCmd.cmd != CMD_ANGLE_TURN && motorCmd.cmd != CMD_DIST_MOVE) {
            motorCmd.param.speed = constrain(motorCmd.param.speed, 0, 255);
          }

          // 发送命令到队列
          xQueueSend(motorCmdQueue, &motorCmd, 0);
          bufferIndex = 0;
          Serial.println();
        }
      }
      // 积累指令字符（防止缓冲区溢出）
      else if (bufferIndex < 49) {
        cmdBuffer[bufferIndex++] = c;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));  // 降低CPU占用
  }
}


// ========================== 8. 初始化与主循环 ===========================
void setup() {
  // 初始化串口（波特率115200）
  Serial.begin(115200);
  while (!Serial) delay(10);  // 等待串口监视器连接
  
  // 创建电机命令队列（缓存5条命令）
  motorCmdQueue = xQueueCreate(5, sizeof(MotorCmd));
  if (motorCmdQueue == NULL) {
    Serial.println("命令队列创建失败！");
    while (1);  // 卡死提示
  }
  
  // 创建电机控制任务（优先级2，栈2048字节）
  xTaskCreate(
    motorControlTask,    "MotorControl",
    2048,  NULL, 2, NULL
  );
  
  // 创建串口指令处理任务（优先级1，栈2048字节）
  xTaskCreate(
    serialCommandTask,   "SerialCommand",
    2048,  NULL, 1, NULL
  );
}

void loop() {
  // 主循环空闲，所有逻辑在FreeRTOS任务中处理
  vTaskDelay(pdMS_TO_TICKS(1000));
}