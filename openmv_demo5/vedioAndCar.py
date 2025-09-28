THRESHOLD = (23, 100, -24, 17, -9, 48)  # 灰度阈值
import sensor, image, time, pyb, gc
from pyb import LED
import car
from mypid import PID

rho_pid = PID(p=0.5, i=0.0)
theta_pid = PID(p=0.001, i=0)
FRAME_HEADER = b'IMG_START'
FRAME_FOOTER = b'IMG_END'

# 初始化LED
LED(1).on()
LED(2).on()
LED(3).on()

# 初始化摄像头
sensor.reset()
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQQVGA)  # 80x60
sensor.skip_frames(time=2000)
clock = time.clock()

# 传感器数据协议定义
SENSOR_HEADER = 0xAA    # 帧头
SENSOR_FOOTER = 0x55    # 帧尾
CMD_DISTANCE = 0x01     # 障碍物指令
CMD_COLOR = 0x02        # 颜色指令

# 行为状态机
STATE_NORMAL = 0            # 正常巡线
STATE_OBSTACLE_STOP = 1     # 遇障停止
STATE_TURN_RIGHT_90 = 2     # 右转90°
STATE_FORWARD_AVOID = 3     # 前进越障
STATE_TURN_LEFT_90 = 4      # 左转90°
STATE_FORWARD_SHORT = 5     # 前进小段
STATE_TURN_LEFT_SEARCH = 6  # 左转找线
STATE_RECOVER_LINE = 7      # 恢复巡线
STATE_AVOID_TIMEOUT = 8     # 避障超时

# 全局变量
current_state = STATE_NORMAL
obstacle_detected = 0       # 障碍物状态（0=无，1=有）
color_id = 2                # 颜色ID（0=白，1=黑，2=未知）
action_start_time = 0
last_image_send_time = 0
last_debug_time = 0

# 串口解析变量（修复后）
parse_state = 0  # 0=等帧头，1=等指令，2=等数据，3=等帧尾
current_cmd = 0  # 当前指令

uart = pyb.UART(3, 115200, timeout_char=100)

def parse_esp32_data():
    """修复后的解析函数：批量读取并解析所有可用数据"""
    global parse_state, current_cmd, obstacle_detected, color_id

    # 读取所有可用字节
    data = uart.read(uart.any())
    if not data:
        return

    for byte in data:
        # 关键修复：任何状态下收到帧头都强制重置同步
        if byte == SENSOR_HEADER:
            parse_state = 1  # 进入等待指令状态
            continue

        # 按状态机解析
        if parse_state == 1:
            # 等待指令
            if byte in [CMD_DISTANCE, CMD_COLOR]:
                current_cmd = byte
                parse_state = 2
            else:
                parse_state = 0  # 指令无效，重置
        elif parse_state == 2:
            # 等待数据
            if current_cmd == CMD_DISTANCE:
                if byte in [0, 1]:
                    obstacle_detected = byte
                    parse_state = 3
                else:
                    parse_state = 0
            elif current_cmd == CMD_COLOR:
                if byte in [0, 1, 2]:
                    color_id = byte
                    parse_state = 3
                else:
                    parse_state = 0
        elif parse_state == 3:
            # 等待帧尾
            if byte == SENSOR_FOOTER:
                # 解析成功，更新调试信息
                print(f"✅ 解析成功: 障碍物={obstacle_detected} | 颜色ID={color_id}")
            else:
                print(f"❌ 帧尾错误: 期望0x{SENSOR_FOOTER:02X}, 收到0x{byte:02X}")
            parse_state = 0  # 无论成功失败都重置

# 测试函数：模拟ESP32发送数据
def test_esp32_simulation():
    """模拟ESP32发送数据用于测试"""
    import random
    global uart

    # 随机生成测试数据
    obstacle = random.randint(0, 1)
    color = random.randint(0, 2)

    # 构造数据包
    test_data = bytes([SENSOR_HEADER, CMD_DISTANCE, obstacle, SENSOR_FOOTER,
                      SENSOR_HEADER, CMD_COLOR, color, SENSOR_FOOTER])

    # 模拟接收（实际使用时删除）
    for byte in test_data:
        parse_esp32_data_byte(byte)

def parse_esp32_data_byte(byte):
    """单字节解析函数（用于测试）"""
    global parse_state, current_cmd, obstacle_detected, color_id

    # 关键修复：任何状态下收到帧头都强制重置同步
    if byte == SENSOR_HEADER:
        parse_state = 1
        return

    if parse_state == 1:
        if byte in [CMD_DISTANCE, CMD_COLOR]:
            current_cmd = byte
            parse_state = 2
        else:
            parse_state = 0
    elif parse_state == 2:
        if current_cmd == CMD_DISTANCE:
            if byte in [0, 1]:
                obstacle_detected = byte
                parse_state = 3
            else:
                parse_state = 0
        elif current_cmd == CMD_COLOR:
            if byte in [0, 1, 2]:
                color_id = byte
                parse_state = 3
            else:
                parse_state = 0
    elif parse_state == 3:
        if byte == SENSOR_FOOTER:
            print(f"✅ 解析成功: 障碍物={obstacle_detected} | 颜色ID={color_id}")
        parse_state = 0

while(True):
    clock.tick()
    img = sensor.snapshot()

    # 压缩图像并发送
    compressed = img.copy().compress(quality=15)
    checksum = sum(compressed) % 256
    uart.write(FRAME_HEADER)
    uart.write(len(compressed).to_bytes(4, 'big'))
    uart.write(bytes([checksum]))
    uart.write(compressed)
    uart.write(FRAME_FOOTER)

    # 修复：使用批量读取解析ESP32数据
    parse_esp32_data()

    # 可选：用于测试的模拟数据（实际使用时注释掉）
    # if time.ticks_ms() % 2000 < 10:  # 每2秒模拟一次
    #     test_esp32_simulation()

    # 定时打印状态
    if time.ticks_ms() - last_debug_time > 500:  # 每500ms打印一次
        print(f"当前状态: 障碍物={obstacle_detected} | 颜色ID={color_id} | 解析状态={parse_state}")
        last_debug_time = time.ticks_ms()

    # 巡线逻辑（根据你的需求启用）
    line = img.binary([THRESHOLD]).get_regression([(0,50)], robust=True)
    if line:
        rho_err = abs(line.rho())-img.width()/2
        if line.theta()>90:
            theta_err = line.theta()-180
        else:
            theta_err = line.theta()
        img.draw_line(line.line(), color=127)

        if line.magnitude() > 2:
            rho_output = rho_pid.get_pid(rho_err,1)
            theta_output = theta_pid.get_pid(theta_err,1)
            output = rho_output+theta_output

            # 根据障碍物状态控制小车
            if obstacle_detected == 0:  # 无障碍物
                car.run(-50+output, -50-output)
            else:  # 有障碍物
                car.run(0, 0)  # 停止
                print("⚠️ 检测到障碍物，停止!")
        else:
            car.run(0,0)
    else:
        if obstacle_detected == 0:  # 无障碍物时才转弯找线
            car.run(30,-30)
        else:
            car.run(0,0)
