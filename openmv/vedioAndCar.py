THRESHOLD = (20, 44, -15, 3, -25, -3) # 灰度阈值
import sensor, image, time, pyb, gc
from pyb import LED
import car
from mypid import PID
import struct

rho_pid = PID(p=1.0, i=0.1)
theta_pid = PID(p=0.05, i=0)
FRAME_HEADER = b'IMG_START'
FRAME_FOOTER = b'IMG_END'
DATA_SEPARATOR = b'||'  # 数据分隔符

# 初始化LED
led1 = pyb.LED(1)
led2 = pyb.LED(2)
led3 = pyb.LED(3)

# 初始化摄像头
sensor.reset()
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQQVGA)  # 80x60
sensor.skip_frames(time=2000)
clock = time.clock()


uart = pyb.UART(3, 115200, timeout_char=100)

def float_to_bytes(f):
    """将浮点数转换为4字节表示"""
    return struct.pack('f', f)

while(True):
    clock.tick()
    led1.on()
    led2.on()
    led3.on()
    img = sensor.snapshot()
    # 压缩图像
    compressed = img.copy().compress(quality=15)
    checksum = sum(compressed) % 256
    # 初始化变量，防止未定义
    rho_output = 0.0
    theta_output = 0.0
    rho_err = 0.0
    theta_err = 0.0

    # 巡线逻辑
    line = img.binary([THRESHOLD]).get_regression([(100,100)], robust=True)
    if line:
        rho_err = abs(line.rho()) - img.width()/2
        if line.theta() > 90:
            theta_err = line.theta() - 180
        else:
            theta_err = line.theta()
        img.draw_line(line.line(), color=127)

        if line.magnitude() > 2:
            rho_output = rho_pid.get_pid(rho_err, 1)
            theta_output = theta_pid.get_pid(theta_err, 1)
            print("rho_output:%d, theta_output:%d",rho_output, theta_output)

    led1.off()
    led2.off()
    led3.off()
    # 发送帧头
    uart.write(FRAME_HEADER)

    # 发送传感器数据：rho_err, theta_err, rho_output, theta_output
    uart.write(struct.pack('f', rho_err))
    uart.write(struct.pack('f', theta_err))
    uart.write(struct.pack('f', rho_output))
    uart.write(struct.pack('f', theta_output))

    # 发送数据分隔符
    uart.write(DATA_SEPARATOR)

    # 发送图像数据
    uart.write(len(compressed).to_bytes(4, 'big'))
    uart.write(bytes([checksum]))
    uart.write(compressed)

    # 发送帧尾
    uart.write(FRAME_FOOTER)
