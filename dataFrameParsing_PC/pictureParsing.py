import serial
import time
import cv2  # 用OpenCV实时显示（比img.show()快10倍）
import numpy as np
from PIL import Image
import io

# 串口配置：缩短超时+移除冗余等待
SERIAL_PORT = 'COM4'
BAUDRATE = 115200  # 若发送端改230400，这里同步改

# 帧头帧尾（与发送端一致）
FRAME_HEADER = b'IMG_START'
FRAME_FOOTER = b'IMG_END'
HEADER_LEN = len(FRAME_HEADER)
FOOTER_LEN = len(FRAME_FOOTER)


def init_serial():
    """初始化串口：缩短超时到1秒，减少无效等待"""
    try:
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUDRATE,
            timeout=1  # 1秒足够传输1帧QQVGA图像
        )
        if ser.is_open:
            print(f"串口连接成功: {SERIAL_PORT}")
            return ser
    except Exception as e:
        print(f"串口初始化失败: {e}")
    return None


def calculate_checksum(data):
    """与发送端一致的校验和计算"""
    return sum(data) % 256


def receive_and_parse(ser):
    """优化接收逻辑：减少循环等待，加快数据读取"""
    # 1. 快速匹配帧头（避免逐字节等待）
    header_buf = b''
    while len(header_buf) < HEADER_LEN:
        chunk = ser.read(HEADER_LEN - len(header_buf))  # 批量读，减少IO次数
        if not chunk:
            return None, None
        header_buf += chunk
        # 滑动窗口匹配：避免漏帧（比如前几字节是无效数据）
        if header_buf[-HEADER_LEN:] == FRAME_HEADER:
            header_buf = header_buf[-HEADER_LEN:]
            break
    if header_buf != FRAME_HEADER:
        print(f"无效帧头: {header_buf.hex()}")
        return None, None
    print("检测到有效帧头")

    # 2. 读取图像大小（4字节）
    size_bytes = ser.read(4)
    if len(size_bytes) != 4:
        print(f"读取大小失败: {len(size_bytes)}字节")
        return None, None
    img_size = int.from_bytes(size_bytes, 'big')

    # 3. 读取校验和（1字节）
    checksum_byte = ser.read(1)
    if len(checksum_byte) != 1:
        print("读取校验和失败")
        return None, None
    expected_checksum = checksum_byte[0]

    # 4. 批量读取图像数据（一次读满，减少循环次数）
    img_data = ser.read(img_size)  # 直接读指定大小，比循环读快
    if len(img_data) != img_size:
        print(f"数据接收不完整: {len(img_data)}/{img_size}字节")
        return None, img_data

    # 5. 验证校验和
    actual_checksum = calculate_checksum(img_data)
    if actual_checksum != expected_checksum:
        print(f"校验和不匹配: 预期{expected_checksum}, 实际{actual_checksum}")
        return None, img_data

    # 6. 验证帧尾
    footer = ser.read(FOOTER_LEN)
    if footer != FRAME_FOOTER:
        print(f"帧尾不匹配: {footer.hex()}")
        return None, img_data

    return img_data, img_data


def main():
    ser = init_serial()
    if not ser:
        return

    # 用OpenCV创建实时显示窗口（比img.show()快，无弹窗延迟）
    cv2.namedWindow("OpenMV Real-Time", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("OpenMV Real-Time", 320, 240)  # 放大显示（原160x120）

    frame_count = 0
    try:
        while True:
            frame_count += 1
            # 接收并解析数据（无额外等待）
            img_data, raw_data = receive_and_parse(ser)

            if img_data:
                try:
                    # 快速转换为OpenCV格式（避免PIL中间步骤）
                    # 1. 从JPEG字节流解码
                    img = Image.open(io.BytesIO(img_data))
                    # 2. 转换为OpenCV图像（PIL→numpy→BGR）
                    img_cv = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
                    # 3. 实时显示（无弹窗，直接刷新窗口）
                    cv2.imshow("OpenMV Real-Time", img_cv)
                    # 4. 保存图像（可选，建议每10帧存1次，避免IO占用）
                    if frame_count % 10 == 0:
                        cv2.imwrite(f"received_frame_{frame_count}.jpg", img_cv)
                    print(f"第{frame_count}帧处理成功")
                except Exception as e:
                    print(f"图像解析失败: {e}")
            else:
                print(f"第{frame_count}帧处理失败")

            # 处理键盘退出（避免窗口卡死）
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("\n程序手动停止")
    finally:
        # 释放资源
        cv2.destroyAllWindows()
        if ser.is_open:
            ser.close()
            print("串口已关闭")


if __name__ == "__main__":
    main()