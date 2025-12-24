import serial
import time
import csv
import struct
import os
import math

COM_PORT = "COM10"
BAUD_RATE = 2000000
DATA_FILE = "../../9_VOFA/data/ZZZ/251115_treadmill.csv"  # 本地数据文件
RESPONSE_TIMEOUT_S = 5.0       # 等待 STM32 响应的超时时间
SKIP_HEADER = True

def pc_send_receive():
    # 使用脚本所在目录作为基准，避免 CWD 不一致导致找不到文件
    script_dir = os.path.abspath(os.path.dirname(__file__))
    data_path = os.path.join(script_dir, DATA_FILE)

    if not os.path.exists(data_path):
        # 调试信息：打印当前工作目录与目录列举，便于定位
        print("数据文件不存在:", DATA_FILE)
        print("脚本目录:", script_dir)
        print("当前工作目录:", os.getcwd())
        print("脚本目录下文件:", os.listdir(script_dir))
        return

    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)  # 短超时用于轮询读取
    time.sleep(0.2)  # 等待串口就绪

    with open(data_path, 'r', newline='') as f:
        reader = csv.reader(f)
        if SKIP_HEADER:
            next(reader, None)

        row_index = 0
        for row in reader:
            # 跳过空行
            if not row or all((cell.strip() == '') for cell in row):
                row_index += 1
                continue

            # 解析为浮点数，忽略无法解析的单元格
            floats = []
            for cell in row:
                s = cell.strip()
                if s == '':
                    continue
                try:
                    v = float(s)
                    if math.isnan(v):
                        v = 0.0
                    floats.append(float(v))
                except ValueError:
                    # 忽略非数值字段
                    pass

            if not floats:
                row_index += 1
                continue

            n = len(floats)
            if n > 0xFF:
                print(f"行 {row_index} 列数 {n} 超过 255, 跳过或拆分该行")
                row_index += 1
                continue

            # 帧结构： uint32(row_index) + uint8(col_count) + floats (little-endian)
            frame = struct.pack('<I', row_index) + struct.pack('<B', n) + struct.pack('<' + 'f'*n, *floats)

            print(f"发送行 {row_index}，列数 {n}，数据：{floats}")
            ser.write(frame)
            ser.flush()
            start_time = time.time()

            # 等待 STM32 回复：首字节 'Z'，后跟 4 字节 uint32 序号（总长 5 字节）
            while True:
                buf = bytearray()
                # 累积读取直到获得至少 5 字节或超时
                while len(buf) < 5 and (time.time() - start_time) < RESPONSE_TIMEOUT_S:
                    chunk = ser.read(5 - len(buf))
                    if chunk:
                        buf.extend(chunk)
                    else:
                        time.sleep(0.01)

                if len(buf) < 5:
                    # 超时重发：在重发前丢弃残留，避免解析错位
                    print("超时，重发当前帧...")
                    ser.reset_input_buffer()
                    ser.write(frame)
                    ser.flush()
                    start_time = time.time()
                    continue

                # 查找 header 'Z' 的位置（容错：丢弃前导噪声）
                header_idx = None
                for i, b in enumerate(buf):
                    if b == ord('Z'):
                        header_idx = i
                        break
                if header_idx is None:
                    # 未找到 'Z'，打印并重发
                    print("未找到 'Z' 头，返回数据（hex）:", buf.hex())
                    ser.reset_input_buffer()
                    if (time.time() - start_time) > RESPONSE_TIMEOUT_S:
                        ser.write(frame); ser.flush(); start_time = time.time()
                    continue

                # 确保从 header_idx 开始有足够字节 (需要 header + 4 字节 seq)
                if len(buf) < header_idx + 5:
                    # 不足时等待补齐或重发
                    # 尝试再读剩余字节一次
                    need = (header_idx + 5) - len(buf)
                    extra = ser.read(need)
                    if extra:
                        buf.extend(extra)
                    if len(buf) < header_idx + 5:
                        print("等待到完整回复超时，重发当前帧...")
                        ser.reset_input_buffer()
                        ser.write(frame); ser.flush(); start_time = time.time()
                        continue

                # 解析序号（小端，紧随 'Z' 之后的 4 字节）
                try:
                    recv_seq = struct.unpack_from('<I', bytes(buf), header_idx + 1)[0]
                except struct.error:
                    print("解析序号失败，数据（hex）:", buf.hex())
                    ser.reset_input_buffer()
                    ser.write(frame); ser.flush(); start_time = time.time()
                    continue

                # 比较序号是否与当前发送的行号一致
                if recv_seq == row_index:
                    print(f"收到确认: header='Z', seq={recv_seq} 匹配，发送下一帧\n")
                    # 丢弃串口输入缓冲内残余，准备下一帧
                    ser.reset_input_buffer()
                    break
                else:
                    print(f"收到 header='Z' 但序号不匹配: recv_seq={recv_seq} sent_row_index={row_index}，重发...")
                    ser.reset_input_buffer()
                    ser.write(frame)
                    ser.flush()
                    start_time = time.time()
                    continue
            row_index += 1

    ser.close()
    print("所有数据发送完成")

if __name__ == "__main__":
    pc_send_receive()