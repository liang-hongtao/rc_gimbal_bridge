#!/usr/bin/env python3
import socket
import time
import sys
import termios
import tty
import select

CAMERA_IP = "192.168.144.108"
UDP_PORT = 5000
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def calculate_checksum(cmd):
    crc = 0
    for char in cmd:
        crc += ord(char)
    return f"{crc & 0xFF:02X}"

def send(cmd):
    checksum = calculate_checksum(cmd)
    full_cmd = cmd + checksum
    sock.sendto(full_cmd.encode(), (CAMERA_IP, UDP_PORT))

def set_speed(yaw, pitch):
    """设置云台速度"""
    if yaw != 0:
        if yaw < 0:
            hex_val = f"{(256 + yaw):02X}"
        else:
            hex_val = f"{yaw:02X}"
        send(f"#TPUG2wGSY{hex_val}")
    
    if pitch != 0:
        if pitch < 0:
            hex_val = f"{(256 + pitch):02X}"
        else:
            hex_val = f"{pitch:02X}"
        send(f"#TPUG2wGSP{hex_val}")

# 保存终端设置
old_settings = termios.tcgetattr(sys.stdin)

try:
    tty.setraw(sys.stdin.fileno())
    
    print("\r================================")
    print("\r      实时相机控制")
    print("\r================================")
    print("\r移动: W↑ S↓ A← D→")
    print("\r功能: C回中 空格停止")
    # 拍照/录像功能已移除
    print("\r变倍: +放大 -缩小")
    print("\r退出: Q")
    print("\r================================\n")
    print("\r就绪...")
    
    while True:
        if select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            
            # 处理大小写
            key_lower = key.lower()
            
            if key_lower == 'q':
                break
            elif key_lower == 'w':
                set_speed(0, 60)
                print("\r↑ 向上        ", end="")
            elif key_lower == 's':
                set_speed(0, -60)
                print("\r↓ 向下        ", end="")
            elif key_lower == 'a':
                set_speed(-70, 0)
                print("\r← 向左        ", end="")
            elif key_lower == 'd':
                set_speed(70, 0)
                print("\r→ 向右        ", end="")
            elif key == ' ':
                send("#TPUG2wGSY00")
                send("#TPUG2wGSP00")
                print("\r■ 停止        ", end="")
            elif key_lower == 'c':
                send("#TPUG2wPTZ05")
                print("\r◎ 回中        ", end="")
            # 拍照/录像按键已移除
            elif key == '+' or key == '=':  # +号和=号都能触发放大
                send("#TPUD2wDZM0A")
                print("\r🔍+ 放大      ", end="")
            elif key == '-' or key == '_':  # -号和_号都能触发缩小
                send("#TPUD2wDZM0B")
                print("\r🔍- 缩小      ", end="")
            elif key == '1':
                send("#TPUD2wDZM01")
                print("\r🔍 1倍        ", end="")
            elif key == '2':
                send("#TPUD2wDZM02")
                print("\r🔍 2倍        ", end="")
            elif key == '3':
                send("#TPUD2wDZM03")
                print("\r🔍 3倍        ", end="")
            elif key == '4':
                send("#TPUD2wDZM04")
                print("\r🔍 4倍        ", end="")
            
            sys.stdout.flush()
            
        time.sleep(0.05)

finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    send("#TPUG2wGSY00")
    send("#TPUG2wGSP00")
    sock.close()
    print("\n\n控制已关闭")
