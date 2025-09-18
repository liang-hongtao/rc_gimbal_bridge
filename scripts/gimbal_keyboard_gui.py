#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import tkinter as tk
import rospy


def calculate_checksum(cmd: str) -> str:
    return f"{sum(ord(c) for c in cmd) & 0xFF:02X}"


class GimbalKeyboardGUI:
    def __init__(self):
        rospy.init_node('gimbal_keyboard_gui')
        self.camera_ip = rospy.get_param('~camera_ip', '192.168.144.108')
        self.udp_port = rospy.get_param('~port', 5000)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.root = tk.Tk()
        self.root.title('云台键盘控制')
        self.root.geometry('480x320')

        info = (
            'W/S: 俯仰 ±60\n'
            'A/D: 偏航 ±70\n'
            '空格: 停止\n'
            'C: 回中\n'
            '+ / -: 变倍   1~4: 指定倍数\n'
            '\n'
            f'目标设备: {self.camera_ip}:{self.udp_port}'
        )
        self.label = tk.Label(self.root, text=info, justify='left')
        self.label.pack(padx=12, pady=12, anchor='w')

        self.status_var = tk.StringVar(value='就绪')
        self.status = tk.Label(self.root, textvariable=self.status_var, fg='green')
        self.status.pack(padx=12, pady=6, anchor='w')

        self.root.bind('<KeyPress>', self.on_key)
        self.root.bind('<KeyRelease>', self.on_key_release)
        self.root.protocol('WM_DELETE_WINDOW', self.on_close)

        rospy.loginfo('gimbal_keyboard_gui 已启动，按键请聚焦该窗口。')

    def send(self, cmd: str):
        full = cmd + calculate_checksum(cmd)
        try:
            self.sock.sendto(full.encode(), (self.camera_ip, self.udp_port))
        except Exception as e:
            rospy.logwarn_throttle(1.0, f'发送失败: {e}')

    @staticmethod
    def to_hex_signed(v: int) -> str:
        if v < 0:
            v = 256 + v
        return f"{int(v) & 0xFF:02X}"

    def set_speed(self, yaw: int, pitch: int):
        if yaw != 0:
            self.send('#TPUG2wGSY' + self.to_hex_signed(yaw))
        if pitch != 0:
            self.send('#TPUG2wGSP' + self.to_hex_signed(pitch))

    def on_key(self, event):
        k = event.keysym.lower()
        if k == 'w':
            self.set_speed(0, 60)
            self.status_var.set('向上')
        elif k == 's':
            self.set_speed(0, -60)
            self.status_var.set('向下')
        elif k == 'a':
            self.set_speed(-70, 0)
            self.status_var.set('向左')
        elif k == 'd':
            self.set_speed(70, 0)
            self.status_var.set('向右')
        elif k == 'space':
            self.send('#TPUG2wGSY00')
            self.send('#TPUG2wGSP00')
            self.status_var.set('停止')
        elif k == 'c':
            self.send('#TPUG2wPTZ05')
            self.status_var.set('回中')
        # 拍照/录像功能已移除
        elif k in ('plus', 'equal'):  # + 或 =
            self.send('#TPUD2wDZM0A')
            self.status_var.set('放大')
        elif k in ('minus', 'underscore'):
            self.send('#TPUD2wDZM0B')
            self.status_var.set('缩小')
        elif k == '1':
            self.send('#TPUD2wDZM01')
            self.status_var.set('1倍')
        elif k == '2':
            self.send('#TPUD2wDZM02')
            self.status_var.set('2倍')
        elif k == '3':
            self.send('#TPUD2wDZM03')
            self.status_var.set('3倍')
        elif k == '4':
            self.send('#TPUD2wDZM04')
            self.status_var.set('4倍')

    def on_key_release(self, event):
        # 松开方向键时自动停止对应轴（简单做法：同时停两轴）
        if event.keysym.lower() in ('w', 's', 'a', 'd'):
            self.send('#TPUG2wGSY00')
            self.send('#TPUG2wGSP00')
            self.status_var.set('停止')

    def on_close(self):
        try:
            self.send('#TPUG2wGSY00')
            self.send('#TPUG2wGSP00')
            self.sock.close()
        except Exception:
            pass
        self.root.destroy()
        rospy.signal_shutdown('UI closed')

    def run(self):
        self.root.mainloop()


if __name__ == '__main__':
    app = GimbalKeyboardGUI()
    app.run()
