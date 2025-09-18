#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from mavros_msgs.msg import RCIn
import socket


class RcToGimbal:
    def __init__(self):
        # 网络与轴映射
        self.cam_ip = rospy.get_param('~camera_ip', '192.168.144.108')
        self.port = rospy.get_param('~port', 5000)
        # 默认：左拨轮=CH9 控偏航，右拨轮=CH10 控俯仰
        self.ch_yaw = rospy.get_param('~ch_yaw', 9)
        self.ch_pitch = rospy.get_param('~ch_pitch', 10)

        # RC端点与中心/死区（单位us）
        self.pwm_min_yaw = rospy.get_param('~pwm_min_yaw', 1050)
        self.pwm_max_yaw = rospy.get_param('~pwm_max_yaw', 1950)
        self.center_yaw = rospy.get_param('~center_yaw', 1500)
        self.deadband_yaw = rospy.get_param('~deadband_yaw', 80)

        self.pwm_min_pitch = rospy.get_param('~pwm_min_pitch', 1050)
        self.pwm_max_pitch = rospy.get_param('~pwm_max_pitch', 1950)
        self.center_pitch = rospy.get_param('~center_pitch', 1500)
        self.deadband_pitch = rospy.get_param('~deadband_pitch', 80)

        # 速度限制与频率
        self.max_yaw = rospy.get_param('~max_yaw', 70)
        self.max_pitch = rospy.get_param('~max_pitch', 60)
        self.repeat_hz = rospy.get_param('~repeat_hz', 15)

        # 模式：deadband_speed（推荐）/ speed / position
        self.control_mode = rospy.get_param('~control_mode', 'deadband_speed')

        # 位置模式参数（保留可选）：PWM -> 角度端点
        self.yaw_min_deg = rospy.get_param('~yaw_min_deg', -90.0)
        self.yaw_max_deg = rospy.get_param('~yaw_max_deg', 90.0)
        self.pitch_min_deg = rospy.get_param('~pitch_min_deg', -30.0)
        self.pitch_max_deg = rospy.get_param('~pitch_max_deg', 30.0)
        self.kp_yaw = rospy.get_param('~kp_yaw', 1.0)
        self.kp_pitch = rospy.get_param('~kp_pitch', 1.0)
        self.speed_scale_yaw = rospy.get_param('~speed_scale_yaw', 0.8)
        self.speed_scale_pitch = rospy.get_param('~speed_scale_pitch', 0.8)

        # 方向反转
        self.invert_yaw = rospy.get_param('~invert_yaw', False)
        self.invert_pitch = rospy.get_param('~invert_pitch', False)

        # 状态
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.last_yaw_cmd = 0
        self.last_pitch_cmd = 0
        self.rc = None
        self.last_time = rospy.Time.now()
        self.est_yaw = 0.0
        self.est_pitch = 0.0

        rospy.Subscriber('/mavros/rc/in', RCIn, self.rc_cb)
        rospy.Timer(rospy.Duration(1.0 / self.repeat_hz), self.on_timer)
        rospy.loginfo('rc_to_udp_gimbal mode=%s | Yaw CH%d [%d..%d] center=%d db=%d | Pitch CH%d [%d..%d] center=%d db=%d',
                      self.control_mode,
                      self.ch_yaw, self.pwm_min_yaw, self.pwm_max_yaw, self.center_yaw, self.deadband_yaw,
                      self.ch_pitch, self.pwm_min_pitch, self.pwm_max_pitch, self.center_pitch, self.deadband_pitch)

    def rc_cb(self, msg):
        self.rc = msg

    @staticmethod
    def calc_checksum(cmd):
        return "%02X" % (sum(ord(c) for c in cmd) & 0xFF)

    def send(self, cmd):
        full = cmd + self.calc_checksum(cmd)
        self.sock.sendto(full.encode(), (self.cam_ip, self.port))

    @staticmethod
    def to_hex_signed(v):
        if v < 0:
            v = 256 + v
        return "%02X" % int(v)

    def rc_to_speed_centered(self, pwm, center, span, max_speed):
        if pwm < 900 or pwm > 2100:
            return 0, True
        delta = pwm - center
        if abs(delta) < 1:
            return 0, False
        norm = max(-1.0, min(1.0, float(delta) / float(span)))
        return int(norm * max_speed), False

    @staticmethod
    def map_pwm_linear(pwm, in_min, in_max, out_min, out_max):
        if in_max == in_min:
            return (out_min + out_max) * 0.5
        if pwm < in_min:
            pwm = in_min
        if pwm > in_max:
            pwm = in_max
        ratio = float(pwm - in_min) / float(in_max - in_min)
        return out_min + ratio * (out_max - out_min)

    def speed_from_deadband(self, pwm, center, deadband, min_pwm, max_pwm, max_speed):
        # 中心区间不动，越远速度越大，端点达到max_speed
        if pwm < 900 or pwm > 2100:
            return 0, True
        offset = pwm - center
        if -deadband <= offset <= deadband:
            return 0, False
        if offset > 0:
            distance = offset - deadband
            max_dist = max(1.0, (max_pwm - center) - deadband)
            ratio = min(1.0, max(0.0, distance / max_dist))
            spd = int(round(ratio * max_speed))
            return spd, False
        else:
            distance = (-offset) - deadband
            max_dist = max(1.0, (center - min_pwm) - deadband)
            ratio = min(1.0, max(0.0, distance / max_dist))
            spd = -int(round(ratio * max_speed))
            return spd, False

    def on_timer(self, _):
        if self.rc is None:
            return
        ch = self.rc.channels

        def get_with_check(i, default=1500):
            if 0 < i <= len(ch):
                return ch[i - 1]
            else:
                rospy.logwarn_throttle(2.0, 'Requested CH%d but only %d channels available', i, len(ch))
                return default

        yaw_pwm = get_with_check(self.ch_yaw)
        pit_pwm = get_with_check(self.ch_pitch)

        yaw_sign = -1.0 if self.invert_yaw else 1.0
        pit_sign = -1.0 if self.invert_pitch else 1.0

        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        if dt <= 0:
            dt = 1.0 / max(self.repeat_hz, 1.0)
        self.last_time = now

        # 模式：deadband_speed / speed / position
        if self.control_mode == 'deadband_speed':
            yaw_spd, failsafe = self.speed_from_deadband(yaw_pwm, self.center_yaw, self.deadband_yaw,
                                                         self.pwm_min_yaw, self.pwm_max_yaw, self.max_yaw)
            pit_spd, _ = self.speed_from_deadband(pit_pwm, self.center_pitch, self.deadband_pitch,
                                                  self.pwm_min_pitch, self.pwm_max_pitch, self.max_pitch)
            yaw_spd = int(yaw_sign * yaw_spd)
            pit_spd = int(pit_sign * pit_spd)

            if failsafe:
                self.send('#TPUG2wGSY00')
                self.send('#TPUG2wGSP00')
                self.last_yaw_cmd = 0
                self.last_pitch_cmd = 0
                return

            if yaw_spd == 0:
                self.send('#TPUG2wGSY00')
            else:
                self.send('#TPUG2wGSY' + self.to_hex_signed(yaw_spd))
            self.last_yaw_cmd = yaw_spd

            if pit_spd == 0:
                self.send('#TPUG2wGSP00')
            else:
                self.send('#TPUG2wGSP' + self.to_hex_signed(pit_spd))
            self.last_pitch_cmd = pit_spd
            return

        if self.control_mode == 'speed':
            # 兼容：以1500为中心、±450 满量程
            yaw_spd, failsafe = self.rc_to_speed_centered(yaw_pwm, 1500, 450.0, self.max_yaw)
            pit_spd, _ = self.rc_to_speed_centered(pit_pwm, 1500, 450.0, self.max_pitch)
            yaw_spd = int(yaw_sign * yaw_spd)
            pit_spd = int(pit_sign * pit_spd)

            if failsafe:
                self.send('#TPUG2wGSY00')
                self.send('#TPUG2wGSP00')
                self.last_yaw_cmd = 0
                self.last_pitch_cmd = 0
                return

            if yaw_spd == 0:
                self.send('#TPUG2wGSY00')
            else:
                self.send('#TPUG2wGSY' + self.to_hex_signed(yaw_spd))
            self.last_yaw_cmd = yaw_spd

            if pit_spd == 0:
                self.send('#TPUG2wGSP00')
            else:
                self.send('#TPUG2wGSP' + self.to_hex_signed(pit_spd))
            self.last_pitch_cmd = pit_spd
            return

        # 位置模式：PWM -> 角度端点（内部速度闭环）
        tgt_yaw = self.map_pwm_linear(yaw_pwm, self.pwm_min_yaw, self.pwm_max_yaw,
                                      self.yaw_min_deg, self.yaw_max_deg) * yaw_sign
        tgt_pitch = self.map_pwm_linear(pit_pwm, self.pwm_min_pitch, self.pwm_max_pitch,
                                        self.pitch_min_deg, self.pitch_max_deg) * pit_sign

        yaw_spd = int(max(-self.max_yaw, min(self.max_yaw, self.kp_yaw * (tgt_yaw - self.est_yaw))))
        pit_spd = int(max(-self.max_pitch, min(self.max_pitch, self.kp_pitch * (tgt_pitch - self.est_pitch))))

        if yaw_spd != self.last_yaw_cmd:
            if yaw_spd == 0:
                self.send('#TPUG2wGSY00')
            else:
                self.send('#TPUG2wGSY' + self.to_hex_signed(yaw_spd))
            self.last_yaw_cmd = yaw_spd

        if pit_spd != self.last_pitch_cmd:
            if pit_spd == 0:
                self.send('#TPUG2wGSP00')
            else:
                self.send('#TPUG2wGSP' + self.to_hex_signed(pit_spd))
            self.last_pitch_cmd = pit_spd

        # 估计角度（度）
        self.est_yaw += self.speed_scale_yaw * float(self.last_yaw_cmd) * dt
        self.est_pitch += self.speed_scale_pitch * float(self.last_pitch_cmd) * dt

    def shutdown(self):
        try:
            self.send('#TPUG2wGSY00')
            self.send('#TPUG2wGSP00')
        except Exception:
            pass
        self.sock.close()


if __name__ == '__main__':
    rospy.init_node('rc_to_udp_gimbal')
    node = RcToGimbal()
    rospy.on_shutdown(node.shutdown)
    rospy.spin()
