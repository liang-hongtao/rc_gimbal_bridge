#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from mavros_msgs.msg import RCIn
import time


class RcChannelMonitor:
    def __init__(self):
        self.change_thresh = rospy.get_param('~change_thresh', 25)  # us
        self.print_every = rospy.get_param('~print_every', 0.5)     # s
        self.show_all = rospy.get_param('~show_all', False)
        self.max_channels = rospy.get_param('~max_channels', 18)

        self.last_values = None
        self.min_values = None
        self.max_values = None
        self.last_print = rospy.Time.now()
        self.last_change_time = None

        rospy.Subscriber('/mavros/rc/in', RCIn, self.rc_cb)
        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo('RC channel monitor ready. Threshold=%dus', self.change_thresh)

    def rc_cb(self, msg):
        ch = list(msg.channels)
        n = len(ch)
        now = rospy.Time.now()

        if self.last_values is None:
            self.last_values = ch[:]
            self.min_values = ch[:] + [1500] * (self.max_channels - n)
            self.max_values = ch[:] + [1500] * (self.max_channels - n)
            self.last_change_time = [now] * self.max_channels
            return

        # extend arrays if needed
        if len(self.last_values) < n:
            extend_cnt = n - len(self.last_values)
            self.last_values += [1500] * extend_cnt
            self.min_values += [1500] * extend_cnt
            self.max_values += [1500] * extend_cnt
            self.last_change_time += [now] * extend_cnt

        changed = []
        for i in range(n):
            v_prev = self.last_values[i]
            v = ch[i]
            if abs(v - v_prev) >= self.change_thresh:
                changed.append((i + 1, v, v - v_prev))  # CH index is 1-based
                self.last_values[i] = v
                self.last_change_time[i] = now
            # update min/max
            if v < self.min_values[i]:
                self.min_values[i] = v
            if v > self.max_values[i]:
                self.max_values[i] = v

        # print periodically
        if (now - self.last_print).to_sec() >= self.print_every:
            self.last_print = now
            stamp = time.strftime('%H:%M:%S')
            if changed:
                parts = [
                    'CH{0}: {1} ({2:+d})'.format(idx, val, dv) for idx, val, dv in changed
                ]
                rospy.loginfo('[%s] channels=%d changed: %s', stamp, n, '  '.join(parts))
            else:
                rospy.loginfo('[%s] channels=%d (no significant change)', stamp, n)

            if self.show_all:
                # compact line for all available channels
                vals = '  '.join('CH{0}:{1}'.format(i + 1, ch[i]) for i in range(n))
                rospy.loginfo('ALL: %s', vals)

    def on_shutdown(self):
        if self.min_values is None:
            return
        n = len(self.last_values)
        rospy.loginfo('Summary (min..max):')
        for i in range(n):
            rospy.loginfo('  CH%-2d  %4d .. %4d', i + 1, self.min_values[i], self.max_values[i])


if __name__ == '__main__':
    rospy.init_node('rc_channel_monitor')
    RcChannelMonitor()
    rospy.spin()

