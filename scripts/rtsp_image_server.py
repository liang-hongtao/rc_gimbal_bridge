#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import time
from dataclasses import dataclass, field
from typing import Optional, Dict

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GLib, GstRtspServer


@dataclass
class StreamSpec:
    name: str
    topic: str
    mountpoint: str
    width: Optional[int] = None
    height: Optional[int] = None
    framerate: int = 20
    bitrate: int = 1500  # kbps
    format: str = "BGR"  # internal appsrc caps format
    # runtime fields
    appsrc = None
    initiated: bool = False
    frame_count: int = 0
    last_shape = None
    lock: threading.Lock = field(default_factory=threading.Lock)


class ImageRtspServer(object):
    def __init__(self):
        rospy.init_node('image_rtsp_server')

        # Init GStreamer
        Gst.init(None)

        self.bridge = CvBridge()
        self.server = GstRtspServer.RTSPServer()

        # 优先读取私有参数，其次回退到全局参数
        port = rospy.get_param('~port', None)
        if port is None:
            port = rospy.get_param('port', 8554)
        self.server.props.service = str(port)
        self.mounts = self.server.get_mount_points()

        # Load streams param (private first, then global)
        streams_param = rospy.get_param('~streams', None)
        if streams_param is None:
            streams_param = rospy.get_param('streams', {})
        if not isinstance(streams_param, dict) or not streams_param:
            rospy.logerr('~streams 参数缺失或格式错误，应为字典。')
            raise SystemExit(1)

        self.streams: Dict[str, StreamSpec] = {}
        for key, cfg in streams_param.items():
            try:
                spec = StreamSpec(
                    name=key,
                    topic=str(cfg.get('source', cfg.get('topic', ''))),
                    mountpoint=str(cfg.get('mountpoint', f'/{key}')),
                    width=cfg.get('width'),
                    height=cfg.get('height'),
                    framerate=int(cfg.get('framerate', 20)),
                    bitrate=int(cfg.get('bitrate', 1500)),
                    format=str(cfg.get('format', 'BGR')).upper(),
                )
            except Exception as e:
                rospy.logerr(f'解析流配置 {key} 失败: {e}')
                raise

            if not spec.topic:
                rospy.logerr(f'流 {key} 未提供 topic/source 参数')
                raise SystemExit(1)

            self.streams[key] = spec

        # Start GLib mainloop for RTSP server
        self.mainloop = GLib.MainLoop()
        self.mainloop_thread = threading.Thread(target=self.mainloop.run, name='glib-mainloop', daemon=True)
        self.mainloop_thread.start()

        # Attach server
        self.server.attach(None)
        rospy.loginfo(f'RTSP 服务监听端口: {port}')

        # Subscribers per stream
        for key, spec in self.streams.items():
            rospy.Subscriber(spec.topic, Image, self._make_img_cb(spec), queue_size=1, buff_size=2**24)
            rospy.loginfo(f'已订阅图像话题: {spec.topic} -> rtsp://0.0.0.0:{port}{spec.mountpoint}')

    def _make_img_cb(self, spec: StreamSpec):
        def cb(msg: Image):
            try:
                # Convert to BGR
                cv_img = self._to_bgr(spec, msg)
                h, w = cv_img.shape[:2]

                # First frame: create RTSP media and mount
                if not spec.initiated:
                    with spec.lock:
                        if not spec.initiated:
                            # Fill width/height if missing
                            spec.width = spec.width or w
                            spec.height = spec.height or h
                            self._create_mount(spec)
                            spec.initiated = True

                # Drop frames until appsrc is available (no client yet)
                if spec.appsrc is None:
                    return

                # If resolution changed after init, skip (or extend to recreate)
                if (spec.width, spec.height) != (w, h):
                    # 简化处理：忽略分辨率切换，避免中断当前推流
                    return

                data = cv_img.tobytes()
                buf = Gst.Buffer.new_allocate(None, len(data), None)
                buf.fill(0, data)

                duration = int(1e9 / max(1, spec.framerate))
                buf.duration = duration
                spec.frame_count += 1
                # 让 appsrc 根据 do-timestamp 自动打时间戳

                ret = spec.appsrc.emit('push-buffer', buf)
                if ret != Gst.FlowReturn.OK:
                    rospy.logwarn_throttle(5.0, f'[{spec.name}] push-buffer 返回 {ret}')
            except Exception as e:
                rospy.logwarn_throttle(2.0, f'[{spec.name}] 处理图像失败: {e}')
        return cb

    def _to_bgr(self, spec: StreamSpec, msg: Image):
        # 尽量统一到 BGR，减少编码器负担
        if msg.encoding.lower() in ('bgr8', 'bgr'):  # already BGR
            return self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        elif msg.encoding.lower() in ('rgb8', 'rgb'):
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        elif msg.encoding.lower() in ('mono8', '8uc1', 'mono16'):
            # 灰度→伪彩 BGR（编码器更广泛支持 yuv420p）
            img = self.bridge.imgmsg_to_cv2(msg)
            if len(img.shape) == 2:
                return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            return img
        else:
            # 回退到 BGR8
            return self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def _create_mount(self, spec: StreamSpec):
        # Build pipeline string
        caps = f"video/x-raw,format={spec.format},width={spec.width},height={spec.height},framerate={spec.framerate}/1"
        # x264enc bitrate 单位 kbps；config-interval=1 定期发送 sps/pps，便于客户端随时加入
        launch_str = (
            f"appsrc name=src_{spec.name} is-live=true block=true do-timestamp=true format=GST_FORMAT_TIME caps=\"{caps}\" ! "
            f"videoconvert ! x264enc tune=zerolatency speed-preset=ultrafast bitrate={spec.bitrate} key-int-max={max(1, spec.framerate*2)} ! "
            f"rtph264pay name=pay0 pt=96 config-interval=1"
        )

        factory = GstRtspServer.RTSPMediaFactory()
        factory.set_launch(launch_str)
        factory.set_shared(True)

        def on_configure(factory, media, user_data):
            try:
                element = media.get_element()
                appsrc = element.get_child_by_name(f"src_{spec.name}")
                if appsrc is None:
                    rospy.logwarn(f'[{spec.name}] 找不到 appsrc')
                    return
                appsrc.set_property('format', Gst.Format.TIME)
                appsrc.set_property('is-live', True)
                appsrc.set_property('block', True)
                appsrc.set_property('do-timestamp', True)
                spec.appsrc = appsrc
                rospy.loginfo(f'[{spec.name}] RTSP 媒体已配置，等待客户端连接: {spec.mountpoint}')
            except Exception as e:
                rospy.logwarn(f'[{spec.name}] 配置媒体失败: {e}')

        factory.connect('media-configure', on_configure, None)

        self.mounts.add_factory(spec.mountpoint, factory)
        rospy.loginfo(f'挂载流: rtsp://0.0.0.0:{self.server.props.service}{spec.mountpoint} ({spec.width}x{spec.height}@{spec.framerate}, {spec.bitrate}kbps)')

    def spin(self):
        rospy.loginfo('Image RTSP Server 已启动。按 Ctrl+C 退出。')
        try:
            rospy.spin()
        finally:
            # 尝试优雅退出（RTSP server 在守护线程里，随主进程退出）
            try:
                self.mainloop.quit()
            except Exception:
                pass


if __name__ == '__main__':
    ImageRtspServer().spin()
