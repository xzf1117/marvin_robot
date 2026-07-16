#!/usr/bin/env python3
import asyncio
import json
import threading
import numpy as np
import cv2
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import aiohttp
from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription, MediaStreamTrack
from aiortc.contrib.media import MediaRelay
from av import VideoFrame
from fractions import Fraction

import os
os.environ["AIORTC_LOCAL_ADDRESS"] = "192.168.1.50"
# =========================
# ROS2 订阅节点
# =========================
class CameraNode(Node):
    def __init__(self):
        super().__init__('vr_stitch_node')
        self.bridge = CvBridge()

        self.left = None
        self.right = None
        self.head = None
        self.frame_count = 0

        self.create_subscription(Image, '/camera/cam_left_wrist/color/image_raw', self.cb_left, 10)
        self.create_subscription(Image, '/camera/cam_head_launch/color/image_raw', self.cb_head, 10)
        self.create_subscription(Image, '/camera/cam_right_wrist/color/image_raw', self.cb_right, 10)

    def cb_left(self, msg):  self.left = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    def cb_right(self, msg): self.right = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    def cb_head(self, msg):
        self.head = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # self.frame_count += 1
        # if self.frame_count % 30 == 0:
        #     sys.stdout.write("📡 [ROS2 节点] 头部相机画面接收正常...\n")
        #     sys.stdout.flush()

    def get_frame(self):
        H_target, W_target = 992, 2560
        # 1. 创建全黑大画布
        canvas = np.zeros((H_target, W_target, 3), dtype=np.uint8)
        
        # 2. 定义三路格子的左边界和宽度目标
        # 左格子: start=0,    width=853
        # 中格子: start=853,  width=854
        # 右格子: start=1707, width=853
        grid_configs = [
            {"img": self.left,  "start_x": 0,    "target_w": 853},
            {"img": self.head,  "start_x": 853,  "target_w": 854},
            {"img": self.right, "start_x": 1707, "target_w": 853}
        ]

        for config in grid_configs:
            src_img = config["img"]
            if src_img is None:
                continue
                
            try:
                # 强行切出前 3 通道，获取原始高、宽
                src_img = src_img[:, :, :3]
                h_src, w_src = src_img.shape[:2]
                
                target_w = config["target_w"]
                target_h = H_target
                
                # 3. 💡 核心算法：计算等比例缩放因子 (取宽、高缩放比的最小值)
                scale = min(target_w / w_src, target_h / h_src)
                new_w = int(w_src * scale)
                new_h = int(h_src * scale)
                
                # 等比缩放图像
                resized_img = cv2.resize(src_img, (new_w, new_h))
                
                # 4. 计算居中偏移量 (确保画面在格子里水平居中、垂直居中)
                dx = (target_w - new_w) // 2
                dy = (target_h - new_h) // 2
                
                # 计算在整个 canvas 上的绝对坐标大门
                x_start = config["start_x"] + dx
                x_end = x_start + new_w
                y_start = dy
                y_end = y_start + new_h
                
                # 将等比帧完美嵌入画布中央
                canvas[y_start:y_end, x_start:x_end] = resized_img
                
            except Exception as e:
                sys.stdout.write(f"❌ 动态等比拼图失败: {e}\n")
                sys.stdout.flush()
                
        return canvas

# =========================
# WebRTC 视频流轨道
# =========================
class VideoTrack(MediaStreamTrack):
    kind = "video"

    def __init__(self, node):
        super().__init__()
        self.node = node # 确保这里叫 self.node
        self.start = time.time()

    async def recv(self):
        # 强制控制 30 FPS，防止死锁
        await asyncio.sleep(1/30)
        frame = self.node.get_frame()
        if frame is None:
            frame = np.zeros((992, 2560, 3), dtype=np.uint8)

        # 将 BGR 转换为浏览器硬解唯一通用的 YUV420p 格式！
        frame_yuv = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV_I420)

        # 格式必须标明为 yuv420p
        video_frame = VideoFrame.from_ndarray(frame_yuv, format="yuv420p")

        # 注入严格递增的时间戳（PTS）
        video_frame.pts = int((time.time() - self.start) * 90000)
        video_frame.time_base = Fraction(1, 90000)

        return video_frame
    
pcs = set()
relay = MediaRelay()

async def offer(request):
    params = await request.json()
    offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
    pc = RTCPeerConnection()
    pcs.add(pc)

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        if pc.connectionState == "failed":
            await pc.close()
            pcs.discard(pc)

    # 使用全局 relay 转发单例轨道，防止刷新网页或断开重连时重复创建 Track 抢占死锁
    video_track = request.app["video_track"]
    pc.addTrack(relay.subscribe(video_track))

    await pc.setRemoteDescription(offer)
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)

    return web.Response(
        content_type="application/json",
        text=json.dumps({"sdp": pc.localDescription.sdp, "type": pc.localDescription.type})
    )

async def index(request):
    html = """
    <!DOCTYPE html>
    <html>
    <head>
        <meta charset="utf-8"/>
        <title>VR Stream</title>
        <style>
            body { margin: 0; background: black; display: flex; justify-content: center; align-items: center; height: 100vh; overflow: hidden; font-family: sans-serif; }
            video { width: 100vw; height: auto; object-fit: contain; cursor: pointer; }
            #tip { position: absolute; color: white; background: rgba(0,0,0,0.6); padding: 10px 20px; border-radius: 5px; pointer-events: none; transition: opacity 0.5s; z-index: 10; }
        </style>
    </head>
    <body>
    <div id="tip">点击屏幕激活视频流</div>
    <video id="v" autoplay playsinline muted style="width:100vw;"></video>

    <script>
    const videoElement = document.getElementById('v');
    const tipElement = document.getElementById('tip');

    window.addEventListener('click', () => {
        videoElement.play().then(() => {
            tipElement.style.opacity = 0;
            console.log("🍏 用户手动唤醒成功！");
        }).catch(console.error);
    });

    const pc = new RTCPeerConnection({ iceServers: [] });

    pc.ontrack = (event) => {
        if (event.streams && event.streams[0]) {
            videoElement.srcObject = event.streams[0];
        } else {
            const inboundStream = new MediaStream([event.track]);
            videoElement.srcObject = inboundStream;
        }
        videoElement.play().then(() => { tipElement.style.opacity = 0; }).catch(() => {});
    };

    pc.addTransceiver('video', { direction: 'recvonly' });

    pc.createOffer()
    .then(offer => pc.setLocalDescription(offer))
    .then(() => {
        // 💡 网页内部握手统一发给 /offer 路径，彻底与软件的 /my_output 隔开！
        return fetch('/offer', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({ sdp: pc.localDescription.sdp, type: pc.localDescription.type })
        });
    })
    .then(r => r.json())
    .then(ans => pc.setRemoteDescription(new RTCSessionDescription(ans)))
    .catch(console.error);
    </script>
    </body>
    </html>
    """
    return web.Response(text=html, content_type='text/html')

# =========================
# 💡 Windows 遥操软件专用 WebSocket 处理器 (不再掺杂任何网页)
# =========================
async def websocket_handler(request):
    if request.headers.get('Upgrade', '').lower() != 'websocket':
        # 如果不是标准软件升级请求，直接拒绝，不再返回网页，斩断冲突循环
        return web.Response(status=400, text="Please use a WebSocket client to connect.")
        
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    
    pc = RTCPeerConnection()
    pcs.add(pc)

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        if pc.connectionState == "failed":
            await pc.close()
            pcs.discard(pc)

    video_track = request.app["video_track"]
    pc.addTrack(relay.subscribe(video_track))

    async for msg in ws:
        if msg.type == aiohttp.WSMsgType.TEXT:
            data = json.loads(msg.data)
            if data.get("type") == "offer":
                offer = RTCSessionDescription(sdp=data["sdp"], type=data["type"])
                await pc.setRemoteDescription(offer)
                answer = await pc.createAnswer()
                await pc.setLocalDescription(answer)
                
                await ws.send_json({
                    "sdp": pc.localDescription.sdp,
                    "type": pc.localDescription.type
                })
    return ws

def ros_thread(node):
    rclpy.spin(node)


def main():
    rclpy.init()
    node = CameraNode()

    global_track = VideoTrack(node)

    t = threading.Thread(target=ros_thread, args=(node,), daemon=True)
    t.start()

    app = web.Application()
    app["node"] = node
    app["video_track"] = global_track 

    app.router.add_get("/", index)                       # 1. 纯网页入口：浏览器访问 http://192.168.1.50:8554 走这里
    app.router.add_post("/offer", offer)                 # 2. 纯网页握手：不影响 /my_output
    app.router.add_get("/my_output", websocket_handler)  # 3. 纯软件入口：天机 Windows 软件 WebSocket 唯一的专属通道！

    web.run_app(app, host="0.0.0.0", port=8554)

if __name__ == "__main__":
    main()

# http://192.168.1.50:8554/