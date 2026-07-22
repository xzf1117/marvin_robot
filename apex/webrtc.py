#!/usr/bin/env python3
import asyncio
import json
import threading
import numpy as np
import cv2
import sys
import time
import os
from fractions import Fraction

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import aiohttp
from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription, MediaStreamTrack
from aiortc.contrib.media import MediaRelay
from av import VideoFrame

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

        self.create_subscription(Image, '/camera/cam_left_wrist/color/image_raw', self.cb_left, 10)
        self.create_subscription(Image, '/camera/cam_head/color/image_raw', self.cb_head, 10)
        self.create_subscription(Image, '/camera/cam_right_wrist/color/image_raw', self.cb_right, 10)

    def cb_left(self, msg):  self.left = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    def cb_right(self, msg): self.right = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    def cb_head(self, msg):  self.head = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def get_frame(self):
        W_target, H_target = 1920, 1080
        canvas = np.zeros((H_target, W_target, 3), dtype=np.uint8)
        
        grid_configs = [
            {"img": self.head,  "start_x": 320, "start_y": 0,   "target_w": 1280, "target_h": 720},
            {"img": self.left,  "start_x": 480, "start_y": 720, "target_w": 480,  "target_h": 360},
            {"img": self.right, "start_x": 960, "start_y": 720, "target_w": 480,  "target_h": 360}
        ]

        for config in grid_configs:
            src_img = config["img"]
            if src_img is None:
                continue
                
            try:
                img = src_img[:, :, :3]
                x, y = config["start_x"], config["start_y"]
                w, h = config["target_w"], config["target_h"]
                
                if img.shape[1] != w or img.shape[0] != h:
                    img_resized = cv2.resize(img, (w, h), interpolation=cv2.INTER_LINEAR)
                else:
                    img_resized = img
                    
                canvas[y:y+h, x:x+w] = img_resized
            except Exception as e:
                sys.stdout.write(f"❌ 拼图处理异常: {e}\n")
                sys.stdout.flush()
                
        return canvas

# =========================
# WebRTC 视频流轨道
# =========================
class VideoTrack(MediaStreamTrack):
    kind = "video"

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.start = time.time()

    async def recv(self):
        await asyncio.sleep(1/30)  # 锁 30 FPS
        frame = self.node.get_frame()
        if frame is None:
            frame = np.zeros((1080, 1920, 3), dtype=np.uint8)

        frame_yuv = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV_I420)
        video_frame = VideoFrame.from_ndarray(frame_yuv, format="yuv420p")

        video_frame.pts = int((time.time() - self.start) * 90000)
        video_frame.time_base = Fraction(1, 90000)

        return video_frame

pcs = set()
relay = MediaRelay()

# =========================
# 纯 WebSocket 客户端信令处理 handler
# =========================
async def websocket_handler(request):
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    sys.stdout.write(f"[WebSocket] 新客户端连接已建立\n")
    sys.stdout.flush()
    
    pc = RTCPeerConnection()
    pcs.add(pc)

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        sys.stdout.write(f"[WebRTC] connectionState -> {pc.connectionState}\n")
        sys.stdout.flush()
        if pc.connectionState in ("failed", "disconnected", "closed"):
            sys.stdout.write(f"[WebRTC] 连接断开，清理 PeerConnection\n")
            sys.stdout.flush()
            await pc.close()
            pcs.discard(pc)

    ice_gathering_complete = asyncio.Event()

    @pc.on("icegatheringstatechange")
    async def on_icegatheringstatechange():
        sys.stdout.write(f"[WebRTC] iceGatheringState -> {pc.iceGatheringState}\n")
        sys.stdout.flush()
        if pc.iceGatheringState == "complete":
            ice_gathering_complete.set()

    @pc.on("icecandidate")
    async def on_icecandidate(candidate):
        if candidate is None:
            return
        if not ws.closed:
            try:
                await ws.send_json({
                    "type": "candidate",
                    "candidate": candidate.candidate,
                    "sdpMid": candidate.sdpMid,
                    "sdpMLineIndex": candidate.sdpMLineIndex
                })
            except Exception as e:
                sys.stdout.write(f"[WebRTC] 发送 ICE candidate 失败: {e}\n")
                sys.stdout.flush()

    video_track = request.app["video_track"]
    pc.addTrack(relay.subscribe(video_track))

    async for msg in ws:
        if msg.type == aiohttp.WSMsgType.TEXT:
            data = json.loads(msg.data)
            if data.get("type") == "offer":
                sys.stdout.write(f"[WebSocket] 收到 Offer (SDP 长度: {len(data.get('sdp',''))})\n")
                sys.stdout.flush()
                
                offer = RTCSessionDescription(sdp=data["sdp"], type=data["type"])
                await pc.setRemoteDescription(offer)
                
                answer = await pc.createAnswer()
                await pc.setLocalDescription(answer)
                
                if pc.iceGatheringState != "complete":
                    sys.stdout.write(f"[WebRTC] 等待 ICE Gathering 完成...\n")
                    sys.stdout.flush()
                else:
                    ice_gathering_complete.set()
                    
                await ice_gathering_complete.wait()
                
                sys.stdout.write(f"[WebSocket] 回复 Answer (SDP 长度: {len(pc.localDescription.sdp)})\n")
                sys.stdout.flush()
                
                await ws.send_json({
                    "sdp": pc.localDescription.sdp,
                    "type": pc.localDescription.type
                })

    sys.stdout.write(f"[WebSocket] 客户端已断开\n")
    sys.stdout.flush()
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

    # 只保留 WebSocket 端点 (同时映射到 '/' 与 '/ws' 路径)
    app.router.add_get("/", websocket_handler)
    app.router.add_get("/ws", websocket_handler)

    web.run_app(app, host="0.0.0.0", port=8554)

if __name__ == "__main__":
    main()