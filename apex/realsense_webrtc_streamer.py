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

        self.create_subscription(Image, '/camera/cam_left_wrist/color/image_raw', self.cb_left, 10)
        self.create_subscription(Image, '/camera/cam_head/color/image_raw', self.cb_head, 10)
        self.create_subscription(Image, '/camera/cam_right_wrist/color/image_raw', self.cb_right, 10)

    def cb_left(self, msg):  self.left = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    def cb_right(self, msg): self.right = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    def cb_head(self, msg):  self.head = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def get_frame(self):
        # 💡 强行锁定 16:9 标准全屏分辨率，无论怎么全屏都不会拉伸变形！
        W_target, H_target = 1920, 1080
        
        # 1. 创建 16:9 全黑大画布 (1080 x 1920 x 3)
        canvas = np.zeros((H_target, W_target, 3), dtype=np.uint8)
        
        # 2. 🔥 基于 16:9 比例精准计算坐标，保持各相机原生宽高比不拉伸
        grid_configs = [
            # 🎥 1. 头部相机 (D435i 原生 16:9，缩放至 1280x720，水平居中: x=320)
            {
                "img": self.head,  
                "start_x": 320,   
                "start_y": 0,   
                "target_w": 1280, 
                "target_h": 720
            },
            # 🤖 2. 左手相机 (D405 原生 4:3，缩放至 480x360，放在左下: x=480, y=720)
            {
                "img": self.left,  
                "start_x": 480,   
                "start_y": 720, 
                "target_w": 480,  
                "target_h": 360
            },
            # 🤖 3. 右手相机 (D405 原生 4:3，缩放至 480x360，放在右下: x=960, y=720)
            {
                "img": self.right, 
                "start_x": 960, 
                "start_y": 720, 
                "target_w": 480,  
                "target_h": 360
            }
        ]

        # 3. 遍历三路视频流贴图
        for config in grid_configs:
            src_img = config["img"]
            if src_img is None:
                continue
                
            try:
                # 强行切出 RGB 3 通道
                img = src_img[:, :, :3]
                
                x = config["start_x"]
                y = config["start_y"]
                w = config["target_w"]
                h = config["target_h"]
                
                # 如果尺寸不匹配，进行等比 resize
                if img.shape[1] != w or img.shape[0] != h:
                    img_resized = cv2.resize(img, (w, h), interpolation=cv2.INTER_LINEAR)
                else:
                    img_resized = img
                    
                # 填充到画布对应区域
                canvas[y:y+h, x:x+w] = img_resized
                
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
        self.node = node
        self.start = time.time()

    async def recv(self):
        # 强制控制 30 FPS，防止死锁
        await asyncio.sleep(1/30)
        frame = self.node.get_frame()
        if frame is None:
            frame = np.zeros((1080, 1920, 3), dtype=np.uint8)

        # 将 BGR 转换为浏览器/遥操软件硬解唯一通用的 YUV420p 格式！
        frame_yuv = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV_I420)

        # 格式必须标明为 yuv420p
        video_frame = VideoFrame.from_ndarray(frame_yuv, format="yuv420p")

        # 注入严格递增的时间戳（PTS）
        video_frame.pts = int((time.time() - self.start) * 90000)
        video_frame.time_base = Fraction(1, 90000)

        return video_frame

pcs = set()
relay = MediaRelay()

# =========================
# 💡 根路径智能分流器（同时支持 WebSocket 软件 & 浏览器 HTTP）
# =========================
async def smart_root_handler(request):
    # 判断是否为 WebSocket 升级请求
    if request.headers.get('Upgrade', '').lower() == 'websocket':
        # ----------------------------------------------
        # 1. 走 Windows 遥操软件的 WebSocket 握手通道
        # ----------------------------------------------
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        sys.stdout.write(f"[WebSocket] 客户端连接已建立\n")
        sys.stdout.flush()
        
        pc = RTCPeerConnection()
        pcs.add(pc)

        # 调试日志 + 状态跟踪
        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            sys.stdout.write(f"[WebRTC] connectionState -> {pc.connectionState}\n")
            sys.stdout.flush()
            if pc.connectionState in ("failed", "disconnected", "closed"):
                sys.stdout.write(f"[WebRTC] 连接结束 ({pc.connectionState})，清理 PeerConnection\n")
                sys.stdout.flush()
                await pc.close()
                pcs.discard(pc)

        # ICE 收集完成事件
        ice_gathering_complete = asyncio.Event()

        @pc.on("icegatheringstatechange")
        async def on_icegatheringstatechange():
            sys.stdout.write(f"[WebRTC] iceGatheringState -> {pc.iceGatheringState}\n")
            sys.stdout.flush()
            if pc.iceGatheringState == "complete":
                ice_gathering_complete.set()

        # Trickle ICE：逐条将 ICE candidate 推送给远端
        @pc.on("icecandidate")
        async def on_icecandidate(candidate):
            sys.stdout.write(f"[WebRTC] ICE candidate: {str(candidate)}\n")
            sys.stdout.flush()
            if candidate is None:
                # None 表示收集结束，不用再发
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
                    sys.stdout.write(f"[WebSocket] 收到客户端 Offer（SDP 长度: {len(data.get('sdp',''))}）\n")
                    sys.stdout.flush()
                    offer = RTCSessionDescription(sdp=data["sdp"], type=data["type"])
                    await pc.setRemoteDescription(offer)
                    answer = await pc.createAnswer()
                    await pc.setLocalDescription(answer)
                    
                    # 等待 ICE gathering 完成，确保 SDP 包含完整 candidates
                    if pc.iceGatheringState != "complete":
                        sys.stdout.write(f"[WebRTC] 等待 ICE gathering 完成...\n")
                        sys.stdout.flush()
                    else:
                        ice_gathering_complete.set()
                    await ice_gathering_complete.wait()
                    
                    # 重新读取 SDP（此时已包含所有 ICE candidates）
                    sys.stdout.write(f"[WebSocket] 发送 Answer（SDP 长度: {len(pc.localDescription.sdp)}）\n")
                    sys.stdout.flush()
                    await ws.send_json({
                        "sdp": pc.localDescription.sdp,
                        "type": pc.localDescription.type
                    })
        sys.stdout.write(f"[WebSocket] 客户端会话结束\n")
        sys.stdout.flush()
        return ws
    else:
        # ----------------------------------------------
        # 2. 走普通网页浏览器的 HTTP 访问通道
        # ----------------------------------------------
        html = """
        <!DOCTYPE html>
        <html>
        <head>
            <meta charset="utf-8"/>
            <title>VR Stream</title>
            <style>
                body { margin: 0; background: black; display: flex; justify-content: center; align-items: center; height: 100vh; overflow: hidden; }
                video { width: 100vw; height: auto; object-fit: contain; cursor: pointer; }
                #tip { position: absolute; color: white; background: rgba(0,0,0,0.6); padding: 10px 20px; border-radius: 5px; pointer-events: none; }
            </style>
        </head>
        <body>
        <div id="tip">点击屏幕激活视频流</div>
        <video id="v" autoplay playsinline muted style="width:100vw;"></video>
        <script>
        const videoElement = document.getElementById('v');
        const tipElement = document.getElementById('tip');
        window.addEventListener('click', () => {
            videoElement.play().then(() => { tipElement.style.opacity = 0; }).catch(console.error);
        });
        const pc = new RTCPeerConnection({ iceServers: [] });
        pc.ontrack = (event) => {
            videoElement.srcObject = (event.streams && event.streams[0]) ? event.streams[0] : new MediaStream([event.track]);
            videoElement.play().then(() => { tipElement.style.opacity = 0; }).catch(() => {});
        };
        pc.addTransceiver('video', { direction: 'recvonly' });
        pc.createOffer()
        .then(offer => pc.setLocalDescription(offer))
        .then(() => fetch('/offer', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({ sdp: pc.localDescription.sdp, type: pc.localDescription.type })
        }))
        .then(r => r.json())
        .then(ans => pc.setRemoteDescription(new RTCSessionDescription(ans)))
        .catch(console.error);
        </script>
        </body>
        </html>
        """
        return web.Response(text=html, content_type='text/html')

async def offer(request):
    params = await request.json()
    offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
    pc = RTCPeerConnection()
    pcs.add(pc)

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        sys.stdout.write(f"[WebRTC/HTTP] connectionState -> {pc.connectionState}\n")
        sys.stdout.flush()
        if pc.connectionState in ("failed", "disconnected", "closed"):
            sys.stdout.write(f"[WebRTC/HTTP] 连接结束 ({pc.connectionState})，清理 PeerConnection\n")
            sys.stdout.flush()
            await pc.close()
            pcs.discard(pc)

    # ICE 收集完成事件
    ice_gathering_complete = asyncio.Event()

    @pc.on("icegatheringstatechange")
    async def on_icegatheringstatechange():
        sys.stdout.write(f"[WebRTC/HTTP] iceGatheringState -> {pc.iceGatheringState}\n")
        sys.stdout.flush()
        if pc.iceGatheringState == "complete":
            ice_gathering_complete.set()

    @pc.on("icecandidate")
    async def on_icecandidate(candidate):
        sys.stdout.write(f"[WebRTC/HTTP] ICE candidate: {str(candidate)}\n")
        sys.stdout.flush()

    video_track = request.app["video_track"]
    pc.addTrack(relay.subscribe(video_track))

    await pc.setRemoteDescription(offer)
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)

    # 等待 ICE gathering 完成，确保 SDP 包含完整 candidates
    if pc.iceGatheringState != "complete":
        sys.stdout.write(f"[WebRTC/HTTP] 等待 ICE gathering 完成...\n")
        sys.stdout.flush()
    else:
        ice_gathering_complete.set()
    await ice_gathering_complete.wait()

    sys.stdout.write(f"[WebRTC/HTTP] 发送 Answer（SDP 长度: {len(pc.localDescription.sdp)}）\n")
    sys.stdout.flush()
    return web.Response(
        content_type="application/json",
        text=json.dumps({"sdp": pc.localDescription.sdp, "type": pc.localDescription.type})
    )

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

    # 💡 关键修改：根路径 / 同时兼容 ws:// 和 http://
    app.router.add_get("/", smart_root_handler)
    app.router.add_post("/offer", offer)
    app.router.add_get("/my_output", smart_root_handler)  # 保持兼容性

    web.run_app(app, host="0.0.0.0", port=8554)

if __name__ == "__main__":
    main()