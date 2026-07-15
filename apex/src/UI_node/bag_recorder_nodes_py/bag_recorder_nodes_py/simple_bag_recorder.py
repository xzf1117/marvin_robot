import sys
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from std_msgs.msg import String, Int32
from std_srvs.srv import Trigger
from rclpy.topic_or_service_is_hidden import topic_or_service_is_hidden
import importlib
from pydantic import BaseModel
from typing import List

import rosbag2_py
import threading
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import time
from datetime import datetime
import os
from typing import Optional

# 统一FastAPI：在同一服务内管理视频与rosbag
# 尝试导入 Jetson 视频工具（若不可用则禁用视频服务）
try:
    from jetson_utils import (
        videoSource,
        videoOutput,
        Log,
        cudaImage,
        cudaOverlay,
        cudaResize,
        cudaFont,
    )
except Exception:
    videoSource = None
    videoOutput = None
    Log = None
    cudaImage = None
    cudaOverlay = None
    cudaResize = None
    cudaFont = None

app = FastAPI()
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # 或指定你的前端地址
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
recorder_node = None
storage_error_message = None  # 存储路径错误消息
# 统一服务中的视频对象与状态
video_streamer = None
MP4_DIR = "/home/marvin/KernelMind_Apex/BAG_STORAGE/videos"
video_state = {
    "start_requested": False,
    "stop_requested": False,
    "active": False,
    "filename": None,
}
video_lock = threading.Lock()
# 允许的录制话题白名单，仅保留规范命名
ALLOWED_TOPICS = {
    "/joint_states",
    "/gripper/feedback_L",
    "/gripper/feedback_R",
    "/info/eef_left",
    "/info/eef_right",
    "/control/joint_cmd_A",
    "/control/joint_cmd_B",
}

# 定义请求体模型
class TopicSelectionModel(BaseModel):
    topics: List[str]

# 获取话题列表API
@app.get("/get_topics")
def get_topics():
    if recorder_node is not None:
        topics = recorder_node.get_available_topics()
        return {"topics": topics}
    return {"topics": [], "error": "No recorder node available"}

# 修改录制API接收话题列表
@app.post("/start_record")
def start_record(topic_selection: TopicSelectionModel = None):
    if recorder_node is not None and not recorder_node.recording:
        try:
            if topic_selection and topic_selection.topics:
                recorder_node.start_recording(topic_selection.topics)
            else:
                recorder_node.start_recording()  # 如果没有指定话题，录制所有话题
            recorder_node._triggered_by_service = False  # 标记为FastAPI触发，非服务触发
            # 同步启动视频录制（如无需联动，可注释）
            _api_start_video_recording()
            return {"status": "started"}
        except FileNotFoundError as e:
            return {"status": "error", "error_message": str(e)}
        except Exception as e:
            return {"status": "error", "error_message": f"启动录制失败: {str(e)}"}
    return {"status": "already recording or no node"}

@app.post("/stop_record")
def stop_record():
    if recorder_node is not None and recorder_node.recording:
        recorder_node.stop_recording()  # 停止录制并关闭当前rosbag文件
        # 同步停止视频录制（如无需联动，可注释）
        _api_stop_video_recording()
        return {"status": "stopped"}
    return {"status": "not recording or no node"}

# 获取录制状态API
@app.get("/record_status")
def get_record_status():
    if recorder_node is not None:
        return {
            "is_recording": recorder_node.recording,
            "topics": recorder_node._selected_topics if recorder_node._selected_topics else [],
            "triggered_by_service": getattr(recorder_node, '_triggered_by_service', False)
        }
    return {
        "is_recording": False,
        "topics": [],
        "triggered_by_service": False
    }

# 获取存储状态API
@app.get("/storage_status")
def get_storage_status():
    global storage_error_message
    if storage_error_message:
        return {
            "has_error": True,
            "error_message": storage_error_message
        }
    return {
        "has_error": False,
        "error_message": None
    }

# 清除存储错误状态API
@app.post("/clear_storage_error")
def clear_storage_error():
    global storage_error_message
    storage_error_message = None
    return {"status": "cleared"}



class SimpleBagRecorder(Node):
    def __init__(self):
        super().__init__('simple_bag_recorder')
        self.recording = False
        self.writer = None
        self._topic_subscriptions = []  # 修改属性名称，避免与Node类内部属性冲突
        self._selected_topics = []  # 存储用户选择的话题
        self._triggered_by_service = False  # 标记是否由ROS服务触发录制
        
        # 声明参数，设置默认的存储路径（指向当前项目下BAG_STORAGE/bags）
        default_storage = '/home/marvin/KernelMind_Apex/BAG_STORAGE/bags'
        self.declare_parameter('record_bags_storage_dir', default_storage)
        self.get_logger().info(f'录制bags存储路径参数已声明: {self.get_parameter("record_bags_storage_dir").value}')
        
        # 创建trigger服务来控制录制
        self.recording_service = self.create_service(
            Trigger,
            'toggle_recording',
            self.toggle_recording_callback
        )
        self.get_logger().info('Trigger服务已创建: /toggle_recording')
        
        # 创建一个发布者，发布录制状态
        self.status_publisher = self.create_publisher(Int32, '/recorder/status', 10)
        # 创建一个定时器，以1Hz频率发布状态
        self.status_timer = self.create_timer(1.0, self.publish_status)
        self.get_logger().info('已创建状态发布者: /recorder/status (1Hz)')
    
    def toggle_recording_callback(self, request, response):
        """
        Trigger服务回调函数
        如果当前没有录制，开始录制除/info/和/control/之外的所有话题
        如果当前正在录制，停止录制
        """
        try:
            if not self.recording:
                # 获取所有话题，并过滤掉不需要的话题
                all_topics = self.get_available_topics()  # 这里已经应用了过滤规则
                filtered_topics = [topic["name"] for topic in all_topics]  # 提取话题名称
                
                # 开始录制过滤后的话题
                self.start_recording(filtered_topics)
                # 同步视频录制
                _api_start_video_recording()
                self._triggered_by_service = True  # 标记为服务触发
                response.success = True
                response.message = f"录制已开始，正在录制{len(filtered_topics)}个话题（已排除/info/和/control/话题）"
                self.get_logger().info(f"通过服务触发：开始录制{len(filtered_topics)}个话题（已排除特定话题）")
            else:
                # 停止录制
                self.stop_recording()
                _api_stop_video_recording()
                response.success = True
                response.message = "录制已停止"
                self.get_logger().info("通过服务触发：停止录制")
        except Exception as e:
            response.success = False
            response.message = f"操作失败: {str(e)}"
            self.get_logger().error(f"服务回调失败: {str(e)}")
        
        return response
    
    def get_available_topics(self):
        """获取所有可用话题及其类型"""
        topic_list = []
        topic_names_and_types = self.get_topic_names_and_types()
        for topic_name, type_list in topic_names_and_types:
            # 过滤掉隐藏话题
            if topic_or_service_is_hidden(topic_name):
                continue
            
            # # 过滤掉包含/info/和/control/的话题
            # if '/info/' in topic_name or '/control/' in topic_name:
            #     self.get_logger().debug(f'过滤掉话题: {topic_name}')
            #     continue

            if not self._is_topic_allowed(topic_name):
                self.get_logger().debug(f'过滤掉非白名单话题: {topic_name}')
                continue
                
            # 添加符合条件的话题
            topic_list.append({
                "name": topic_name,
                "type": type_list[0]
            })
        return topic_list

    def _is_topic_allowed(self, topic_name: str) -> bool:
        """仅允许白名单中的话题被录制"""
        return topic_name in ALLOWED_TOPICS
    
    def _import_msg_type(self, msg_type_str):
        """
        动态导入消息类型
        例如: 'std_msgs/msg/String' -> std_msgs.msg.String类
        """
        try:
            parts = msg_type_str.split('/')
            module_name = '.'.join(parts[:-1])
            class_name = parts[-1]
            module = importlib.import_module(module_name)
            return getattr(module, class_name)
        except (ImportError, AttributeError) as e:
            self.get_logger().error(f'导入消息类型失败 {msg_type_str}: {str(e)}')
            return None

    def _create_topic_callback(self, topic_name):
        """
        为每个话题创建专属回调函数
        """
        def callback(msg):
            if self.recording and self.writer is not None:
                try:
                    self.writer.write(
                        topic_name,
                        serialize_message(msg),
                        self.get_clock().now().nanoseconds)
                except Exception as e:
                    self.get_logger().error(f'写入话题 {topic_name} 失败: {str(e)}')
        return callback

    def start_recording(self, selected_topics=None):
        """开始录制指定话题，如果未指定则录制所有话题"""
        # 清理旧的订阅者
        self.destroy_subscriptions()
        
        # 保存选中的话题
        if selected_topics:
            allowed_selection = [t for t in selected_topics if self._is_topic_allowed(t)]
            dropped_topics = set(selected_topics) - set(allowed_selection)
            if dropped_topics:
                self.get_logger().info(f'忽略非白名单话题: {dropped_topics}')
            self._selected_topics = allowed_selection
        else:
            self._selected_topics = None
        
        # 创建新的bag文件
        now = datetime.now()
        bag_name = now.strftime("my_bag-%y-%m-%d-%H-%M-%S")
        # 从参数服务器获取存储路径
        storage_dir = self.get_parameter('record_bags_storage_dir').value
        
        # 检查存储路径的父目录是否存在，不存在则尝试创建
        parent_dir = os.path.dirname(storage_dir)
        if not os.path.exists(parent_dir):
            try:
                os.makedirs(parent_dir, exist_ok=True)
                self.get_logger().info(f'已创建存储父目录: {parent_dir}')
            except Exception:
                global storage_error_message
                error_msg = '存储硬盘不存在，请插入重试'
                storage_error_message = error_msg
                self.get_logger().error(f'存储路径的父目录不存在: {parent_dir}')
                raise FileNotFoundError(error_msg)
        
        self.writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py.StorageOptions(
            uri=(storage_dir + '/'+ bag_name),
            storage_id='mcap')
        converter_options = rosbag2_py.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)
        
        # 获取所有当前存在的话题
        topic_count = 0
        topic_names_and_types = self.get_topic_names_and_types()
        for topic_name, type_list in topic_names_and_types:
            # 跳过隐藏话题
            if topic_or_service_is_hidden(topic_name):
                continue

            # 仅录制白名单中的话题
            if not self._is_topic_allowed(topic_name):
                continue
            
            # 如果指定了话题列表，只处理列表中的话题
            if self._selected_topics and topic_name not in self._selected_topics:
                continue
                
            # 获取话题类型
            topic_type = type_list[0]
            
            # 创建话题元数据
            topic_info = rosbag2_py.TopicMetadata(
                name=topic_name,
                type=topic_type,
                serialization_format='cdr')
            
            # 在bag中注册话题
            try:
                self.writer.create_topic(topic_info)
                topic_count += 1
                
                # 动态导入消息类型
                msg_class = self._import_msg_type(topic_type)
                if msg_class is not None:
                    # 创建订阅者和对应回调
                    sub = self.create_subscription(
                        msg_class,
                        topic_name,
                        self._create_topic_callback(topic_name),
                        10)
                    self._topic_subscriptions.append(sub)  # 使用新的属性名
                    self.get_logger().info(f'已订阅话题: {topic_name} ({topic_type})')
            except Exception as e:
                self.get_logger().error(f'注册话题 {topic_name} 失败: {str(e)}')
        
        self.recording = True
        if topic_count == 0:
            self.get_logger().warn(f"未找到白名单话题，bag为空: {bag_name}")
        else:
            self.get_logger().info(f"开始录制: {bag_name} (已注册 {topic_count} 个话题)")

    def stop_recording(self):
        self.recording = False
        self._triggered_by_service = False  # 重置服务触发标志
        if self.writer is not None:
            # 清理writer资源
            del self.writer
            self.writer = None
        # 可选：停止录制时销毁订阅者
        self.destroy_subscriptions()
        self.get_logger().info("停止录制.")

    def destroy_subscriptions(self):
        """销毁所有已创建的订阅者"""
        for sub in self._topic_subscriptions:
            self.destroy_subscription(sub)
        self._topic_subscriptions.clear()
        self.get_logger().info("已销毁所有订阅者")
        
    def publish_status(self):
        """发布录制状态，1表示正在录制，0表示停止状态"""
        msg = Int32()
        msg.data = 1 if self.recording else 0
        self.status_publisher.publish(msg)
        # 只有状态变化时才记录日志，避免过多输出
        # self.get_logger().debug(f"发布录制状态: {'录制中' if self.recording else '已停止'}")


# ---------------- 在同一 FastAPI 中集成视频服务 ----------------
class VideoStreamer:
    """后台线程运行视频采集/合成与WebRTC推流；通过共享的 video_state 控制MP4录制。"""

    def __init__(self):
        if not all([videoSource, videoOutput, cudaImage, cudaOverlay, cudaResize, cudaFont]):
            raise RuntimeError("jetson_utils 未可用，无法启动视频服务")

        self.input_stream_L = videoSource("csi://1", options={"width": 1920, "height": 1488})
        self.input_stream_R = videoSource("csi://0", options={"width": 1920, "height": 1488})
        self.input_stream_Lh = videoSource("csi://8", options={"width": 1920, "height": 1488})
        self.input_stream_Rh = videoSource("csi://9", options={"width": 1920, "height": 1488})

        self.output_stream = videoOutput(
            "webrtc://@:8554/my_output",
            argv=sys.argv, options={"width": 3840, "height": 1488, "codec": "h264", "bitrate": 8000000},
        )
        self.output_stream_mp4: Optional[object] = None

        self.font = cudaFont()
        self.img_quad = None
        self.img_stereo_resized = None
        self.img_quad_resized = None
        self.t0_mono_ns = None
        self.first_frame_written = False
        self.first_frame_yaml = "first_frame_time.yaml"

        self._thread = threading.Thread(target=self._run_loop, daemon=True)

    def start(self):
        os.makedirs(MP4_DIR, exist_ok=True)
        if not self._thread.is_alive():
            self._thread.start()

    def _run_loop(self):
        num_frames = 0
        while True:
            imgl = self.input_stream_L.Capture()
            imgr = self.input_stream_R.Capture()
            imglh = self.input_stream_Lh.Capture()
            imgrh = self.input_stream_Rh.Capture()

            if imgl is None or imgr is None or imglh is None or imgrh is None:
                continue

            img_stereo = cudaImage(width=imgl.width + imgr.width, height=imgl.height, format=imgl.format)
            img_Lh_sml = cudaImage(width=imgl.width // 4, height=imgl.height // 4, format=imgl.format)
            img_Rh_sml = cudaImage(width=imgr.width // 4, height=imgr.height // 4, format=imgr.format)
            cudaResize(imglh, img_Lh_sml)
            cudaResize(imgrh, img_Rh_sml)
            cudaOverlay(imgl, img_stereo, 0, 0)
            cudaOverlay(imgr, img_stereo, imgl.width, 0)
            cudaOverlay(img_Lh_sml, img_stereo, 10, imgl.height - img_Lh_sml.height - 10)
            cudaOverlay(img_Rh_sml, img_stereo, imgl.width - img_Rh_sml.width - 10, imgl.height - img_Rh_sml.height - 10)
            cudaOverlay(img_Lh_sml, img_stereo, imgl.width + 10, imgl.height - img_Lh_sml.height - 10)
            cudaOverlay(img_Rh_sml, img_stereo, imgl.width * 2 - img_Rh_sml.width - 10, imgl.height - img_Rh_sml.height - 10)

            if self.img_quad is None:
                self.img_quad = cudaImage(width=imgl.width * 2, height=imgl.height * 2, format=imgl.format)
            cudaOverlay(imgl, self.img_quad, 0, 0)
            cudaOverlay(imgr, self.img_quad, imgl.width, 0)
            cudaOverlay(imglh, self.img_quad, 0, imgl.height)
            cudaOverlay(imgrh, self.img_quad, imgl.width, imgl.height)

            if self.img_stereo_resized is None:
                self.img_stereo_resized = cudaImage(width=3840, height=1488, format=imgl.format)
            if self.img_quad_resized is None:
                self.img_quad_resized = cudaImage(width=2560, height=1984, format=imgl.format)
            cudaResize(img_stereo, self.img_stereo_resized)
            cudaResize(self.img_quad, self.img_quad_resized)

            if self.t0_mono_ns is None:
                self.t0_mono_ns = time.monotonic_ns()
            now_ns = time.time_ns()
            mono_ns = time.monotonic_ns() - self.t0_mono_ns
            now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")


            text = f"{now_str} | epoch_ns={now_ns} | mono_ms={mono_ns/1e6:.3f}"
            self.font.OverlayText(
                self.img_stereo_resized,
                self.img_stereo_resized.width,
                self.img_stereo_resized.height,
                text,
                10,
                10,
                self.font.White,
                self.font.Gray40,
            )
            self.font.OverlayText(
                self.img_quad_resized,
                self.img_quad_resized.width,
                self.img_quad_resized.height,
                text,
                10,
                10,
                self.font.White,
                self.font.Gray40,
            )

            with video_lock:
                if video_state["start_requested"]:
                    if self.output_stream_mp4 is not None:
                        try:
                            if hasattr(self.output_stream_mp4, "Close"):
                                self.output_stream_mp4.Close()
                        except Exception:
                            pass
                    filename = os.path.join(MP4_DIR, datetime.now().strftime("quad-%Y%m%d-%H%M%S.mp4"))
                    self.output_stream_mp4 = videoOutput(filename, argv=sys.argv, options={"codec": "h264", "bitrate": 16000000})
                    video_state.update({
                        "start_requested": False,
                        "active": True,
                        "filename": filename,
                    })

                if video_state["stop_requested"]:
                    if self.output_stream_mp4 is not None:
                        try:
                            if hasattr(self.output_stream_mp4, "Close"):
                                self.output_stream_mp4.Close()
                        except Exception:
                            pass
                        self.output_stream_mp4 = None
                    video_state.update({
                        "stop_requested": False,
                        "active": False,
                        "filename": None,
                    })

            if video_state["active"] and self.output_stream_mp4 is not None:
                self.output_stream_mp4.Render(self.img_quad_resized)
                if not self.first_frame_written:
                    with open(self.first_frame_yaml, "w", encoding="utf-8") as f:
                        f.write("first_frame_time:\n")
                        f.write(f"  epoch_ns: {now_ns}\n")
                        f.write(f"  datetime: '{now_str}'\n")
                        f.write(f"  mono_ns: {self.t0_mono_ns}\n")
                    self.first_frame_written = True

            self.output_stream.Render(self.img_stereo_resized)
            self.output_stream.SetStatus(
                "Video Viewer | {:d}x{:d} | {:.1f} FPS".format(
                    imgl.width, imgl.height, self.output_stream.GetFrameRate()
                )
            )

            if num_frames < 15 and Log:
                Log.Verbose(f"video-viewer: captured {num_frames} frames ({imgl.width} x {imgl.height})")
            num_frames += 1

            if (
                not self.input_stream_L.IsStreaming()
                and not self.input_stream_R.IsStreaming()
                and not self.input_stream_Lh.IsStreaming()
                and not self.input_stream_Rh.IsStreaming()
            ):
                break


def _api_start_video_recording():
    if video_streamer is None:
        return {"status": "video_service_unavailable"}
    with video_lock:
        if video_state["active"] or video_state["start_requested"]:
            return {"status": "already_recording", "filename": video_state["filename"]}
        os.makedirs(MP4_DIR, exist_ok=True)
        video_state.update({"start_requested": True, "stop_requested": False})
        return {"status": "start_queued"}


def _api_stop_video_recording():
    if video_streamer is None:
        return {"status": "video_service_unavailable"}
    with video_lock:
        if not video_state["active"] and not video_state["start_requested"]:
            return {"status": "not_recording"}
        video_state.update({"stop_requested": True, "start_requested": False})
        return {"status": "stop_queued"}


@app.post("/video/start")
def api_start_video():
    return _api_start_video_recording()


@app.post("/video/stop")
def api_stop_video():
    return _api_stop_video_recording()


@app.get("/video/status")
def api_video_status():
    with video_lock:
        return {
            "active": video_state["active"],
            "filename": video_state["filename"],
            "start_requested": video_state["start_requested"],
            "stop_requested": video_state["stop_requested"],
        }


def ros2_thread():
    global recorder_node
    rclpy.init()
    recorder_node = SimpleBagRecorder()
    rclpy.spin(recorder_node)
    rclpy.shutdown()

def main(args=None):
    # 启动ROS2节点线程
    t = threading.Thread(target=ros2_thread, daemon=True)
    t.start()
    # 启动视频采集/推流线程（与同一FastAPI共享API）
    global video_streamer
    try:
        video_streamer = VideoStreamer()
        video_streamer.start()
    except Exception as e:
        # 若视频库不可用，不阻塞FastAPI与ROS服务
        print(f"[WARN] 视频服务未启动: {e}")
    # 启动FastAPI服务
    uvicorn.run(app, host="0.0.0.0", port=8000)


if __name__ == '__main__':
    main()