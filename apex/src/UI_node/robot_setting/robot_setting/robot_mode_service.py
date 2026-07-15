#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import threading
from marvin_msgs.srv import Int
from std_srvs.srv import Trigger
from pydantic import BaseModel
from std_msgs.msg import Int16MultiArray

# FastAPI应用
app = FastAPI()
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 请求模型
class ModeRequest(BaseModel):
    mode: int  # 0, 1, 2 对应三种模式

class SpeedFactorRequest(BaseModel):
    speed_factor: float  # 速度因子：1.0, 1.2, 1.5等

# 全局节点引用
mode_node = None

@app.post("/set_mode")
def set_robot_mode(request: ModeRequest):
    if mode_node is None:
        return {"status": "error", "message": "节点未初始化"}
    
    try:
        result = mode_node.set_mode(request.mode)
        return {"status": "success", "result": result}
    except Exception as e:
        return {"status": "error", "message": str(e)}

@app.get("/get_current_mode")
def get_current_mode():
    if mode_node is None:
        return {"status": "error", "message": "节点未初始化"}
    
    return {
        "status": "success", 
        "mode": mode_node.current_mode
    }

@app.get("/get_robot_status")
def get_robot_status():
    if mode_node is None:
        return {"status": "error", "message": "节点未初始化"}
    
    return {
        "status": "success", 
        "robot_started": mode_node.robot_started,
        "robot_error": mode_node.robot_error,
        "last_error_time": mode_node.last_error_time,
        "error_message": mode_node.error_message
    }

@app.post("/set_ready")
def start_robot():
    if mode_node is None:
        return {"status": "error", "message": "节点未初始化"}
    
    try:
        result = mode_node.start_robot()
        return {"status": "success", "result": result}
    except Exception as e:
        return {"status": "error", "message": str(e)}

@app.post("/set_vel_ratio")
def set_vel_ratio(request: SpeedFactorRequest):
    if mode_node is None:
        return {"status": "error", "message": "节点未初始化"}
    
    try:
        result = mode_node.set_vel_ratio(request.speed_factor)
        return {"status": "success", "result": result}
    except Exception as e:
        return {"status": "error", "message": str(e)}

@app.get("/get_speed_factor")
def get_speed_factor():
    if mode_node is None:
        return {"status": "error", "message": "节点未初始化"}
    
    return {
        "status": "success", 
        "speed_factor": mode_node.current_speed_factor
    }

@app.post("/api/robot/reset")
def home_arm():
    if mode_node is None:
        return {"status": "error", "message": "节点未初始化"}
    
    try:
        result = mode_node.home_robot()
        return {"status": "success", "result": result}
    except Exception as e:
        return {"status": "error", "message": str(e)}
    

@app.post("/api/gripper/restart")
def reset_grippers():
    if mode_node is None:
        return {"status": "error", "message": "节点未初始化"}
    
    try:
        result = mode_node.reset_grippers()
        return {"status": "success", "result": result}
    except Exception as e:
        return {"status": "error", "message": str(e)}

@app.post("/api/hand/home")
def home_hand():
    if mode_node is None:
        return {"status": "error", "message": "节点未初始化"}
    
    try:
        result = mode_node.home_hand()
        return {"status": "success", "result": result}
    except Exception as e:
        return {"status": "error", "message": str(e)}
    


# ROS2节点实现
class RobotModeService(Node):
    def __init__(self):
        super().__init__('robot_mode_service')
        
        # 创建服务客户端
        self.mode_client = self.create_client(Int, '/control/set_mode')
        # 创建启动机器人服务客户端
        self.start_robot_client = self.create_client(Trigger, '/control/set_ready')
        # 创建速度控制服务客户端
        self.speed_client = self.create_client(Int, '/control/set_vel_ratio')
        self.home_arm_client = self.create_client(Trigger, '/control/home_arm')
        self.reset_grippers_client = self.create_client(Trigger, '/control/reset_grippers')
        self.home_hand_left_client = self.create_client(Trigger, '/hand_left/home_hand')
        self.home_hand_right_client = self.create_client(Trigger, '/hand_right/home_hand')
        self.request_mode_service = self.create_service(
            Int,
            '/control/request_mode',
            self.request_mode_callback,
        )
        self.home_hand_service = self.create_service(
            Trigger,
            '/control/home_hand',
            self.home_hand_callback,
        )
        # 状态变量
        self.current_mode = -1  # 未初始化状态
        self.robot_started = False  # 记录机器人启动状态
        self.current_speed_factor = 1.0  # 默认速度因子为1.0
        
        # 错误状态跟踪
        self.robot_error = False  # 机器人错误状态
        self.last_error_time = None  # 最后一次错误时间
        self.error_message = ""  # 错误消息
        
        # 模式切换暂态保护（应对两臂切换不同步导致 mode1 != mode2）
        self._mode_mismatch_count = 0      # 连续不一致计数
        self._mode_mismatch_threshold = 5  # 连续 N 次不一致才触发错误（约 0.5s @ 10Hz）
        self._mode_switch_in_progress = False   # 模式切换进行中标志
        self._mode_switch_start_time = None     # 模式切换开始时间
        self._mode_switch_grace_period = 3.0    # 切换宽限期（秒）
        
        # 订阅 /info/arm_state 话题
        self.arm_state_subscriber = self.create_subscription(
            Int16MultiArray,
            '/info/arm_state',
            self.arm_state_callback,
            10
        )
        
        self.get_logger().info('机器人模式服务节点已启动')
        self.get_logger().info('订阅话题: /info/arm_state (机器人状态监听)')
        self.get_logger().info('提供服务: /control/request_mode -> /control/set_mode')
        self.get_logger().info('提供服务: /control/home_hand -> /hand_left/right/home_hand')
        self.get_logger().info('机器人模式状态将通过话题回调自动更新')
    
    def arm_state_callback(self, msg):
        """处理 /info/arm_state 话题的回调函数
        
        注意：两臂硬件切换可能不同步，导致短暂 mode1 != mode2。
        本回调使用以下策略容忍暂态不一致：
        1. 如果处于模式切换宽限期内 → 暂时容忍，累计连续不一致计数
        2. 仅当连续不一致超过阈值时才触发错误
        3. 一旦看到一致值，立即重置计数
        """
        import datetime  # keep import here for defensive isolation
        
        if len(msg.data) >= 2:
            mode1, mode2 = msg.data[:2]
            
            # ── 情况0: 模式切换宽限期内，临时容忍暂态 ──
            if self._mode_switch_in_progress:
                grace_remaining = (datetime.datetime.now() - self._mode_switch_start_time).total_seconds()
                if grace_remaining < self._mode_switch_grace_period:
                    if mode1 != mode2 or (mode1 == -1 and mode2 == -1):
                        # 仍在切换窗口内，静默容忍（不触发错误）
                        self.get_logger().debug(
                            f'切换宽限期内暂态 mode1={mode1}, mode2={mode2}, '
                            f'剩余 {self._mode_switch_grace_period - grace_remaining:.1f}s'
                        )
                        return
            
            # ── 情况1: (-1, -1) 表示未启动/未初始化状态 ──
            if mode1 == -1 and mode2 == -1:
                self.robot_started = False
                self.current_mode = -1
                self.robot_error = False
                self._mode_mismatch_count = 0
                return

            # ── 情况5: mode1 != mode2 —— 暂态不一致 ──
            if mode1 != mode2:
                self._mode_mismatch_count += 1
                if self._mode_mismatch_count < self._mode_mismatch_threshold:
                    # 还没达到阈值，只是警告
                    self.get_logger().warning(
                        f'模式暂态不一致 ({self._mode_mismatch_count}/{self._mode_mismatch_threshold}): '
                        f'mode1={mode1}, mode2={mode2}'
                    )
                    return
                
                # 连续不一致超过阈值 → 报错
                self.get_logger().error(
                    f'机器人状态错误: 连续 {self._mode_mismatch_count} 次不一致 '
                    f'(mode1={mode1}, mode2={mode2})'
                )
                self.robot_started = False
                self.current_mode = -1
                self.robot_error = True
                self.last_error_time = datetime.datetime.now().isoformat()
                self.error_message = f"内部状态不一致 (连续{self._mode_mismatch_count}次): mode1={mode1}, mode2={mode2}"
                self.get_logger().error('机器人状态已重置，需要手动重启')
                return

            # ── 情况3-4: 两臂一致 —— 正常状态 ──
            # 进入此分支时 mode1 == mode2
            self._mode_mismatch_count = 0  # 重置不一致计数
            
            if mode1 in [0, 1, 2, 3]:
                # 更新机器人模式
                self.robot_error = False
                old_mode = self.current_mode
                self.current_mode = mode1
                if old_mode != self.current_mode:
                    self.get_logger().info(f'机器人模式从 {old_mode} 更新为: {self.current_mode}')
                
                # 如果模式有效且不是未初始化状态，则认为机器人已启动
                if not self.robot_started:
                    self.robot_started = True
            else:
                # 相等但不在有效列表内的值，视为无效
                self.get_logger().warning(f'接收到无效的模式数据: mode1={mode1}, mode2={mode2}')
        else:
            self.get_logger().warning(f'接收到的模式数据长度不足: {len(msg.data)}，需要至少2个数据')

    def set_mode(self, mode):
        """调用Int服务设置机器人模式"""
        self.get_logger().info(f'设置机器人模式: {mode}')
        
        # 检查服务是否可用
        if not self.mode_client.wait_for_service(timeout_sec=1.0):
            error_msg = '服务不可用: /set_mode'
            self.get_logger().error(error_msg)
            return {"success": False, "error": error_msg}
        
        # 激活模式切换宽限期 —— 在切换期间 arm_state_callback 会容忍 mode1 != mode2
        import datetime
        self._mode_switch_in_progress = True
        self._mode_switch_start_time = datetime.datetime.now()
        self._mode_mismatch_count = 0  # 重置不一致计数
        
        try:
            # 创建请求
            request = Int.Request()
            request.data = mode
            
            # 发送请求并等待响应
            future = self.mode_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.done():
                response = future.result()
                if response.success:
                    # 直接更新当前模式状态，确保前端轮询能立即获取到最新值
                    self.current_mode = mode
                    self.get_logger().info(f'模式已更新为: {mode}')
                    return {"success": True, "mode": mode}
                else:
                    return {"success": False, "error": f"服务调用失败, 错误码: {response.error_code}"}
            else:
                return {"success": False, "error": "服务调用超时"}
        except Exception as e:
            error_msg = f"设置机器人模式时发生错误: {str(e)}"
            self.get_logger().error(error_msg)
            return {"success": False, "error": error_msg}
        finally:
            # 服务响应后，宽限期留给回调继续容忍暂态，但不在这里清除 flag
            # 回调会在宽限期到期后自行退出容忍状态
            pass

    def request_mode_callback(self, request, response):
        """ROS service wrapper used by playback nodes to request robot mode."""
        result = self.set_mode(request.data)
        response.success = bool(result.get("success", False))
        if response.success:
            response.error_code = 0
            response.message = f"Mode set to {request.data}"
        else:
            response.error_code = -1
            response.message = result.get("error", "Failed to set mode")
        return response

    def home_hand_callback(self, request, response):
        """ROS service wrapper used to request available hands to home."""
        del request
        clients = [
            ('/hand_left/home_hand', self.home_hand_left_client),
            ('/hand_right/home_hand', self.home_hand_right_client),
        ]
        available = []
        missing = []
        for service_name, client in clients:
            if client.service_is_ready():
                available.append((service_name, client))
            else:
                missing.append(service_name)

        if not available:
            response.success = False
            response.message = '服务不可用: ' + ', '.join(missing)
            return response

        for _, client in available:
            client.call_async(Trigger.Request())

        response.success = True
        sent = [service_name for service_name, _ in available]
        response.message = '回 home 请求已发送: ' + ', '.join(sent)
        if missing:
            response.message += '; 跳过不可用服务: ' + ', '.join(missing)
        return response
            
    def start_robot(self):
        """调用Trigger服务启动机器人"""
        self.get_logger().info('启动机器人')
        
        # 如果机器人已启动，直接返回
        if self.robot_started:
            self.get_logger().info('机器人已经处于启动状态')
            return {"success": True, "message": "机器人已经处于启动状态", "already_started": True}
            
        # 检查服务是否可用
        if not self.start_robot_client.wait_for_service(timeout_sec=1.0):
            error_msg = '服务不可用: /set_ready'
            self.get_logger().error(error_msg)
            return {"success": False, "message": error_msg}
        
        try:
            # 创建请求
            request = Trigger.Request()
            
            # 发送请求并等待响应
            future = self.start_robot_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.done():
                response = future.result()
                if response.success:
                    self.robot_started = True  # 更新启动状态
                    # 清除错误状态
                    self.robot_error = False
                    self.last_error_time = None
                    self.error_message = ""
                    return {"success": True, "message": response.message}
                else:
                    return {"success": False, "message": response.message}
            else:
                return {"success": False, "message": "服务调用超时"}
        except Exception as e:
            error_msg = f"启动机器人时发生错误: {str(e)}"
            self.get_logger().error(error_msg)
            return {"success": False, "message": error_msg}
    
    def set_vel_ratio(self, speed_factor):
        """调用Int服务设置机器人速度因子"""
        self.get_logger().info(f'设置机器人速度因子: {speed_factor}')
        
        # 检查输入是否有效
        if speed_factor < 0:
            error_msg = f'无效的速度因子值: {speed_factor}, 必须大于0'
            self.get_logger().error(error_msg)
            return {"success": False, "error": error_msg}
        
        # 检查服务是否可用
        if not self.speed_client.wait_for_service(timeout_sec=1.0):
            error_msg = '服务不可用: /set_vel_ratio'
            self.get_logger().error(error_msg)
            return {"success": False, "error": error_msg}
        
        try:
            # 创建请求
            request = Int.Request()
            # 转换float为int，因为Int.srv使用int64
            # 将速度因子乘以100保存为整数，保留两位小数精度
            request.data = int(speed_factor)
            self.get_logger().warning(f'request.data ={request.data }')
            
            # 发送请求并等待响应
            future = self.speed_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.done():
                response = future.result()
                if response.success:
                    self.current_speed_factor = speed_factor
                    return {"success": True, "speed_factor": speed_factor}
                else:
                    return {"success": False, "error": f"服务调用失败, 错误码: {response.error_code}"}
            else:
                return {"success": False, "error": "服务调用超时"}
        except Exception as e:
            error_msg = f"设置机器人速度因子时发生错误: {str(e)}"
            self.get_logger().error(error_msg)
            return {"success": False, "error": error_msg}
    
    def home_arm(self):
        """调用Trigger服务回家机器人臂"""
        self.get_logger().info('执行手臂回家命令')
        
        # 检查服务是否可用
        if not self.home_arm_client.wait_for_service(timeout_sec=1.0):
            error_msg = '服务不可用: /control/home_arm'
            self.get_logger().error(error_msg)
            return {"success": False, "message": error_msg}
        
        try:
            # 创建请求
            request = Trigger.Request()
            
            # 发送请求并等待响应
            future = self.home_arm_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.done():
                response = future.result()
                if response.success:
                    self.get_logger().info('手臂成功回家')
                    return {"success": True, "message": response.message}
                else:
                    return {"success": False, "message": response.message}
            else:
                return {"success": False, "message": "服务调用超时"}
        except Exception as e:
            error_msg = f"执行手臂回家命令时发生错误: {str(e)}"
            self.get_logger().error(error_msg)
            return {"success": False, "message": error_msg}

    def home_robot(self):
        """调用机械臂和手部 home 服务"""
        self.get_logger().info('执行机器人回 home 命令')

        arm_future = None
        arm_result = None
        hand_clients = [
            ('/hand_left/home_hand', self.home_hand_left_client),
            ('/hand_right/home_hand', self.home_hand_right_client),
        ]
        available_hands = []
        missing_hands = []

        arm_available = self.home_arm_client.service_is_ready()
        for service_name, client in hand_clients:
            if client.service_is_ready():
                available_hands.append((service_name, client))
            else:
                missing_hands.append(service_name)

        if not arm_available:
            arm_result = {"success": False, "message": "服务不可用: /control/home_arm"}

        hand_futures = {}
        try:
            if arm_available:
                arm_future = self.home_arm_client.call_async(Trigger.Request())
            for service_name, client in available_hands:
                hand_futures[service_name] = client.call_async(Trigger.Request())
        except Exception as e:
            error_msg = f"发送机器人回 home 请求时发生错误: {str(e)}"
            self.get_logger().error(error_msg)
            return {"success": False, "message": error_msg}

        if arm_future is not None:
            rclpy.spin_until_future_complete(self, arm_future, timeout_sec=2.0)
            if arm_future.done():
                arm_response = arm_future.result()
                arm_result = {"success": bool(arm_response.success), "message": arm_response.message}
            else:
                arm_result = {"success": False, "message": "服务调用超时"}

        if hand_futures:
            hand_result = {
                "success": True,
                "message": "手回 home 请求已发送",
                "requests": list(hand_futures.keys()),
                "skipped": missing_hands,
            }
        else:
            hand_result = {
                "success": False,
                "message": "服务不可用: " + ', '.join(missing_hands),
                "skipped": missing_hands,
            }

        success = bool(arm_result.get("success", False)) and bool(hand_result.get("success", False))
        return {
            "success": success,
            "message": "机器人回 home 请求完成" if success else "机器人回 home 部分失败",
            "arm": arm_result,
            "hand": hand_result,
        }

    def reset_grippers(self):
        """调用Trigger服务重置夹爪"""
        self.get_logger().info('执行重置夹爪命令')
        
        # 检查服务是否可用
        if not self.reset_grippers_client.wait_for_service(timeout_sec=1.0):
            error_msg = '服务不可用: /control/reset_grippers'
            self.get_logger().error(error_msg)
            return {"success": False, "message": error_msg}
        
        try:
            request = Trigger.Request()
            future = self.reset_grippers_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.done():
                response = future.result()
                if response.success:
                    self.get_logger().info('夹爪重置成功')
                    return {"success": True, "message": response.message}
                else:
                    return {"success": False, "message": response.message}
            else:
                return {"success": False, "message": "服务调用超时"}
        except Exception as e:
            error_msg = f"执行重置夹爪命令时发生错误: {str(e)}"
            self.get_logger().error(error_msg)
            return {"success": False, "message": error_msg}

    def home_hand(self):
        """调用可用的手部 home 服务"""
        self.get_logger().info('执行手回 home 命令')

        clients = [
            ('/hand_left/home_hand', self.home_hand_left_client),
            ('/hand_right/home_hand', self.home_hand_right_client),
        ]

        available = []
        missing = []
        for service_name, client in clients:
            if client.service_is_ready():
                available.append((service_name, client))
            else:
                missing.append(service_name)

        if not available:
            error_msg = '服务不可用: ' + ', '.join(missing)
            self.get_logger().error(error_msg)
            return {"success": False, "message": error_msg, "skipped": missing}

        futures = {}
        for service_name, client in available:
            try:
                futures[service_name] = client.call_async(Trigger.Request())
            except Exception as e:
                error_msg = f"执行手回 home 命令时发生错误: {service_name}: {str(e)}"
                self.get_logger().error(error_msg)
                return {"success": False, "message": error_msg}

        return {
            "success": True,
            "message": "手回 home 请求已发送",
            "requests": list(futures.keys()),
            "skipped": missing,
        }

# ROS2线程函数
def ros2_thread():
    global mode_node
    rclpy.init()
    mode_node = RobotModeService()
    rclpy.spin(mode_node)
    rclpy.shutdown()

def main():
    # 启动ROS2线程
    t = threading.Thread(target=ros2_thread, daemon=True)
    t.start()
    # 启动FastAPI服务
    # Disable uvicorn access logs to avoid noisy per-request GET INFO lines
    uvicorn.run(app, host="0.0.0.0", port=8001, access_log=False, log_level="debug")

if __name__ == '__main__':
    main()
