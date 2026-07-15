#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from marvin_msgs.srv import Int
import random
import time

class RobotSimulatorNode(Node):
    """
    模拟机器人节点，提供三个服务：
    1. /set_ready - 响应启动机器人的请求
    2. /set_mode - 响应设置机器人模式的请求
    3. /set_vel_ratio - 响应设置速度因子的请求
    """

    def __init__(self):
        super().__init__('robot_simulator_node')
        
        # 创建服务器
        self.start_service = self.create_service(
            Trigger, 
            '/set_ready', 
            self.handle_start_robot
        )
        
        self.mode_service = self.create_service(
            Int,
            '/set_mode',
            self.handle_set_mode
        )
        
        self.speed_factor_service = self.create_service(
            Int,
            '/set_vel_ratio',
            self.handle_set_vel_ratio
        )
        
        # 机器人状态
        self.is_started = False
        self.current_mode = -1  # 未初始化状态
        self.current_speed_factor = 100  # 默认速度因子为1.0（存储为整数：1.0 * 100）
        
        self.get_logger().info('机器人模拟节点已启动')
        self.get_logger().info('提供服务：/set_ready（启动机器人）')
        self.get_logger().info('提供服务：/set_mode（设置机器人模式）')
        self.get_logger().info('提供服务：/set_vel_ratio（设置速度因子）')
    
    def handle_start_robot(self, request, response):
        """处理启动机器人请求"""
        self.get_logger().info('收到启动机器人请求')
        
        # 模拟处理延迟
        delay = random.uniform(0.5, 2.0)
        self.get_logger().info(f'模拟处理中 (延迟 {delay:.1f}秒)...')
        time.sleep(delay)
        
        # 随机成功/失败（90%成功率）
        success = random.random() < 0.9
        
        if success:
            self.is_started = True
            response.success = True
            response.message = "机器人启动成功！系统初始化完成。"
            self.get_logger().info('机器人启动成功')
        else:
            response.success = False
            response.message = "机器人启动失败：" + random.choice([
                "电机初始化错误",
                "安全检查未通过",
                "通信总线故障",
                "电源电压不足"
            ])
            self.get_logger().error(f'机器人启动失败: {response.message}')
        
        return response
    
    def handle_set_mode(self, request, response):
        """处理设置机器人模式请求"""
        mode = request.data
        self.get_logger().info(f'收到设置机器人模式请求: 模式{mode + 1}')
        
        # 检查机器人是否已启动
        if not self.is_started:
            self.get_logger().error('机器人未启动，无法设置模式')
            response.success = False
            response.error_code = 1
            response.message = "机器人未启动，请先启动机器人"
            return response
            
        # 模拟处理延迟
        delay = random.uniform(0.3, 1.5)
        self.get_logger().info(f'模拟处理中 (延迟 {delay:.1f}秒)...')
        time.sleep(delay)
        
        # 检查模式是否有效
        if mode not in [0, 1, 2, 3]:
            self.get_logger().error(f'无效的模式: {mode}')
            response.success = False
            response.error_code = 2
            response.message = f"无效的模式: {mode}"
            return response
        
        # 随机成功/失败（95%成功率）
        success = random.random() < 0.95
        
        if success:
            self.current_mode = mode
            response.success = True
            response.error_code = 0
            response.message = f"已切换到模式{mode + 1}"
            self.get_logger().info(f'已切换到模式{mode + 1}')
        else:
            response.success = False
            response.error_code = 3
            response.message = "模式切换失败：" + random.choice([
                "执行器初始化错误",
                "模式参数无效",
                "内部状态冲突",
                "当前任务无法中断"
            ])
            self.get_logger().error(f'模式切换失败: {response.message}')
        
        return response
        
    def handle_set_vel_ratio(self, request, response):
        """处理设置速度因子请求"""
        speed_factor_int = request.data
        speed_factor = speed_factor_int / 100.0  # 转换回浮点数
        self.get_logger().info(f'收到设置速度因子请求: {speed_factor:.2f}x')
        
        # 检查机器人是否已启动
        if not self.is_started:
            self.get_logger().error('机器人未启动，无法设置速度因子')
            response.success = False
            response.error_code = 1
            return response
            
        # 模拟处理延迟
        delay = random.uniform(0.2, 1.0)
        self.get_logger().info(f'模拟处理中 (延迟 {delay:.1f}秒)...')
        time.sleep(delay)
        
        # 检查速度因子是否有效 (支持0.1x到5.0x，即10到500)
        if speed_factor_int < 0 or speed_factor_int > 500:
            self.get_logger().error(f'无效的速度因子: {speed_factor:.2f}')
            response.success = False
            response.error_code = 2
            response.message = f"无效的速度因子: {speed_factor:.2f}"
            return response
        
        # 随机成功/失败（98%成功率）
        success = random.random() < 0.98
        
        if success:
            self.current_speed_factor = speed_factor_int
            response.success = True
            response.error_code = 0
            response.message = f"速度因子已设置为 {speed_factor:.2f}x"
            self.get_logger().info(f'速度因子已设置为 {speed_factor:.2f}x')
        else:
            response.success = False
            response.error_code = 3
            error_msg = random.choice([
                "速度参数更新失败",
                "控制器拒绝参数",
                "速度限制器异常",
                "系统资源不足"
            ])
            response.message = f"速度因子设置失败: {error_msg}"
            self.get_logger().error(f'速度因子设置失败: {error_msg}')
        
        return response


def main(args=None):
    rclpy.init(args=args)
    robot_simulator = RobotSimulatorNode()
    
    try:
        rclpy.spin(robot_simulator)
    except KeyboardInterrupt:
        pass
    finally:
        robot_simulator.get_logger().info('关闭机器人模拟节点')
        robot_simulator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()