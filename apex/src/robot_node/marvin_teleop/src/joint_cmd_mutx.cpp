// joint_cmd_mutx.cpp
// Multiplex joint commands from teleop and planner sources per arm.

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "marvin_msgs/msg/jointcmd.hpp"
#include "std_msgs/msg/int32.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class JointCmdMuxNode : public rclcpp::Node
{
public:
	JointCmdMuxNode()
	: rclcpp::Node("joint_cmd_mux"),
	  timeout_sec_(this->declare_parameter<double>("input_timeout_sec", 0.2))
	{
		rclcpp::SensorDataQoS sensor_data_qos;

		joint_cmd_pub_A_ = this->create_publisher<marvin_msgs::msg::Jointcmd>(
			"control/joint_cmd_A", sensor_data_qos);
		joint_cmd_pub_B_ = this->create_publisher<marvin_msgs::msg::Jointcmd>(
			"control/joint_cmd_B", sensor_data_qos);

		sub_tele_A_ = this->create_subscription<marvin_msgs::msg::Jointcmd>(
			"control/joint_cmd_tele_A", sensor_data_qos,
			std::bind(&JointCmdMuxNode::teleop_callback_A, this, _1));
		sub_tele_B_ = this->create_subscription<marvin_msgs::msg::Jointcmd>(
			"control/joint_cmd_tele_B", sensor_data_qos,
			std::bind(&JointCmdMuxNode::teleop_callback_B, this, _1));
		sub_plan_A_ = this->create_subscription<marvin_msgs::msg::Jointcmd>(
			"control/joint_cmd_plan_A", sensor_data_qos,
			std::bind(&JointCmdMuxNode::plan_callback_A, this, _1));
		sub_plan_B_ = this->create_subscription<marvin_msgs::msg::Jointcmd>(
			"control/joint_cmd_plan_B", sensor_data_qos,
			std::bind(&JointCmdMuxNode::plan_callback_B, this, _1));
		system_state_pub_ = this->create_publisher<std_msgs::msg::Int32>("control/switch_state", 10);

		watchdog_timer_ = this->create_wall_timer(
			50ms, std::bind(&JointCmdMuxNode::watchdog_check, this));
	}

private:
	enum class Source
	{
		None,
		Teleop,
		Plan
	};

	struct ArmState
	{
		Source active = Source::None;
		rclcpp::Time last_msg = rclcpp::Time(0, 0, RCL_ROS_TIME);
	};

	void teleop_callback_A(const marvin_msgs::msg::Jointcmd::SharedPtr msg)
	{
		handle_msg('A', Source::Teleop, msg);
	}

	void teleop_callback_B(const marvin_msgs::msg::Jointcmd::SharedPtr msg)
	{
		handle_msg('B', Source::Teleop, msg);
	}

	void plan_callback_A(const marvin_msgs::msg::Jointcmd::SharedPtr msg)
	{
		handle_msg('A', Source::Plan, msg);
	}

	void plan_callback_B(const marvin_msgs::msg::Jointcmd::SharedPtr msg)
	{
		handle_msg('B', Source::Plan, msg);
	}

	void handle_msg(char arm, Source source, const marvin_msgs::msg::Jointcmd::SharedPtr &msg)
	{
		auto now = this->now();
		ArmState &state = (arm == 'A') ? arm_A_ : arm_B_;

		if (state.active == Source::None)
		{
			state.active = source;
			RCLCPP_INFO(this->get_logger(), "Arm %c active source set to %s", arm,
						source == Source::Teleop ? "teleop" : "plan");
			publish_state_change(arm, state.active);
		}

		if (state.active != source)
		{
			return; // Ignore non-active source
		}

		state.last_msg = now;

		auto out = *msg;
		out.header.stamp = now;

		if (arm == 'A')
		{
			joint_cmd_pub_A_->publish(out);
		}
		else
		{
			joint_cmd_pub_B_->publish(out);
		}
	}

	void watchdog_check()
	{
		auto now = this->now();
		check_arm_timeout('A', arm_A_, now);
		check_arm_timeout('B', arm_B_, now);
	}

	void check_arm_timeout(char arm, ArmState &state, const rclcpp::Time &now)
	{
		if (state.active == Source::None)
		{
			return;
		}

		double dt = (now - state.last_msg).seconds();
		if (dt > timeout_sec_)
		{
			state.active = Source::None;
			RCLCPP_WARN(this->get_logger(), "Arm %c idle (no msg %.3fs)", arm, dt);
			publish_state_change(arm, state.active);
		}
	}

	void publish_state_change(char arm, Source source)
	{
		std_msgs::msg::Int32 msg;
		msg.data = source_to_mode(source);
		system_state_pub_->publish(msg);
		RCLCPP_INFO(this->get_logger(), "Published switch_state=%d (arm %c)", msg.data, arm);
	}

	int source_to_mode(Source source) const
	{
		switch (source)
		{
		case Source::Teleop:
			return 0;
		case Source::Plan:
			return 2;
		case Source::None:
		default:
			return 0;
		}
	}

	rclcpp::Publisher<marvin_msgs::msg::Jointcmd>::SharedPtr joint_cmd_pub_A_;
	rclcpp::Publisher<marvin_msgs::msg::Jointcmd>::SharedPtr joint_cmd_pub_B_;
	rclcpp::Subscription<marvin_msgs::msg::Jointcmd>::SharedPtr sub_tele_A_;
	rclcpp::Subscription<marvin_msgs::msg::Jointcmd>::SharedPtr sub_tele_B_;
	rclcpp::Subscription<marvin_msgs::msg::Jointcmd>::SharedPtr sub_plan_A_;
	rclcpp::Subscription<marvin_msgs::msg::Jointcmd>::SharedPtr sub_plan_B_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr system_state_pub_;

	rclcpp::TimerBase::SharedPtr watchdog_timer_;

	ArmState arm_A_;
	ArmState arm_B_;
	double timeout_sec_;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<JointCmdMuxNode>());
	rclcpp::shutdown();
	return 0;
}
