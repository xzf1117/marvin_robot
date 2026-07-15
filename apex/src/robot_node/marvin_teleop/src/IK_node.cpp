#include <iostream>
// #include <casadi/casadi.hpp>
#include "math.h"
#include "float.h"
#include "ctime"

#include "stdio.h"
#include <iostream>
#include <string.h>
#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

// #include "fx_sdk/FxRobot.h"
// #include "fx_sdk/FXMath.h"
#include "fx_sdk/FxRobot.h"
// #include "kinematicsSDK/FXMath.h"
// #include "marvin_teleop/fx_sdk/Nsp.h"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Geometry>
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <string>
#include "marvin_msgs/srv/int.hpp"
#include "marvin_msgs/srv/velratio.hpp"
#include "marvin_msgs/msg/jointcmd.hpp"
#include "marvin_msgs/msg/jointfeedback.hpp"
#include "marvin_msgs/msg/ik_request.hpp"
#include "marvin_msgs/msg/ik_result.hpp"
#include "marvin_teleop/oneEuro_filter.h"

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/mjcf.hpp"
#include "pinocchio/parsers/srdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/collision/collision.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "tpm/uid_check.hpp"
// Pilot the_pilot;
// PilotLmt the_lmt;


double barrier_inv_sq(double x, double xmax, double eps = 1e-2)
{
	double d = xmax - x;
	if (d <= 0.0)
		return 1e6;

	return 1.0 / ((d + eps) * (d + eps));
}
double slidingAverage(double x)
{
	constexpr int N = 100;

	static double buf[N] = {0.0};  // 窗口初始化为 0
	static int idx = 0;
	static double sum = 0.0;

	// 去掉最旧值
	sum -= buf[idx];

	// 加入新值
	buf[idx] = x;
	sum += x;

	// 环形推进
	idx = (idx + 1) % N;

	// 始终是 100 长度平均
	return sum / N;
}

geometry_msgs::msg::Pose convertArrayToPose(const double pgA[4][4])
{
	geometry_msgs::msg::Pose pose;

	// Extract position (translation)
	pose.position.x = pgA[0][3] * 0.001;
	pose.position.y = pgA[1][3] * 0.001;
	pose.position.z = pgA[2][3] * 0.001;

	// Extract orientation (convert rotation matrix to quaternion)
	tf2::Matrix3x3 rotation_matrix(
		pgA[0][0], pgA[0][1], pgA[0][2],
		pgA[1][0], pgA[1][1], pgA[1][2],
		pgA[2][0], pgA[2][1], pgA[2][2]);

	tf2::Quaternion quat;
	rotation_matrix.getRotation(quat);

	pose.orientation.x = quat.x();
	pose.orientation.y = quat.y();
	pose.orientation.z = quat.z();
	pose.orientation.w = quat.w();

	return pose;
}

Eigen::Isometry3d convertArrayToIsometry(const double pgA[4][4])
{
	Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
	iso.matrix() << pgA[0][0], pgA[0][1], pgA[0][2], pgA[0][3]*0.001,
		pgA[1][0], pgA[1][1], pgA[1][2], pgA[1][3]*0.001,
		pgA[2][0], pgA[2][1], pgA[2][2], pgA[2][3]*0.001,
		0.0, 0.0, 0.0, 1.0;
	return iso;
}

void convertIsometryToArray(const Eigen::Isometry3d &iso, double pgA[4][4])
{
	const Eigen::Matrix4d mat = iso.matrix();
	for (int i = 0; i < 4; ++i)
	{
		for (int j = 0; j < 4; ++j)
		{
			pgA[i][j] = mat(i, j);
		}
	}
	pgA[0][3] *= 1000.0; // Convert back to mm
	pgA[1][3] *= 1000.0;
	pgA[2][3] *= 1000.0;
}



geometry_msgs::msg::PoseStamped transform_pose(const geometry_msgs::msg::PoseStamped &input_pose,
											   const geometry_msgs::msg::TransformStamped &transform)
{
	geometry_msgs::msg::PoseStamped output_pose;

	// Transform the pose
	tf2::doTransform(input_pose, output_pose, transform);

	return output_pose;
}

void extractJointPositions(
	const sensor_msgs::msg::JointState &joint_state,
	const std::vector<std::string> &desired_joint_names,
	double (&joint_positions)[7])
{
	// Create a map from joint names to their positions
	std::map<std::string, double> name_to_position;
	for (size_t i = 0; i < 7; ++i)
	{
		name_to_position[joint_state.name[i]] = joint_state.position[i];
	}

	// Extract positions in the order of desired_joint_names

	for (size_t i = 0; i < 7; ++i)
	{
		const std::string &name = desired_joint_names[i];
		auto it = name_to_position.find(name);
		if (it != name_to_position.end())
		{
			joint_positions[i] = it->second; // Joint position found
		}
		else
		{
			RCLCPP_WARN(rclcpp::get_logger("pilot_arm_node"), "Joint '%s' not found in JointState message", name.c_str());
			joint_positions[i] = 0.0; // Default to 0 if joint not found
		}
	}
}

class Pilot_arm_node : public rclcpp::Node
{
public:
	Pilot_arm_node() : Node("pilot_arm_node")//, ref_gen_A(2.0, 10.0), ref_gen_B(2.0, 10.0)
	{
		// init marvine kine.
		const std::string package_name = "marvin_teleop";
		const std::string package_share_directory = ament_index_cpp::get_package_share_directory(package_name);
		

		this->declare_parameter<std::vector<std::string>>(
			"left_joints",
			std::vector<std::string>{"left_joint1", "left_joint2", "left_joint3", "left_joint4", "left_joint5", "left_joint6", "left_joint7"});
		this->declare_parameter<std::vector<std::string>>(
			"right_joints",
			std::vector<std::string>{"right_joint1", "right_joint2", "right_joint3", "right_joint4", "right_joint5", "right_joint6", "right_joint7"});
		this->declare_parameter<std::string>(
			"config_file","ccs_m6.MvKDCfg");
		this->declare_parameter<std::string>(
			"urdf_file","marvin_CCS_m6.urdf");
		this->declare_parameter<std::string>(
			"srdf_file","marvin_robot.srdf");
		this->declare_parameter<std::string>(
			"mjcf_file","marvin_pro_mink.xml");
		this->declare_parameter<std::string>("eef_name_left", "left_joint7");
		this->declare_parameter<std::string>("eef_name_right", "right_joint7");
		this->declare_parameter<std::string>("elbow_name_left", "left_joint4");
		this->declare_parameter<std::string>("elbow_name_right", "right_joint4");
		this->declare_parameter<std::string>("base_name", "base_link");
		this->declare_parameter<std::string>("left_base_name", "base_L");
		this->declare_parameter<std::string>("right_base_name", "base_R");
		this->declare_parameter<std::string>("left_base_nameJ", "base_to_left_arm");
		this->declare_parameter<std::string>("right_base_nameJ", "base_to_right_arm");
		this->declare_parameter<std::vector<double>>(
			"elbow_ref_dir",
			std::vector<double>{0.0, 0.3, 1.0});
		this->declare_parameter<bool>("use_incremental_control", true);
		
		// Get the parameter
		joint_namesA_ = this->get_parameter("left_joints").as_string_array();
		joint_namesB_ = this->get_parameter("right_joints").as_string_array();

		ee_nameA = this->get_parameter("eef_name_left").as_string();
		ee_nameB = this->get_parameter("eef_name_right").as_string();
		elbow_nameA = this->get_parameter("elbow_name_left").as_string();
		elbow_nameB = this->get_parameter("elbow_name_right").as_string();
		base_name = this->get_parameter("base_name").as_string();
		left_base_name = this->get_parameter("left_base_name").as_string();
		right_base_name = this->get_parameter("right_base_name").as_string();
		left_base_nameJ = this->get_parameter("left_base_nameJ").as_string();
		right_base_nameJ = this->get_parameter("right_base_nameJ").as_string();
		use_incremental_control_ = this->get_parameter("use_incremental_control").as_bool();
		RCLCPP_INFO(this->get_logger(), "use_incremental_control: %s",
					use_incremental_control_ ? "true" : "false");

		std::vector<double> tmp = this->get_parameter("elbow_ref_dir").as_double_array();
        if (tmp.size() == 3)
			{
				for (int i = 0; i < 3; ++i)
					ref_dir[i] = tmp[i];

				RCLCPP_INFO(this->get_logger(), "ref_dir loaded: [%f, %f, %f]", ref_dir[0], ref_dir[1], ref_dir[2]);
			}
			else
			{
				RCLCPP_WARN(this->get_logger(), "ref_dir size != 3, using default");
			}
		

		RCLCPP_INFO(this->get_logger(), "left_base_name: %s", left_base_name.c_str());
		RCLCPP_INFO(this->get_logger(), "right_base_name: %s", right_base_name.c_str());
		RCLCPP_INFO(this->get_logger(), "eef_nameA: %s", ee_nameA.c_str());
		RCLCPP_INFO(this->get_logger(), "eef_nameB: %s", ee_nameB.c_str());
		// for (size_t i = 0; i < 7; ++i)
		// {
		// 	std::cout << joint_namesA_[i] << std::endl;
		// 	;
		// }

		// for (size_t i = 0; i < 7; ++i)
		// {
		// 	std::cout << joint_namesB_[i] << std::endl;
		// 	;
		// }


		config_file = this->get_parameter("config_file").as_string();
		config_file = package_share_directory + "/config/" + config_file;
		RCLCPP_INFO(this->get_logger(), "\033[1;32mUsing config file: %s\033[0m", config_file.c_str());
		urdf_file = this->get_parameter("urdf_file").as_string();
		urdf_file = package_share_directory + "/config/" + urdf_file;
		srdf_file = this->get_parameter("srdf_file").as_string();
		srdf_file = package_share_directory + "/config/" + srdf_file;
		mjcf_file = this->get_parameter("mjcf_file").as_string();
		mjcf_file = package_share_directory + "/config/mjcf/" + mjcf_file;
		MakePara(config_file);
		FX_Robot_Init_Type(0, FX_ROBOT_TYPE_PILOT_CCS);
		FX_Robot_Init_Kine(0, DH[0]);
		FX_BOOL ret1 = FX_Robot_Init_Type(1, FX_ROBOT_TYPE_PILOT_CCS);
		FX_BOOL ret2 = FX_Robot_Init_Kine(1, DH[1]);
		FX_Robot_Init_Lmt(0, PNVA[0], BD[0]);
		FX_Robot_Init_Lmt(1, PNVA[1], BD[1]);


		tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
		tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
		size_t tf_count = 0;

		// while (true)
		// {
		// 	try
		// 	{
		// 		cached_tf_b_left = tf_buffer_->lookupTransform(
		// 			root_link_name, left_base_name, rclcpp::Time(0));
		// 		cached_tf_b_right = tf_buffer_->lookupTransform(
		// 			root_link_name, right_base_name, rclcpp::Time(0));
		// 		RCLCPP_INFO(this->get_logger(), "Transform from base_link to right_base_link and left_base_link found");
		// 		break; // Exit the loop if transform is found
		// 	}
		// 	catch (...)
		// 	{
		// 		// RCLCPP_WARN(this->get_logger(), "Waiting for transform from controller_frame to right_base_link");
		// 		tf_count++;
		// 		if (tf_count > 100)
		// 		{
		// 			RCLCPP_ERROR(this->get_logger(), "Failed to get transform from controller_frame to right_base_link after 100 attempts");
		// 			break; // Exit the loop after too many attempts
		// 		}
		// 		std::this_thread::sleep_for(std::chrono::seconds(1));
		// 		continue;
		// 	}
		// }

		// for logging 
		// joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
		rclcpp::SensorDataQoS sensor_data_qos;
		joint_fb_subscriber_ = this->create_subscription<marvin_msgs::msg::Jointfeedback>(
			"info/joint_feedback", sensor_data_qos,
			std::bind(&Pilot_arm_node::joint_feedback_callback, this, std::placeholders::_1));
		// pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("right_arm/eef_pose", sensor_data_qos);
		pose_publisher_eefA_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("control/eef_cmd_A", sensor_data_qos);
		pose_publisher_eefB_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("control/eef_cmd_B", sensor_data_qos);

		target_pose_subscriberA_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
			"control/target_poseL", sensor_data_qos,
			std::bind(&Pilot_arm_node::pose_callbackA, this, std::placeholders::_1));
		target_pose_subscriberB_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
			"control/target_poseR", sensor_data_qos,
			std::bind(&Pilot_arm_node::pose_callbackB, this, std::placeholders::_1));
		left_DS_subscriver_ = this->create_subscription<std_msgs::msg::Bool>(
			"control/enableL", sensor_data_qos,
			std::bind(&Pilot_arm_node::left_DS_callback, this, std::placeholders::_1));
		right_DS_subscriver_ = this->create_subscription<std_msgs::msg::Bool>(
			"control/enableR", sensor_data_qos,
			std::bind(&Pilot_arm_node::right_DS_callback, this, std::placeholders::_1));
		mode_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
			"control/switch_state", 10,
			std::bind(&Pilot_arm_node::mode_callback, this, std::placeholders::_1));
		Elbow_pose_subscriberA_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
			"control/Elbow_left", sensor_data_qos,
			std::bind(&Pilot_arm_node::elbow_pose_callbackA, this, std::placeholders::_1));
		Elbow_pose_subscriberB_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
			"control/Elbow_right", sensor_data_qos,
			std::bind(&Pilot_arm_node::elbow_pose_callbackB, this, std::placeholders::_1));
		pose_A_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("info/eef_left", 10);
		pose_B_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("info/eef_right", 10);
		joint_cmd_publisherA_ = this->create_publisher<marvin_msgs::msg::Jointcmd>("control/joint_cmd_tele_A", sensor_data_qos);
		joint_cmd_publisherB_ = this->create_publisher<marvin_msgs::msg::Jointcmd>("control/joint_cmd_tele_B", sensor_data_qos);
		ik_publisherA_ = this->create_publisher<marvin_msgs::msg::Jointcmd>("control/ik_cmd_A", sensor_data_qos);
		ik_publisherB_ = this->create_publisher<marvin_msgs::msg::Jointcmd>("control/ik_cmd_B", sensor_data_qos);
		ik_request_publisher_ = this->create_publisher<marvin_msgs::msg::IKRequest>("control/ik_request", sensor_data_qos);
		ik_result_subscriber_ = this->create_subscription<marvin_msgs::msg::IKResult>(
			"control/ik_result", sensor_data_qos,
			std::bind(&Pilot_arm_node::ik_result_callback, this, std::placeholders::_1));
		collision_pub_A = this->create_publisher<std_msgs::msg::Bool>("info/collision_statusA", sensor_data_qos);
		collision_pub_B = this->create_publisher<std_msgs::msg::Bool>("info/collision_statusB", sensor_data_qos);
		marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("info/collision_marker", 10);
		// joint_feedback_publisherA_ = this->create_publisher<marvin_msgs::msg::Jointfeedback>("control/joint_target_A", sensor_data_qos);
		// joint_feedback_publisherB_ = this->create_publisher<marvin_msgs::msg::Jointfeedback>("control/joint_target_B", sensor_data_qos);

		// joint_feedback_subscriberA_ = this->create_subscription<marvin_msgs::msg::Jointfeedback>(
		// 	"control/joint_target_scaled_A", sensor_data_qos,
		// 	std::bind(&Pilot_arm_node::joint_feedback_callbackA, this, std::placeholders::_1));
		// joint_feedback_subscriberB_ = this->create_subscription<marvin_msgs::msg::Jointfeedback>(
		// 	"control/joint_target_scaled_B", sensor_data_qos,
		// 	std::bind(&Pilot_arm_node::joint_feedback_callbackB, this, std::placeholders::_1));

		pin_init();

		timer_A = this->create_wall_timer(
			std::chrono::milliseconds(1),
			std::bind(&Pilot_arm_node::on_timer_A, this));
		timer_B = this->create_wall_timer(
			std::chrono::milliseconds(1),
			std::bind(&Pilot_arm_node::on_timer_B, this));

		timer_C = this->create_wall_timer(
			std::chrono::milliseconds(1),
			std::bind(&Pilot_arm_node::pin_tick, this));
	}

	void mode_callback(const std_msgs::msg::Int32::SharedPtr msg)
	{
		if(msg->data == 2)
		{
			teleop_mode = false;
			RCLCPP_INFO(this->get_logger(), "Teleoperation mode cannot be enabled");
			left_hooked = false;
			right_hooked = false;
		}
		else
		{
			teleop_mode = true;
			RCLCPP_INFO(this->get_logger(), "Teleoperation mode enabled");
		}
	}
	// void zero_space_B_callback(const std_msgs::msg::Float32::SharedPtr msg)
	// {
	// 	zero_space_B = msg->data;
	// 	// RCLCPP_INFO(this->get_logger(), "zero_space_B: %f", zero_space_B);
	// }

	void pub_joint_cmdA(double *jv)
	{
		auto joint_state_msg = marvin_msgs::msg::Jointcmd();
		joint_state_msg.header.stamp = this->now();

		joint_state_msg.header.frame_id = left_base_name;

		for (size_t i = 0; i < 7; ++i)
		{
			joint_state_msg.positions[i] = (jv[i] ); // No convertion needed
		}
		if(left_hooked)
		joint_cmd_publisherA_->publish(joint_state_msg);
	}

	void pub_joint_cmdB(double *jv)
	{
		auto joint_state_msg = marvin_msgs::msg::Jointcmd();
		joint_state_msg.header.stamp = this->now();
		joint_state_msg.header.frame_id = right_base_name;

		for (size_t i = 0; i < 7; ++i)
		{
			joint_state_msg.positions[i] = (jv[i] ); // No convertion needed
		}
		if(right_hooked)
		joint_cmd_publisherB_->publish(joint_state_msg);
	}

	void pub_ikA(double *jv)
	{
		auto joint_state_msg = marvin_msgs::msg::Jointcmd();
		joint_state_msg.header.stamp = this->now();
		joint_state_msg.header.frame_id = left_base_name;

		for (size_t i = 0; i < 7; ++i)
		{
			joint_state_msg.positions[i] = (jv[i] * FXARM_D2R); // Convert radians to degrees
		}

		ik_publisherA_->publish(joint_state_msg);
	}
	void pub_ikB(double *jv)
	{
		auto joint_state_msg = marvin_msgs::msg::Jointcmd();
		joint_state_msg.header.stamp = this->now();
		joint_state_msg.header.frame_id = right_base_name;

		for (size_t i = 0; i < 7; ++i)
		{
			joint_state_msg.positions[i] = (jv[i] * FXARM_D2R); // Convert radians to degrees
		}

		ik_publisherB_->publish(joint_state_msg);
	}

	// void FKine(double joint_positions[7], double pg[4][4]){
	//     // Convert joint positions from radians to degrees
	//     for (int i = 0; i < 7; ++i) {
	//         joint_positions[i] *= (180.0 / M_PI);
	//     }
	//     FX_Kine_Pilot(&the_pilot, joint_positions, pg);
	// }

private:
	void MakePara(std::string path) // Load the configuration file
	{

		FX_BOOL ret = LOADMvCfg((char *)path.c_str(), TYPE, GRV, DH, PNVA, BD, Mass, MCP, I);
		if (ret == FX_FALSE)
		{
			printf("LOAD ERR\n");
		}
		else
		{

			// printf("LOAD OK\n");
			// for (int i = 0; i < 2; i++)
			// {
			// 	for (int j = 0; j < 8; j++)
			// 	{
			// 		for (int k = 0; k < 4; k++)
			// 		{
			// 			printf("%f ", DH[i][j][k]);
			// 		}
			// 		printf("\n");
			// 	}
			// 	printf("\n");
			// }
		}
	}

	void on_timer_A()
	{
		IK_A();
	}

	void on_timer_B()
	{
		IK_B();
	}
	


	void pub_eef_pose(pinocchio::SE3 T_eefL, pinocchio::SE3 T_eefR)
	{
		// geometry_msgs::msg::Pose poseA, poseB;
		geometry_msgs::msg::PoseStamped poseA_base, poseB_base, poseA_, poseB_;
		
		poseA_base.header.stamp = this->now();
		poseB_base.header.stamp = this->now();
		poseA_base.header.frame_id = root_link_name;
		poseB_base.header.frame_id = root_link_name;

		Eigen::Quaterniond quatA(T_eefL.rotation());
		Eigen::Quaterniond quatB(T_eefR.rotation());
		// Eigen::Quaterniond quatB = T_eefR.rotation();
		poseA_base.pose.position.x = T_eefL.translation().x();
		poseA_base.pose.position.y = T_eefL.translation().y();
		poseA_base.pose.position.z = T_eefL.translation().z();
		poseA_base.pose.orientation.x = quatA.x();
		poseA_base.pose.orientation.y = quatA.y();
		poseA_base.pose.orientation.z = quatA.z();
		poseA_base.pose.orientation.w = quatA.w();
		poseB_base.pose.position.x = T_eefR.translation().x();
		poseB_base.pose.position.y = T_eefR.translation().y();
		poseB_base.pose.position.z = T_eefR.translation().z();
		poseB_base.pose.orientation.x = quatB.x();
		poseB_base.pose.orientation.y = quatB.y();
		poseB_base.pose.orientation.z = quatB.z();
		poseB_base.pose.orientation.w = quatB.w();
		pose_A_pub_->publish(poseA_base); // Publish the pose message

		pose_B_pub_->publish(poseB_base); // Publish the pose message
	}


	void ik_result_callback(const marvin_msgs::msg::IKResult::SharedPtr msg)
	{

		if(!teleop_mode)
		{
			 RCLCPP_WARN(this->get_logger(), "Received IKResult but teleoperation mode is disabled. Ignoring.");
			 return;
		}
		if (msg->joint_state.position.size() == model_.nq)
		{
			double joint_cmd_armA[7], joint_cmd_armB[7];
			for (size_t i = 0; i < 7; ++i)
			{
				joint_cmd_armA[i] = msg->joint_state.position[i];
				joint_cmd_armB[i] = msg->joint_state.position[i + 7];
			}
			if(left_hooked)
			{
				pub_joint_cmdA(joint_cmd_armA);
			}
			if(right_hooked)
			{
			pub_joint_cmdB(joint_cmd_armB);
			}
		}
		 else {
			RCLCPP_WARN(this->get_logger(), "Received IKResult with incorrect joint state size: %zu", msg->joint_state.position.size());
		}
	}

	void joint_feedback_callback(marvin_msgs::msg::Jointfeedback::SharedPtr msg)
	{
		using namespace pinocchio;
		SE3 T_A_in_world, T_B_in_world;
		eef_points_A.clear();
		eef_points_B.clear();
		collision_status_A.clear();
		collision_status_B.clear();
		for (size_t i = 0; i < 14; ++i)
		{
			q_[i] = msg->positions[i];
			v_[i] = msg->velocities[i];
		}

		// Extract joint positions for left and right arms
		for (size_t i = 0; i < 7; ++i)
		{
			joint_positions_A[i] = msg->positions[i];
			joint_positions_AD[i] = msg->positions[i] * FXARM_R2D;	   // Convert radians to degrees
			joint_positions_B[i] = msg->positions[i + 7];			   // Assuming the second half of the message contains right arm joint positions
			joint_positions_BD[i] = msg->positions[i + 7] * FXARM_R2D; // Convert radians to degrees
		}
		// FX_Kine_Pilot(&the_pilot, joint_positions_AD, pgA);
		// FX_Kine_Pilot(&the_pilot, joint_positions_BD, pgB);

		// FX_Robot_Kine_FK(0, joint_positions_AD, pgA);
		// FX_Robot_Kine_FK(1, joint_positions_BD, pgB);
		// T_pgA = convertArrayToIsometry(pgA);
		// T_pgB = convertArrayToIsometry(pgB);

		// for (int i =0;i<4;i++){
		// 	for (int j =0;j<4;j++){
		// 		printf("%f ", pgB[i][j]);
		// 	}
		// 	printf("\n");
		// }
		// printf("---\n");
		// pub_eef_pose();
	}

	void pose_callbackA(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
	{
		// printf("pose_callback: %s\n", msg->header.frame_id.c_str());
		geometry_msgs::msg::PoseStamped controller_pose = *msg;
		geometry_msgs::msg::PoseStamped output_pose;
		Eigen::Isometry3d pgAF = Eigen::Isometry3d::Identity();
		rclcpp::Time stamp(controller_pose.header.stamp);
		double timestamp = stamp.seconds();

		// pose in left_base_link frame
		try
		{
			output_pose = transform_pose(controller_pose,
										 tf_buffer_->lookupTransform(base_name, controller_pose.header.frame_id, rclcpp::Time(0)));

			pgAF.translation() << output_pose.pose.position.x, output_pose.pose.position.y, output_pose.pose.position.z;
			pgAF.linear() = Eigen::Quaterniond(
								output_pose.pose.orientation.w,
								output_pose.pose.orientation.x,
								output_pose.pose.orientation.y,
								output_pose.pose.orientation.z)
								.toRotationMatrix();
			pgRaw_L = pinocchio::SE3(pgAF.rotation(), pgAF.translation());// SE3 in left_base_link frame
			// ref_gen_A.updateMeasurement(pgRaw_L, timestamp);

		}
		catch (tf2::TransformException &ex)
		{
			RCLCPP_ERROR(rclcpp::get_logger("logger"), "Transform failed: %s", ex.what());
		}
	}

	void pose_callbackB(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
	{
		// printf("pose_callback: %s\n", msg->header.frame_id.c_str());
		geometry_msgs::msg::PoseStamped controller_pose = *msg;
		geometry_msgs::msg::PoseStamped output_pose;
		Eigen::Isometry3d pgBF = Eigen::Isometry3d::Identity();
		rclcpp::Time stamp(controller_pose.header.stamp);
		double timestamp = stamp.seconds();

		// pose in right_base_link frame
		try
		{
			output_pose = transform_pose(controller_pose,
										 tf_buffer_->lookupTransform(base_name, controller_pose.header.frame_id, rclcpp::Time(0)));

			pgBF.translation() << output_pose.pose.position.x, output_pose.pose.position.y, output_pose.pose.position.z;
			pgBF.linear() = Eigen::Quaterniond(
								output_pose.pose.orientation.w,
								output_pose.pose.orientation.x,
								output_pose.pose.orientation.y,
								output_pose.pose.orientation.z)
								.toRotationMatrix();
			pgRaw_R = pinocchio::SE3(pgBF.rotation(), pgBF.translation());// SE3 in right_base_link frame
		}
		catch (tf2::TransformException &ex)
		{
			RCLCPP_ERROR(rclcpp::get_logger("logger"), "Transform failed: %s", ex.what());
		}
	}

void elbow_pose_callbackA(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
	{
		// printf("pose_callback: %s\n", msg->header.frame_id.c_str());
		geometry_msgs::msg::PoseStamped elbow_pose = *msg;
		geometry_msgs::msg::PoseStamped output_pose;
		Eigen::Isometry3d pgAF = Eigen::Isometry3d::Identity();
		rclcpp::Time stamp(elbow_pose.header.stamp);
		double timestamp = stamp.seconds();

		// pose in right_base_link frame
		try
		{
			output_pose = transform_pose(elbow_pose,
										 tf_buffer_->lookupTransform(base_name, elbow_pose.header.frame_id, rclcpp::Time(0)));
			pgAF.translation() << output_pose.pose.position.x, output_pose.pose.position.y, output_pose.pose.position.z;
			pgAF.linear() = Eigen::Quaterniond(
								output_pose.pose.orientation.w,
								output_pose.pose.orientation.x,
								output_pose.pose.orientation.y,
								output_pose.pose.orientation.z)
								.toRotationMatrix();
			Elbow_left = pinocchio::SE3(pgAF.rotation(), pgAF.translation());

		}
		catch (tf2::TransformException &ex)
		{
			RCLCPP_ERROR(rclcpp::get_logger("logger"), "Transform failed: %s", ex.what());
		}
	}

	void elbow_pose_callbackB(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
	{
		// printf("pose_callback: %s\n", msg->header.frame_id.c_str());
		geometry_msgs::msg::PoseStamped elbow_pose = *msg;
		geometry_msgs::msg::PoseStamped output_pose;
		Eigen::Isometry3d pgBF = Eigen::Isometry3d::Identity();
		rclcpp::Time stamp(elbow_pose.header.stamp);
		double timestamp = stamp.seconds();

		// pose in right_base_link frame
		try
		{
			output_pose = transform_pose(elbow_pose,
										 tf_buffer_->lookupTransform(base_name, elbow_pose.header.frame_id, rclcpp::Time(0)));

			pgBF.translation() << output_pose.pose.position.x, output_pose.pose.position.y, output_pose.pose.position.z;
			pgBF.linear() = Eigen::Quaterniond(
								output_pose.pose.orientation.w,
								output_pose.pose.orientation.x,
								output_pose.pose.orientation.y,
								output_pose.pose.orientation.z)
								.toRotationMatrix();
			Elbow_right = pinocchio::SE3(pgBF.rotation(), pgBF.translation());
			
		}
		catch (tf2::TransformException &ex)
		{
			RCLCPP_ERROR(rclcpp::get_logger("logger"), "Transform failed: %s", ex.what());
		}
	}

	inline double clamp(double val, double min_val, double max_val)
	{
		return std::max(min_val, std::min(max_val, val));
	}

	// rotationOrder = Eigen::XYZ, Eigen::ZYX etc.
	Eigen::Matrix3d limitRotationAxisAngle(
		const Eigen::Matrix3d &R1,
		const Eigen::Matrix3d &R2,
		double max_angle_deg)
	{
		// Relative rotation from R1 to R2
		Eigen::Matrix3d R_rel = R2 * R1.transpose();

		Eigen::AngleAxisd aa(R_rel);
		double angle = aa.angle();		  // [0, pi]
		Eigen::Vector3d axis = aa.axis(); // unit vector

		double max_angle_rad = max_angle_deg * M_PI / 180.0;

		// If angle is zero (or very small), return R1 directly (no rotation)
		if (angle < 1e-12)
		{
			return R1;
		}

		// Clamp angle to max_angle_rad by scaling if necessary
		if (angle > max_angle_rad)
		{
			angle = max_angle_rad;
		}
		else if (angle < -max_angle_rad)
		{
			angle = -max_angle_rad;
		}

		// Normalize the axis to ensure it is a unit vector
		axis.normalize();

		// If the angle is zero after clamping, return R1 directly
		{
			/* code */
		}

		// Rebuild limited relative rotation
		Eigen::AngleAxisd aa_limited(angle, axis);
		Eigen::Matrix3d R_limited_rel = aa_limited.toRotationMatrix();

		// Apply limited rotation relative to R1
		return R_limited_rel * R1;
	}

	Eigen::Matrix3d scaleRotationAxisAngle(
		const Eigen::Matrix3d &R1,
		const Eigen::Matrix3d &R2,
		double scale_factor,
		double max_angle_deg = 180.0 // Default max angle in degrees
	)
	{
		// Relative rotation from R1 to R2
		Eigen::Matrix3d R_rel = R2 * R1.transpose();

		// Convert to angle-axis representation
		Eigen::AngleAxisd aa(R_rel);
		double angle = aa.angle();		  // [0, pi]
		Eigen::Vector3d axis = aa.axis(); // unit vector

		// Convert max angle to radians
		double max_angle_rad = max_angle_deg * M_PI / 180.0;

		// If angle is very small, return R1 (no rotation)
		if (angle < 1e-12)
		{
			return R1;
		}

		// Apply scale factor to the rotation angle
		angle *= scale_factor;

		// Clamp angle to max_angle_rad
		if (angle > max_angle_rad)
		{
			angle = max_angle_rad;
		}
		else if (angle < -max_angle_rad)
		{
			angle = -max_angle_rad;
		}

		// If the angle is zero after scaling and clamping, return R1
		if (std::abs(angle) < 1e-12)
		{
			return R1;
		}

		// Normalize the axis to ensure it is a unit vector
		axis.normalize();

		// Rebuild limited relative rotation with scaled angle
		Eigen::AngleAxisd aa_limited(angle, axis);
		Eigen::Matrix3d R_limited_rel = aa_limited.toRotationMatrix();

		// Apply limited rotation relative to R1
		return R_limited_rel * R1;
	}

	inline double wrapTo180(double angle_deg)
	{
		angle_deg = fmod(angle_deg + 180.0, 360.0);
		if (angle_deg < 0)
			angle_deg += 360.0;
		return angle_deg - 180.0;
	}

	Eigen::Vector3d minimizeAbsRPY(const Eigen::Vector3d &rpy_deg)
	{
		// Wrap input angles to [-180,180]
		Eigen::Vector3d A;
		for (int i = 0; i < 3; ++i)
			A[i] = wrapTo180(rpy_deg[i]);

		// Compute alternative solution B:
		// B[0] = yaw + 180, B[1] = 180 - pitch, B[2] = roll + 180
		Eigen::Vector3d B;
		B[0] = wrapTo180(A[0] + 180.0);
		B[1] = wrapTo180(180.0 - A[1]);
		B[2] = wrapTo180(A[2] + 180.0);

		// Compare sums of absolute angles
		double normA = A.cwiseAbs().sum();
		double normB = B.cwiseAbs().sum();

		return (normA <= normB) ? A : B;
	}

	Eigen::Matrix3d limitRotationRPY(
		const Eigen::Matrix3d &R1,
		const Eigen::Matrix3d &R2,
		const Eigen::Vector3d &max_rpy_deg // [roll_limit, pitch_limit, yaw_limit]
	)
	{
		// Relative rotation from R1 to R2
		Eigen::Matrix3d R_rel = R2 * R1.transpose();

		// Extract RPY in radians, order: ZYX (yaw, pitch, roll)
		// Eigen::Vector3d rpy = R_rel.eulerAngles(2, 1, 0); // yaw, pitch, roll (rad)
		// rpy = minimizeAbsRPY(rpy * 180.0 / M_PI); // Convert to degrees and minimize absolute angles

		tf2::Matrix3x3 m;
		m.setValue(R_rel(0, 0), R_rel(0, 1), R_rel(0, 2),
				   R_rel(1, 0), R_rel(1, 1), R_rel(1, 2),
				   R_rel(2, 0), R_rel(2, 1), R_rel(2, 2));

		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
		Eigen::Vector3d rpy;
		rpy[0] = yaw * 180.0 / M_PI;   // yaw
		rpy[1] = pitch * 180.0 / M_PI; // pitch
		rpy[2] = roll * 180.0 / M_PI;  // roll

		std::cout << "rpy before limit: " << rpy.transpose() << std::endl;
		// Clamp each angle independently
		for (int i = 0; i < 3; ++i)
		{
			rpy[i] = std::clamp(rpy[i], -max_rpy_deg[i], max_rpy_deg[i]);
		}

		// Convert back to radians
		rpy = rpy * M_PI / 180.0;

		// Rebuild limited relative rotation
		Eigen::Matrix3d R_limited_rel;
		R_limited_rel =
			Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitZ()) * // yaw
			Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) * // pitch
			Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitX());  // roll

		// Apply limited rotation relative to R1
		return R_limited_rel * R1;
	}

	void IK_A()
	{
		// auto now = std::chrono::steady_clock::now();
		// auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - grip_L_stamp);
		// double timestamp = std::chrono::duration<double>(
		// 	std::chrono::steady_clock::now().time_since_epoch()
		// ).count();
		// // filter_A_.filter(pgRaw_L, timestamp);
		// // Eigen::Isometry3d T_target =
        // // ref_gen_A.getReference(timestamp);
		// if (duration.count() > 20000)
		// {
		// 	return; // Skip IK calculation if the left DSwitch is pressed within 0.1 seconds
		// }
		// 	Eigen::Isometry3d Flange_L,T_pgAF;
		// 	Eigen::Matrix3d rot90Y = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()).toRotationMatrix();
		// 	Eigen::Matrix3d rot90Z = Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
		// 	Eigen::Matrix3d rot90X = Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitX()).toRotationMatrix();
		// 	Eigen::Matrix3d rotx = Eigen::AngleAxisd(-M_PI / 8, Eigen::Vector3d::UnitX()).toRotationMatrix();
		// 	Flange_L.linear() = rotx * rot90X * rot90Y * rot90Z;
		// 	Flange_L.translation() << 0.0, 0.0, 0.0;

		if (left_dswitch)
			{
				if (!left_dswitch_last)
				{
					// RCLCPP_INFO(this->get_logger(),"left gripped!");
					Eigen::Vector3d disp= pgRaw_L.translation() - T_pgA.translation();
					double norm_disp = disp.norm();
					double hook_thres_L = use_incremental_control_ ? 1.0 : 0.1;
					if (norm_disp < hook_thres_L)
					{
						left_hooked = true;
						pgA_offset = T_pgA * pgRaw_L.inverse();
						// Record initial poses for incremental mode (pos and rot separately)
						init_human_pos_L = pgRaw_L.translation();
						init_human_rot_L = pgRaw_L.rotation();
						init_robot_pos_A = T_pgA.translation();
						init_robot_rot_A = T_pgA.rotation();
					}
					left_dswitch_last = true; // prevent multi-fire until next DS message
				}
			}

		// 	for (int i = 0; i < 7; ++i) {
		// 		ref_jointA[i] = joint_positions_AD[i];
		// 	}
			

			
		// 	T_pgAF = pgRaw_L * Flange_L * pgA_offset.inverse();
		// 	Eigen::Matrix3d rotation_matrix = T_pgAF.linear();
		// 	geometry_msgs::msg::PoseStamped pose_msg;
		// 	pose_msg.header.stamp = this->now();
		// 	pose_msg.header.frame_id = left_base_name; // Set the frame_id
		// 	Eigen::Quaterniond quat_msg = Eigen::Quaterniond(rotation_matrix);
		// 	pose_msg.pose.position.x = T_pgAF.translation().x();
		// 	pose_msg.pose.position.y = T_pgAF.translation().y();
		// 	pose_msg.pose.position.z = T_pgAF.translation().z();
		// 	pose_msg.pose.orientation.x = quat_msg.x();
		// 	pose_msg.pose.orientation.y = quat_msg.y();
		// 	pose_msg.pose.orientation.z = quat_msg.z();
		// 	pose_msg.pose.orientation.w = quat_msg.w();
		// 	if (true)
		// 	{
		// 		pose_publisher_eefA_->publish(pose_msg);
		// 		convertIsometryToArray(T_pgAF, pgAT);
		// 	}
		// // max_diff = std::abs(ret_joint[4] - joint_positions_AD[4]);

		// FX_DOUBLE ret_joint[7], q_aD[7];
		// FX_BOOL solved;
		// FX_BOOL IsOutRange, Inrange;
		// FX_BOOL Is123Deg;
		// FX_BOOL Is567Deg;
		// // FX_DOUBLE ref_dir[3] = {0, 0.3, 1.0};
		// FX_InvKineSolvePara solve_para,solve_para2;
		// memcpy(solve_para.m_Input_IK_RefJoint, ref_jointA, sizeof(ref_jointA));
		// memcpy(solve_para.m_Input_IK_TargetTCP, pgAT, sizeof(pgAT));
		// solve_para.m_Input_IK_ZSPType = FX_PILOT_NSP_TYPES_NEAR_DIR;
		// solve_para.m_Input_IK_ZSPPara[0] = ref_dir[0];
		// solve_para.m_Input_IK_ZSPPara[1] = ref_dir[1];
		// solve_para.m_Input_IK_ZSPPara[2] = ref_dir[2];


		// solved = FX_Robot_Kine_IK(0, &solve_para);
		// memcpy(solve_para2.m_Input_IK_RefJoint, ref_jointA, sizeof(ref_jointA));
		// memcpy(solve_para2.m_Input_IK_TargetTCP, pgAT, sizeof(pgAT));
		// solve_para2.m_Input_IK_ZSPType = FX_PILOT_NSP_TYPES_NEAR_DIR;

		// Eigen::Vector3d x_axis = Elbow_left.rotation().col(0);
		// solve_para2.m_Input_IK_ZSPPara[0] = -x_axis[0];
		// solve_para2.m_Input_IK_ZSPPara[1] = -x_axis[1];
		// solve_para2.m_Input_IK_ZSPPara[2] = -x_axis[2];

		// solved = FX_Robot_Kine_IK(0, &solve_para2);


		// if (solved){

		// 	memcpy(ret_joint, solve_para2.m_Output_RetJoint, sizeof(ret_joint));

		// 	for (int i=0; i<7; i++){
		// 		if (ret_joint[i] > solve_para2.m_Output_RunLmtP[i])
		// 				{
		// 					ret_joint[i] = solve_para2.m_Output_RunLmtP[i];
		// 				}
		// 				if (ret_joint[i] < solve_para2.m_Output_RunLmtN[i])
		// 				{
		// 					ret_joint[i] = solve_para2.m_Output_RunLmtN[i];
		// 				}
		// 	}
		// 	if (solve_para2.m_Output_IsJntExd){
		// 		// return;
		// 	}
		// 	pub_ikA(ret_joint);

		// 	if (solved == FX_TRUE && left_hooked)
		// 	{
		// 		if(!will_coll_A){
		// 			memcpy(ret_jointA, ret_joint, sizeof(ret_jointA));
		// 		}
		// 		else{
		// 			memcpy(ret_jointA, joint_positions_AD, sizeof(ret_jointA));
		// 		}
		// 		pub_joint_cmdA(ret_jointA);
		// 	}
			
		// }
	}

	void IK_B()
	{
		// auto now = std::chrono::steady_clock::now();
		// auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - grip_R_stamp);
		// double timestamp = std::chrono::duration<double>(
		// 	std::chrono::steady_clock::now().time_since_epoch()
		// ).count();
		// // filter_B_.filter(pgRaw_R, timestamp);
		// // Eigen::Isometry3d T_target =
        // // ref_gen_B.getReference(timestamp);
		// if (duration.count() > 20000)
		// {
		// 	return; // Skip IK calculation if the right DSwitch is pressed within 0.1 seconds
		// }
		// // Eigen::Isometry3d Flange_R, T_pgBF;
		// // Eigen::Matrix3d rot90Y = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()).toRotationMatrix();
		// // Eigen::Matrix3d rot90Z = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
		// // Eigen::Matrix3d rot90X = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()).toRotationMatrix();
		// // Eigen::Matrix3d rotx = Eigen::AngleAxisd(0 * M_PI / 8, Eigen::Vector3d::UnitX()).toRotationMatrix();
		// // Flange_R.linear() = rotx * rot90X * rot90Y * rot90Z;
		// // Flange_R.translation() << 0.0, 0.0, 0.0;

		if (right_dswitch)
		{
			if (!right_dswitch_last)
			{
				// RCLCPP_INFO(this->get_logger(),"right gripped!");
				Eigen::Vector3d disp = pgRaw_R.translation() - T_pgB.translation();
				double norm_disp = disp.norm();
				double hook_thres_R = use_incremental_control_ ? 1.0 : 0.1;
				if (norm_disp < hook_thres_R)
				{
					right_hooked = true;
					pgB_offset = T_pgB * pgRaw_R.inverse();
					// Record initial poses for incremental mode (pos and rot separately)
					init_human_pos_R = pgRaw_R.translation();
					init_human_rot_R = pgRaw_R.rotation();
					init_robot_pos_B = T_pgB.translation();
					init_robot_rot_B = T_pgB.rotation();
				}
				right_dswitch_last = true; // prevent multi-fire until next DS message
			}
		}

		// for (int i = 0; i < 7; ++i)
		// {
		// 	ref_jointB[i] = joint_positions_BD[i];
		// }

		// T_pgBF = pgRaw_R * Flange_R * pgB_offset.inverse();
		// Eigen::Matrix3d rotation_matrix = T_pgBF.linear();
		// geometry_msgs::msg::PoseStamped pose_msg;
		// pose_msg.header.stamp = this->now();
		// pose_msg.header.frame_id = right_base_name; // Set the frame_id
		// Eigen::Quaterniond quat_msg = Eigen::Quaterniond(rotation_matrix);
		// pose_msg.pose.position.x = T_pgBF.translation().x();
		// pose_msg.pose.position.y = T_pgBF.translation().y();
		// pose_msg.pose.position.z = T_pgBF.translation().z();
		// pose_msg.pose.orientation.x = quat_msg.x();
		// pose_msg.pose.orientation.y = quat_msg.y();
		// pose_msg.pose.orientation.z = quat_msg.z();
		// pose_msg.pose.orientation.w = quat_msg.w();
		// if (true)
		// {
		// 	pose_publisher_eefB_->publish(pose_msg);
		// 	convertIsometryToArray(T_pgBF, pgBT);
		// }
		// // max_diff = std::abs(ret_joint[4] - joint_positions_AD[4]);
		
		// FX_DOUBLE ret_joint[7], q_bD[7];
		// FX_BOOL solved;
		// FX_BOOL IsOutRange, Inrange;
		// FX_BOOL Is123Deg;
		// FX_BOOL Is567Deg;
		// // FX_DOUBLE ref_dir[3] = {0, 0.3, 1.0};
		// FX_InvKineSolvePara solve_para,solve_para2;
		// memcpy(solve_para.m_Input_IK_RefJoint, ref_jointB, sizeof(ref_jointB));
		// memcpy(solve_para.m_Input_IK_TargetTCP, pgBT, sizeof(pgBT));
		// solve_para.m_Input_IK_ZSPType = FX_PILOT_NSP_TYPES_NEAR_DIR;
		// solve_para.m_Input_IK_ZSPPara[0] = -ref_dir[0];
		// solve_para.m_Input_IK_ZSPPara[1] = -ref_dir[1];
		// solve_para.m_Input_IK_ZSPPara[2] = -ref_dir[2];

		// // std::cout<<"ref_dir:"<<solve_para.m_Input_IK_ZSPPara[0]<<", "<<solve_para.m_Input_IK_ZSPPara[1]<<", "<<solve_para.m_Input_IK_ZSPPara[2]<<std::endl;

	
		// solved = FX_Robot_Kine_IK(1, &solve_para);
		// // std::cout<<"opt_null_dir"<<solve_para.m_null_elbow[0]<<", "<<solve_para.m_null_elbow[1]<<", "<<solve_para.m_null_elbow[2]<<std::endl;
		// memcpy(solve_para2.m_Input_IK_RefJoint, ref_jointB, sizeof(ref_jointB));
		// memcpy(solve_para2.m_Input_IK_TargetTCP, pgBT, sizeof(pgBT));
		// solve_para2.m_Input_IK_ZSPType = FX_PILOT_NSP_TYPES_NEAR_DIR;
		
		// Eigen::Vector3d x_axis = Elbow_right.rotation().col(0);
		// solve_para2.m_Input_IK_ZSPPara[0] = -x_axis[0];
		// solve_para2.m_Input_IK_ZSPPara[1] = -x_axis[1];
		// solve_para2.m_Input_IK_ZSPPara[2] = -x_axis[2];
		// // solve_para2.m_Input_IK_ZSPPara[0] = solve_para.m_null_elbow[0];
		// // solve_para2.m_Input_IK_ZSPPara[1] = solve_para.m_null_elbow[1];
		// // solve_para2.m_Input_IK_ZSPPara[2] = solve_para.m_null_elbow[2];

		// solved = FX_Robot_Kine_IK(1, &solve_para2);

		
		// if (solved){

		// 	memcpy(ret_joint, solve_para2.m_Output_RetJoint, sizeof(ret_joint));


		// 	// double max_diff = 0.0;
		// 	// for (int i = 4; i < 7; ++i) {
		// 	// 	double diff = std::abs(joint_positions_BD[i] - ret_joint[i]);
		// 	// 	if (diff > max_diff) {
		// 	// 		max_diff = diff;
		// 	// 	}
		// 	// }
		// 	// // max_diff = std::abs(ret_joint[4] - joint_positions_AD[4]);
		// 	// double diff[7];

		// 	// if (max_diff > 100.0){
		// 	// 	for (int i = 4; i < 7; ++i) {
		// 	// 		ret_joint[i] = joint_positions_BD[i];
		// 	// 	}
		// 	// }
			
		// 	for (int i=0; i<7; i++){
		// 		if (ret_joint[i] > solve_para2.m_Output_RunLmtP[i])
		// 				{
		// 					ret_joint[i] = solve_para2.m_Output_RunLmtP[i];
		// 				}
		// 				if (ret_joint[i] < solve_para2.m_Output_RunLmtN[i])
		// 				{
		// 					ret_joint[i] = solve_para2.m_Output_RunLmtN[i];
		// 				}
		// 	}
		// 	if (solve_para2.m_Output_IsJntExd){
		// 		// return;
		// 	}
		// 	pub_ikB(ret_joint);
			

		// 	if (solved == FX_TRUE && right_hooked)
		// 	{
		// 		if(!will_coll_B){	
		// 			memcpy(ret_jointB, ret_joint, sizeof(ret_jointB));
		// 		}
		// 		else{
		// 			memcpy(ret_jointB, joint_positions_BD, sizeof(ret_jointB));
		// 		}
		// 		pub_joint_cmdB(ret_jointB);
		// 	}
		// }
	}

	void left_DS_callback(const std_msgs::msg::Bool::SharedPtr msg)
	{
		left_dswitch_last = left_dswitch;
		left_dswitch = msg->data;
		grip_L_stamp = std::chrono::steady_clock::now();
		if (!left_dswitch)
		{
			left_hooked = false;
		}
		// RCLCPP_INFO(rclcpp::get_logger("pilot_arm_node"), "Left DSwitch: %s", left_dswitch ? "ON" : "OFF");
	}
	void right_DS_callback(const std_msgs::msg::Bool::SharedPtr msg)
	{
		right_dswitch_last = right_dswitch;
		right_dswitch = msg->data;
		grip_R_stamp = std::chrono::steady_clock::now();
		if (!right_dswitch)
		{
			right_hooked = false;
		}
		// RCLCPP_INFO(rclcpp::get_logger("pilot_arm_node"), "Right DSwitch: %s", right_dswitch ? "ON" : "OFF");
	}

	void pin_init()
	{
		const std::string package_name = "marvin_teleop";
		const std::string package_share_directory = ament_index_cpp::get_package_share_directory(package_name);
		const std::string mjcf_path = package_share_directory + "/mjcf/";
		const std::string urdf_filename = urdf_file;
		const std::string srdf_filename = srdf_file;
		const std::string mjcf_filename = mjcf_file;

		RCLCPP_INFO(this->get_logger(), "URDF: %s", urdf_filename.c_str());
		RCLCPP_INFO(this->get_logger(), "SRDF: %s", srdf_filename.c_str());
		RCLCPP_INFO(this->get_logger(), "MJCF: %s", mjcf_filename.c_str());

		// Load Pinocchio model
		pinocchio::mjcf::buildModel(mjcf_filename, model_,true);
		std::cout << "model.nq = " << model_.nq << std::endl;
		std::cout << "model.nv = " << model_.nv << std::endl;
		data_ = pinocchio::Data(model_);

		// pinocchio::mjcf::buildGeom(
		// 	model_, mjcf_file, pinocchio::COLLISION, geom_model_, mjcf_path);

		ee_idA = model_.getFrameId(ee_nameA);
		ee_idB = model_.getFrameId(ee_nameB);
		std::cout<<"ee_idA: "<<ee_idA<<", ee_idB: "<<ee_idB<<std::endl;
		q_ = model_.referenceConfigurations["home"];
		v_ = pinocchio::Model::TangentVectorType::Zero(model_.nv);


		pinocchio::forwardKinematics(model_, data_, q_);
		pinocchio::updateFramePlacements(model_, data_);
	}

	void process_ik_request(){
		pinocchio::SE3 pg_incrementA, pg_incrementB;
		if (use_incremental_control_) {
			// Incremental mode: apply delta from hook point onto robot EEF pose at hook time
			Eigen::Vector3d target_pos_A =
				init_robot_pos_A + (pgRaw_L.translation() - init_human_pos_L);
			Eigen::Matrix3d target_rot_A =
				(pgRaw_L.rotation() * init_human_rot_L.transpose()) * init_robot_rot_A;
			pg_incrementA = pinocchio::SE3(target_rot_A, target_pos_A);

			Eigen::Vector3d target_pos_B =
				init_robot_pos_B + (pgRaw_R.translation() - init_human_pos_R);
			Eigen::Matrix3d target_rot_B =
				(pgRaw_R.rotation() * init_human_rot_R.transpose()) * init_robot_rot_B;
			pg_incrementB = pinocchio::SE3(target_rot_B, target_pos_B);
		} else {
			// Absolute mode: use raw controller pose directly
			pg_incrementA = pgRaw_L;
			pg_incrementB = pgRaw_R;
		}
		marvin_msgs::msg::IKRequest IKRequest_msg;
		IKRequest_msg.header.stamp = this->now();
		IKRequest_msg.available = {false, false, false, false, false, false};
		IKRequest_msg.left_hand_pose.position.x = pg_incrementA.translation().x();
		IKRequest_msg.left_hand_pose.position.y = pg_incrementA.translation().y();
		IKRequest_msg.left_hand_pose.position.z = pg_incrementA.translation().z();
		Eigen::Quaterniond quatA(pg_incrementA.rotation());
		IKRequest_msg.left_hand_pose.orientation.x = quatA.x();
		IKRequest_msg.left_hand_pose.orientation.y = quatA.y();
		IKRequest_msg.left_hand_pose.orientation.z = quatA.z();
		IKRequest_msg.left_hand_pose.orientation.w = quatA.w();
		IKRequest_msg.right_hand_pose.position.x = pg_incrementB.translation().x();
		IKRequest_msg.right_hand_pose.position.y = pg_incrementB.translation().y();
		IKRequest_msg.right_hand_pose.position.z = pg_incrementB.translation().z();
		Eigen::Quaterniond quatB(pg_incrementB.rotation());
		IKRequest_msg.right_hand_pose.orientation.x = quatB.x();
		IKRequest_msg.right_hand_pose.orientation.y = quatB.y();
		IKRequest_msg.right_hand_pose.orientation.z = quatB.z();
		IKRequest_msg.right_hand_pose.orientation.w = quatB.w();
		if(left_hooked){
			IKRequest_msg.available[0] = true;
		}
		if(right_hooked){
			IKRequest_msg.available[1] = true;
		}

		IKRequest_msg.joint_state.position.resize(model_.nq);
		for (size_t i = 0; i < model_.nq; ++i)
		{
			IKRequest_msg.joint_state.position[i] = q_[i];
		}
		ik_request_publisher_->publish(IKRequest_msg);

	}



	void pin_tick()
	{
		using namespace pinocchio;

		SE3 T_A_in_world, T_B_in_world;
		// eef_points_A.clear();
		// eef_points_B.clear();
		// collision_status_A.clear();
		// collision_status_B.clear();
		// int last_safe_idx_A = 0;
		// int last_safe_idx_B = 0;

		pinocchio::forwardKinematics(model_, data_, q_);
		pinocchio::updateFramePlacements(model_, data_);
		T_pgA = data_.oMf[ee_idA];
		T_pgB = data_.oMf[ee_idB];

		pub_eef_pose(T_pgA, T_pgB);
		process_ik_request();

	}

	void publishMarkerArray()
	{
		visualization_msgs::msg::MarkerArray marker_array;
		for (size_t i = 0; i < eef_points_A.size(); ++i)
		{
			visualization_msgs::msg::Marker marker;
			marker.header.frame_id = root_link_name;
			marker.header.stamp = this->now();
			marker.ns = "eef_pointsA";
			marker.id = static_cast<int>(i);
			marker.type = visualization_msgs::msg::Marker::SPHERE;
			marker.action = visualization_msgs::msg::Marker::ADD;
			marker.pose.position.x = eef_points_A[i].x();
			marker.pose.position.y = eef_points_A[i].y();
			marker.pose.position.z = eef_points_A[i].z();
			marker.scale.x = 0.02; // Diameter of the sphere
			marker.scale.y = 0.02;
			marker.scale.z = 0.02;
			if (collision_status_A[i])
			{
				marker.color.r = 1.0f; // Red color for collision
				marker.color.g = 0.0f;
				marker.color.b = 0.0f;
			}
			else
			{
				marker.color.r = 0.0f; // Green color for no collision
				marker.color.g = 1.0f;
				marker.color.b = 0.0f;
			}
			marker.color.a = 1.0;							  // Fully opaque
			marker.lifetime = rclcpp::Duration(0, 100000000); // Lifetime of the marker

			marker_array.markers.push_back(marker);
		}
		for (size_t i = 0; i < eef_points_B.size(); ++i)
		{
			visualization_msgs::msg::Marker marker;
			marker.header.frame_id = root_link_name;
			marker.header.stamp = this->now();
			marker.ns = "eef_pointsB";
			marker.id = static_cast<int>(i);
			marker.type = visualization_msgs::msg::Marker::SPHERE;
			marker.action = visualization_msgs::msg::Marker::ADD;
			marker.pose.position.x = eef_points_B[i].x();
			marker.pose.position.y = eef_points_B[i].y();
			marker.pose.position.z = eef_points_B[i].z();
			marker.scale.x = 0.02; // Diameter of the sphere
			marker.scale.y = 0.02;
			marker.scale.z = 0.02;
			if (collision_status_B[i])
			{
				marker.color.r = 1.0f; // Red color for collision
				marker.color.g = 0.0f;
				marker.color.b = 0.0f;
			}
			else
			{
				marker.color.r = 0.0f; // Green color for no collision
				marker.color.g = 1.0f;
				marker.color.b = 0.0f;
			}
			marker.color.a = 1.0;							  // Fully opaque
			marker.lifetime = rclcpp::Duration(0, 100000000); // Lifetime of the marker

			marker_array.markers.push_back(marker);
		}

		marker_pub_->publish(marker_array);
	}

	std::array<double, 3> rot_to_rpy(const Eigen::Matrix3d& R)
	{
		double sy = std::sqrt(R(0,0) * R(0,0) + R(1,0) * R(1,0));
		bool singular = sy < 1e-6;

		double roll, pitch, yaw;
		if (!singular) {
			roll  = std::atan2(R(2,1), R(2,2));
			pitch = std::atan2(-R(2,0), sy);
			yaw   = std::atan2(R(1,0), R(0,0));
		} else {
			roll  = std::atan2(-R(1,2), R(1,1));
			pitch = std::atan2(-R(2,0), sy);
			yaw   = 0.0;
		}

		return {roll, pitch, yaw};  // [roll, pitch, yaw]
	}

	std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
	std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
	rclcpp::TimerBase::SharedPtr timer_A, timer_B, timer_C;
	double pgA[4][4], pgB[4][4], pgAT[4][4], pgBT[4][4];
	pinocchio::SE3 T_pgA, T_pgB, T_pgAT, T_pgBT,pgA_offset, pgB_offset;
	// Incremental control: initial human and robot poses recorded at hook time (pos/rot separate)
	Eigen::Vector3d init_human_pos_L = Eigen::Vector3d::Zero();
	Eigen::Matrix3d init_human_rot_L = Eigen::Matrix3d::Identity();
	Eigen::Vector3d init_human_pos_R = Eigen::Vector3d::Zero();
	Eigen::Matrix3d init_human_rot_R = Eigen::Matrix3d::Identity();
	Eigen::Vector3d init_robot_pos_A = Eigen::Vector3d::Zero();
	Eigen::Matrix3d init_robot_rot_A = Eigen::Matrix3d::Identity();
	Eigen::Vector3d init_robot_pos_B = Eigen::Vector3d::Zero();
	Eigen::Matrix3d init_robot_rot_B = Eigen::Matrix3d::Identity();
	pinocchio::SE3 Elbow_left, Elbow_right;
	double joint_positions_B[7], joint_positions_BD[7];
	double joint_positions_A[7], joint_positions_AD[7];
	FX_DOUBLE ret_jointA[7], ret_jointB[7];
	FX_DOUBLE ret_jointAS[7], ret_jointBS[7];
	FX_DOUBLE ref_jointA[7], ref_jointB[7];

	FX_INT32L TYPE[2];
	FX_DOUBLE GRV[2][3];
	FX_DOUBLE DH[2][8][4];
	FX_DOUBLE PNVA[2][7][4];
	FX_DOUBLE BD[2][4][3];

	FX_DOUBLE Mass[2][7];
	FX_DOUBLE MCP[2][7][3];
	FX_DOUBLE I[2][7][6];

	double ref_dir[3];
	//
	std::string ee_nameA, ee_nameB,
		elbow_nameA, elbow_nameB,
		base_name,
		left_base_name, right_base_name,
		left_base_nameJ, right_base_nameJ,
		config_file, urdf_file, srdf_file,mjcf_file;
	pinocchio::FrameIndex ee_idA, ee_idB, elbow_idA, elbow_idB,
		left_base_id, right_base_id;
	float mpc_dt_ = 0.02; // MPC time step
	bool in_collision_ = false;
	pinocchio::Model model_, model_A, model_B;
	pinocchio::Data data_, data_A, data_B;
	pinocchio::GeometryModel geom_model_;
	pinocchio::GeometryData geom_data_;
	pinocchio::Model::ConfigVectorType q_, qAs_, qBs_;
	pinocchio::Model::TangentVectorType v_, v2_;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr collision_pub_A,
		collision_pub_B;
	bool will_coll_A, will_coll_B;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
	std::vector<Eigen::Vector3d> eef_points_A, eef_points_B;
	std::vector<bool> collision_status_A, collision_status_B;
	rclcpp::TimerBase::SharedPtr mpc_timer;

	int no_collision_itr_A = -1;
	int no_collision_itr_B = -1;
	// Pilot and PilotLmt objects are initialized in the constructor
	// Pilot the_pilot;
	// PilotLmt the_lmt;
	rclcpp::Subscription<marvin_msgs::msg::Jointfeedback>::SharedPtr joint_fb_subscriber_;
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_subscriber_;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr left_DS_subscriver_;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr right_DS_subscriver_;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr left_zerosw_subscriber_;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr right_zerosw_subscriber_;
	rclcpp::Publisher<marvin_msgs::msg::Jointcmd>::SharedPtr joint_cmd_publisherA_, joint_cmd_publisherB_, ik_publisherA_, ik_publisherB_;
	rclcpp::Publisher<marvin_msgs::msg::Jointfeedback>::SharedPtr joint_feedback_publisherA_, joint_feedback_publisherB_;
	rclcpp::Publisher<marvin_msgs::msg::IKRequest>::SharedPtr ik_request_publisher_;
	rclcpp::Subscription<marvin_msgs::msg::IKResult>::SharedPtr ik_result_subscriber_;
	rclcpp::Subscription<marvin_msgs::msg::Jointfeedback>::SharedPtr joint_feedback_subscriberA_, joint_feedback_subscriberB_;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_, pose_publisher_eefA_, pose_publisher_eefB_, pose_A_pub_, pose_B_pub_;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_subscriberA_, target_pose_subscriberB_,Elbow_pose_subscriberA_,Elbow_pose_subscriberB_;

	rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr zero_space_A_subscriber_;
	rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr zero_space_B_subscriber_;
	double zero_space_A, zero_space_B;
	std::vector<std::string> joint_namesA_;
	std::vector<std::string> joint_namesB_;
	// OneEuroSE3Filter filter_A_;
	// OneEuroSE3Filter filter_B_;
	// SE3ReferenceGenerator ref_gen_A;
	// SE3ReferenceGenerator ref_gen_B;
	// geometry_msgs::msg::TransformStamped cached_tf_b_left, cached_tf_b_right;

	bool left_dswitch = false, right_dswitch = false;
	bool left_dswitch_last = false, right_dswitch_last = false;
	bool left_hooked = false, right_hooked = false;
	// bool zero_567_A = false;
	// bool zero_567_B = false;
	bool teleop_mode = true;
	bool use_incremental_control_ = true;

	std::chrono::steady_clock::time_point grip_L_stamp, grip_R_stamp;

	pinocchio::SE3 T_left_base, T_right_base;
	std::vector<pinocchio::JointIndex> joint_to_lockA, joint_to_lockB;

	pinocchio::SE3 pgRaw_L,pgRaw_R;

	std::string root_link_name = "base_link";


};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<Pilot_arm_node>();
      std::string err;//uid for orin
      //if (!myapp::EnforceUidLicense(err)) {
	//std::cerr << "Not licensed for this machine: " << err << std::endl;
	//return 1;
     // }
	std::cout<<"license ok"<<std::endl;
	// Test6();
	// node->Test6();

	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
