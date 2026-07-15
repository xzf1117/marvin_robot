#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>
#include <chrono>
#include <memory>
#include <omp.h>
#include <mutex>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "robot_env.h" // Your RobotEnv class header
#include "fabric_task_maps.h"
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "marvin_msgs/msg/jointcmd.hpp"
#include "marvin_msgs/msg/jointfeedback.hpp"
#include "marvin_msgs/srv/move_j.hpp"
// #include "marvin_msgs/msg/extforce.hpp"

using namespace Eigen;
using namespace std::chrono_literals;

struct AxisTargets
{
    std::array<Eigen::Vector3d, 2> x_axis; // [0]=p, [1]=n
    std::array<Eigen::Vector3d, 2> y_axis; // [0]=p, [1]=n
};

geometry_msgs::msg::PoseStamped transform_pose(const geometry_msgs::msg::PoseStamped &input_pose,
                                               const geometry_msgs::msg::TransformStamped &transform)
{
    geometry_msgs::msg::PoseStamped output_pose;

    // Transform the pose
    tf2::doTransform(input_pose, output_pose, transform);

    return output_pose;
}

class FabricNode : public rclcpp::Node
{
public:
    FabricNode()
        : Node("robot_env_node")
    {
        // --- Parameters ---
        // std::string mjcf_path = "/home/tianhao/fabrics/torch-fabrics/Models/marvin_pro_mink.xml";

        q_init_ = VectorXd::Zero(14);
        v_init_ = VectorXd::Ones(14) * 0.001;
        target_left = VectorXd::Zero(7);
        target_right = VectorXd::Zero(7);

        // target_left << 0.27609100341796875, 0.2650805711746216, -0.18348458409309387, 0.0, 0.0, 0.0, 1.0;
        // target_right << 0.27609100341796875, -0.2650805711746216, -0.18348458409309387, 0.0, 0.0, 0.0, 1.0;
        // target_left_home = target_left;
        // target_right_home = target_right;
        // std::cout << "target left: " << target_left.transpose() << std::endl;
        // std::cout << "target right: " << target_right.transpose() << std::endl;
        auto mjcf_name = this->declare_parameter<std::string>("mjcf_name", "marvin_pro_mink.xml");
        auto fabric_config = this->declare_parameter<std::string>("fabric_config", "robot_config.yaml");
        auto package_share_directory = ament_index_cpp::get_package_share_directory("marvin_teleop");
        auto eef_name_left = this->declare_parameter<std::string>("eef_name_left", "Link7_L");
        auto eef_name_right = this->declare_parameter<std::string>("eef_name_right", "Link7_R");
        home_joints = this->declare_parameter<std::vector<double>>("home_joints");
        if (home_joints.size() == 14)
        {
            for (size_t i = 0; i < 14; ++i)
            {
                q_init_[i] = home_joints[i];
            }
            std::cout << "Loaded home joints." <<q_init_.transpose() << std::endl;

        }
        
        std::string mjcf_path = package_share_directory + "/config/mjcf/" + mjcf_name;
        std::string mesh_path = package_share_directory + "/config/mjcf";
        std::string fabric_config_path = package_share_directory + "/config/" + fabric_config;
        RCLCPP_INFO(this->get_logger(), "Using MJCF: %s", mjcf_path.c_str());
        RCLCPP_INFO(this->get_logger(), "Using Fabric Config: %s", fabric_config.c_str());
        // --- Initialize RobotEnv ---
        int frequency = 500; // 1000 Hz
        env_ = std::make_unique<RobotEnv>(mjcf_path, mesh_path, fabric_config_path, frequency);
        env_->reset();
        eef_left_id = env_->model.getFrameId(eef_name_left);
        eef_right_id = env_->model.getFrameId(eef_name_right);
        gen_tool_direction_frames();
        VectorXd action = VectorXd::Zero(env_->model.nv);
        env_->step_sim(action);
        init_fdbk_arr();

        // --- ROS 2 publisher ---
        rclcpp::SensorDataQoS sensor_data_qos;
        // joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        eef_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("eef_pose", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("collision_spheres", 10);
        marker_pub_2_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("fabric_markers", 10);
        mode_pub_ = this->create_publisher<std_msgs::msg::Int32>("control/switch_state", 10);
        mode_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "control/switch_state", 10,
            std::bind(&FabricNode::mode_callback, this, std::placeholders::_1));
        joint_fb_subscriber_ = this->create_subscription<marvin_msgs::msg::Jointfeedback>(
            "info/joint_feedback", sensor_data_qos,
            std::bind(&FabricNode::joint_feedback_callback, this, std::placeholders::_1));

        robot_state_sub_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
            "info/arm_state", sensor_data_qos,
            std::bind(&FabricNode::robot_state_callback, this, std::placeholders::_1));

        eef_state_pub_ = this->create_publisher<std_msgs::msg::Int16MultiArray>("arm/eef_state", 10);
        pose_A_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("info/eef_left", 10);
        pose_B_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("info/eef_right", 10);
        joint_cmd_publisherA_ = this->create_publisher<marvin_msgs::msg::Jointcmd>("control/joint_cmd_plan_A", sensor_data_qos);
        joint_cmd_publisherB_ = this->create_publisher<marvin_msgs::msg::Jointcmd>("control/joint_cmd_plan_B", sensor_data_qos);
        test_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("test_pub", 10);
        movej_service = this->create_service<marvin_msgs::srv::MoveJ>(
            "control/movej",
            std::bind(&FabricNode::movej_callback, this, std::placeholders::_1, std::placeholders::_2));
        reset_arm_service = this->create_service<std_srvs::srv::Trigger>(
            "control/home_arm",
            std::bind(&FabricNode::reset_arm_service_callback, this, std::placeholders::_1, std::placeholders::_2));
        auto timer_period = 1.0 / frequency * 1s; // seconds
        timer_ = this->create_wall_timer(timer_period, std::bind(&FabricNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "✅ FabricNode initialized with %d Hz update rate.", frequency);
    }

private:
    std::vector<Eigen::MatrixXd> all_J;
    std::vector<Eigen::MatrixXd> all_metric;
    std::vector<Eigen::VectorXd> all_accel;
    std::vector<Eigen::VectorXd> all_c;
    Eigen::VectorXd target_left, target_right;
    Eigen::VectorXd target_left_home, target_right_home;
    std::map<int, Motion> sphere_vels;
    std::map<int, VectorXd> sphere_curvs;
    std::map<int, MatrixXd> sphere_jacs;
    std::vector<RobotEnv::FrameFK> frame_fks;
    int eef_left_id, eef_right_id;
    int eef_left_xp_id, eef_left_yp_id, eef_right_xp_id, eef_right_yp_id, eef_left_zp_id, eef_right_zp_id;
    int eef_left_xn_id, eef_left_yn_id, eef_right_xn_id, eef_right_yn_id, eef_left_zn_id, eef_right_zn_id;
    std::vector<std::pair<std::string, Eigen::Vector3d>> points;
    Eigen::VectorXd current_joint_positions_;
    Eigen::VectorXd current_joint_velocities_;
    Eigen::VectorXd current_joint_efforts_;
    Eigen::VectorXd ext_joint_efforts_;
    // Eigen::Vector3d eef_left_x_targetp,eef_left_x_targetn, eef_right_x_targetp,eef_right_x_targetn;
    // Eigen::Vector3d eef_left_y_targetp,eef_left_y_targetn, eef_right_y_targetp,eef_right_y_targetn;
    // AxisTargets left_axis_targets;
    // AxisTargets right_axis_targets;
    std::map<std::string, Eigen::Vector3d> eef_left_targets;
    std::map<std::string, Eigen::Vector3d> eef_right_targets;
    std::vector<float> left_constraints = {0.5, 0.5, 0.5};   // x,y,z
    std::vector<float> right_constraints = {0.5, 10.5, 0.5}; // x,y,z
    bool left_enabled = false;
    bool right_enabled = false;
    bool left_enabled_last = false;
    bool right_enabled_last = false;
    bool left_transit = false;
    bool right_transit = false;
    float max_attractor_strength = 200.0;
    float speed_scale_L = 1.0;
    float speed_scale_R = 1.0;
    bool all_ready = false;
    bool use_fabric = false;
    int switch_mode = 0; // 1: teleop, 2: fabric, 3: replay

    void robot_state_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
    {
        // Process robot state information
        int left_state, right_state;
        if (msg->data.size() >= 2)
        {
            left_state = msg->data[0];
            right_state = msg->data[1];
        }
        if (left_state == 3 && right_state == 3)
        {
            all_ready = true;
        }
        else
        {
            all_ready = false;
        }
    }

    void mode_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        switch_mode = msg->data;
    }
    void reset_arm_service_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (home_joints.size() == 14)
        {
            std::array<double, 14> home_joints_arr{};
            for (size_t i = 0; i < 14; ++i)
            {
                home_joints_arr[i] = home_joints[i];
            }
            movej(home_joints_arr);
            response->success = true;
            response->message = "Arm reset to initial position.";
        }
        else
        {
            response->success = false;
            response->message = "Invalid home_joints size.";
        }
    }

    void movej(const std::array<double, 14> &joint_values)
    {
        if(switch_mode==0){
        if (joint_values.size() == 14)
        {
            for (size_t i = 0; i < 14; ++i)
            {
                q_init_[i] = joint_values[i];
            }
            mode_pub_->publish(std_msgs::msg::Int32().set__data(2)); // switch to joint mode
            std::cout << "Home joints: " << q_init_.transpose() << std::endl;
            homming_in_progress = true;
        }
        }
    }

    void movej_callback(
        const std::shared_ptr<marvin_msgs::srv::MoveJ::Request> request,
        std::shared_ptr<marvin_msgs::srv::MoveJ::Response> response)
    {
        // Move to specified joint positions
        if (request->joint_values.size() == 14)
        {
            movej(request->joint_values);
            response->success = true;
        }
        else
        {
            response->success = false;
        }
    }

    void joint_feedback_callback(
        const marvin_msgs::msg::Jointfeedback::SharedPtr msg)
    {
        current_joint_positions_.resize(msg->positions.size());
        current_joint_velocities_.resize(msg->velocities.size());
        current_joint_efforts_.resize(msg->efforts.size());
        for (size_t i = 0; i < msg->positions.size(); ++i)
        {
            current_joint_positions_[i] = msg->positions[i];
        }
        for (size_t i = 0; i < msg->velocities.size(); ++i)
        {
            current_joint_velocities_[i] = msg->velocities[i];
        }
        for (size_t i = 0; i < msg->efforts.size(); ++i)
        {
            current_joint_efforts_[i] = msg->efforts[i];
        }
    }

    void left_cmd_callback(const marvin_msgs::msg::Jointcmd::SharedPtr msg)
    {
        for (size_t i = 0; i < 7; ++i)
        {

            left_ik_ref[i] = msg->positions[i];
        }
    }

    void right_cmd_callback(const marvin_msgs::msg::Jointcmd::SharedPtr msg)
    {
        for (size_t i = 0; i < 7; ++i)
        {

            right_ik_ref[i] = msg->positions[i];
        }
    }

    void init_fdbk_arr()
    {
        current_joint_positions_.resize(14);
        current_joint_velocities_.resize(14);
        current_joint_efforts_.resize(14);
        ext_joint_efforts_.resize(14);
        for (size_t i = 0; i < 14; ++i)
        {
            current_joint_positions_[i] = 0.0;
            current_joint_velocities_[i] = 0.0;
            current_joint_efforts_[i] = 0.0;
            ext_joint_efforts_[i] = 0.0;
        }
    }
    
    void eef_constraint_cbk(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
    {
        if (msg->data.size() >= 6)
        {
            // First element for left eef, second for right eef
            // left_constraints.assign(msg->data.begin(), msg->data.begin() + 3);
            // right_constraints.assign(msg->data.begin() + 3, msg->data.begin() + 6);
            for (size_t i = 0; i < 3; ++i)
            {
                left_constraints[i] = std::min(std::max(static_cast<float>(msg->data[i]) / 100.0f, 0.0f), 1.0f);
                right_constraints[i] = std::min(std::max(static_cast<float>(msg->data[i + 3]) / 100.0f, 0.0f), 1.0f);
            }
        }
    }
    void speed_scale_cbk(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
    {
        speed_scale_L = std::min(std::max(static_cast<float>(msg->data[0]) / 100.0f, 0.0f), 1.0f);
        speed_scale_R = std::min(std::max(static_cast<float>(msg->data[1]) / 100.0f, 0.0f), 1.0f);
    }
    void left_DS_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        bool last_state = left_enabled; // store old state first
        left_enabled = msg->data;       // update to new state

        // if switch released → unhook immediately
        if (!left_enabled)
        {
            // left_transit = true;
            // left_reached = false;
            return;
        }

        // if switch changed from false → true → hook it
        if (!last_state && left_enabled)
        {
            left_transit = true;

            {
                // Optionally:
                // RCLCPP_INFO(rclcpp::get_logger("pilot_arm_node"), "Left hooked!");
            }
        }
    }

    void right_DS_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        bool last_state = right_enabled; // store old state first
        right_enabled = msg->data;       // update to new state

        // if switch released → unhook immediately
        if (!right_enabled)
        {
            // right_transit = false;
            // right_reached = false;
            return;
        }

        // if switch changed from false → true → hook it
        if (!last_state && right_enabled)
        {
            right_transit = true;

            {
                // Optionally:
                // RCLCPP_INFO(rclcpp::get_logger("pilot_arm_node"), "Right hooked!");
            }
        }
    }
    void set_eef_state()
    {
        std_msgs::msg::Int16MultiArray eef_state_msg;
        eef_state_msg.data.resize(2);

        auto left_vel = frame_fks[eef_left_id].v;
        double v_norm = left_vel.linear().norm();
        double w_norm = left_vel.angular().norm();
        if (v_norm < stop_thres_linear && w_norm < stop_thres_angular && left_tcp_err < 0.02)
        {
            eef_state_msg.data[0] = 0;
        }
        else
        {
            eef_state_msg.data[0] = 1;
        }

        auto right_vel = frame_fks[eef_right_id].v;
        v_norm = right_vel.linear().norm();
        w_norm = right_vel.angular().norm();
        if (v_norm < stop_thres_linear && w_norm < stop_thres_angular && right_tcp_err < 0.02)
        {
            eef_state_msg.data[1] = 0;
        }
        else
        {
            eef_state_msg.data[1] = 1;
        }
        eef_state_pub_->publish(eef_state_msg);
        // std::cout << "Left EEF velocity norm: " << v_norm << std::endl;
        // std::cout << "Left EEF angular velocity norm: " << w_norm << std::endl;
    }
    void target_l_cbk(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {

        // If the left deadman switch is pressed
        // if (left_transit)
        {

            target_left[0] = msg->pose.position.x;
            target_left[1] = msg->pose.position.y;
            target_left[2] = msg->pose.position.z;
            target_left[3] = msg->pose.orientation.x;
            target_left[4] = msg->pose.orientation.y;
            target_left[5] = msg->pose.orientation.z;
            target_left[6] = msg->pose.orientation.w;
        }
    }
    void target_r_cbk(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // If the right deadman switch is pressed
        // if (right_transit)
        {

            target_right[0] = msg->pose.position.x;
            target_right[1] = msg->pose.position.y;
            target_right[2] = msg->pose.position.z;
            target_right[3] = msg->pose.orientation.x;
            target_right[4] = msg->pose.orientation.y;
            target_right[5] = msg->pose.orientation.z;
            target_right[6] = msg->pose.orientation.w;
        }
    }
    void timer_callback()
    {
        // Step simulation

        if (homming_in_progress)
        {
            // for (size_t i = 0; i < 7; ++i)
            // {
            //     target_left[i] = target_left_home[i];
            //     target_right[i] = target_right_home[i];
            // }
            // std::cout<<"target q_init_: "<<q_init_.transpose()<<std::endl;
            env_->default_pose = q_init_;
            left_constraints = {0.5, 0.5, 0.5};
            right_constraints = {0.5, 10.5, 0.5};
            frame_fks = env_->get_fk();
            set_eef_state();
            update_target_direction_frames();
            // env_->get_sphere_vel(frame_fks, sphere_vels, sphere_curvs, sphere_jacs);
            // marker_pub(this, env_->geom_model_, env_->geom_data_);
            std::vector<Eigen::Vector3d> centers;
            for (size_t i = 0; i < env_->sphere_num; ++i)
            {
                centers.push_back(frame_fks[env_->sphere_to_frame_indices[i]].oMf.translation());
            }
            marker_pub(marker_pub_, centers, env_->sphere_radii);
            // eef_pub_->publish(eef_msg);

            c_space_fabric();
            // t_space_fabric();
            collision_fabric();
            // floor_collision_fabric();
            MatrixXd M_r;
            VectorXd f_r;
            // auto time_start = std::chrono::high_resolution_clock::now();
            compute_combined_fabric_multi(
                all_J,
                all_metric,
                all_accel,
                M_r,
                f_r);

            Eigen::MatrixXd M_r_inv = M_r.completeOrthogonalDecomposition().pseudoInverse();
            VectorXd accel = M_r_inv * f_r;
            Eigen::VectorXd tau_ext(env_->model.nv); // 你已知的每个关节外力
            pinocchio::computeMinverse(env_->model, env_->data, env_->q);
            VectorXd action = accel; // Combine fabric acceleration with external forces

            VectorXd joint_diff(14);
            for (size_t i = 0; i < 14; ++i)
            {
                joint_diff[i] = env_->q[i] - q_init_[i];
            }

            // std::cout << "Joint Position Difference: \n"
            //           << joint_diff.transpose() << std::endl;
            // if first 7 smaller than 0.1 left arm reached
            // 2. Check all 7 joints (or use joint_diff_left.size())
            both_reached = true;
            for (int i = 0; i < joint_diff.size(); ++i)
            {
                // 3. Use std::abs to catch errors in both directions
                if (std::abs(joint_diff[i]) > 0.05||std::abs(env_->v[i])>0.1)
                {
                    both_reached = false;
                    break; // Stop looking; one joint is still too far
                }
            }
            VectorXd previous_q = env_->q;
            for (size_t i = 0; i < 7; ++i)
            {
                env_->default_pose[i] = left_ik_ref[i];
                env_->default_pose[i + 7] = right_ik_ref[i];
            }
            env_->step_sim(action);
            if(both_reached){
                homming_in_progress = false;
                std::cout << "Homing completed." << std::endl;
                mode_pub_->publish(std_msgs::msg::Int32().set__data(0)); // switch to teleop mode
            }

            pub_joint_cmdAB(env_->q);
            clear_data();
        }
        else
        {
            for (size_t i = 0; i < 14; ++i)
            {
                env_->q[i] = current_joint_positions_[i];
            }
        }

        // std::cout << "Left Reached: " << (left_reached ? "TRUE" : "FALSE") << std::endl;
    }

    void clear_data()
    {
        all_J.clear();
        all_metric.clear();
        all_accel.clear();
        all_c.clear();
    }

    void c_space_fabric()
    {
        Eigen::VectorXd x_upper = upper_joint_limit_task_map(env_->q, env_->max_pos_limit);
        Eigen::VectorXd x_lower = lower_joint_limit_task_map(env_->q, env_->min_pos_limit);
        Eigen::VectorXd x_default = default_config_task_map(env_->q, env_->default_pose);
        Eigen::MatrixXd jacb_upper = upper_joint_limit_jacobian(env_->q);
        Eigen::MatrixXd jacb_lower = lower_joint_limit_jacobian(env_->q);
        Eigen::MatrixXd jacb_default = default_config_jacobian(env_->q);
        Eigen::MatrixXd Metrics_lower, Metrics_upper, Metrics_default, Metrics_damping;
        Eigen::VectorXd Accels_lower, Accels_upper, Accels_default, Accels_damping;
        Eigen::VectorXd x_dot_upper = jacb_upper * env_->v;
        Eigen::VectorXd x_dot_lower = jacb_lower * env_->v;
        joint_limit_fabric(x_upper, x_dot_upper, Metrics_upper, Accels_upper);
        joint_limit_fabric(x_lower, x_dot_lower, Metrics_lower, Accels_lower);
        joint_damping_fabric(env_->v, Metrics_damping, Accels_damping);
        default_config_fabric(x_default, Metrics_default, Accels_default);

        all_J.push_back(jacb_upper);
        all_metric.push_back(Metrics_upper);
        all_accel.push_back(Accels_upper);
        all_c.push_back(Eigen::VectorXd::Zero(env_->model.nv));
        //
        all_J.push_back(jacb_lower);
        all_metric.push_back(Metrics_lower);
        all_accel.push_back(Accels_lower);
        all_c.push_back(Eigen::VectorXd::Zero(env_->model.nv));

        all_J.push_back(jacb_default);
        all_metric.push_back(Metrics_default);
        all_accel.push_back(Accels_default);
        all_c.push_back(Eigen::VectorXd::Zero(env_->model.nv));

        all_J.push_back(Eigen::MatrixXd::Identity(env_->model.nv, env_->model.nv));
        all_metric.push_back(Metrics_damping);
        all_accel.push_back(Accels_damping);
        all_c.push_back(Eigen::VectorXd::Zero(env_->model.nv));
    }

    void t_space_fabric()
    {
        // Placeholder for task space fabric computation
        // std::cout << "eef_ids: " << eef_left_id << ", " << eef_right_id << std::endl;
        Eigen::Vector3d eef_left_pos = frame_fks[eef_left_id].oMf.translation();
        Eigen::VectorXd x_left_attractor = attractor_task_map(eef_left_pos, target_left);
        left_tcp_err = x_left_attractor.norm();
        Eigen::MatrixXd j_x = attractor_task_jacobian(eef_left_pos);
        Eigen::VectorXd v_left = frame_fks[eef_left_id].v.linear();
        Eigen::VectorXd x_dot_left = j_x * v_left;
        Eigen::MatrixXd Metrics_left;
        Eigen::VectorXd Accels_left;
        Eigen::VectorXd c_left;
        attractor_task_fabric(x_left_attractor, x_dot_left, Metrics_left, Accels_left, speed_scale_L * max_attractor_strength);
        Eigen::MatrixXd J = j_x * frame_fks[eef_left_id].J.block(0, 0, 3, env_->model.nv);
        c_left = j_x * frame_fks[eef_left_id].c.head<3>();
        all_J.push_back(J);
        all_metric.push_back(Metrics_left);
        all_accel.push_back(Accels_left);
        all_c.push_back(c_left);

        Eigen::Vector3d eef_right_pos = frame_fks[eef_right_id].oMf.translation();
        Eigen::VectorXd x_right_attractor = attractor_task_map(eef_right_pos, target_right);
        right_tcp_err = x_right_attractor.norm();
        Eigen::MatrixXd jx_right = attractor_task_jacobian(eef_right_pos);
        Eigen::VectorXd v_right = frame_fks[eef_right_id].v.linear();
        Eigen::VectorXd x_dot_right = jx_right * v_right;
        Eigen::MatrixXd Metrics_right;
        Eigen::VectorXd Accels_right;
        Eigen::VectorXd c_right;
        attractor_task_fabric(x_right_attractor, x_dot_right, Metrics_right, Accels_right, speed_scale_R * max_attractor_strength);
        Eigen::MatrixXd J_right = jx_right * frame_fks[eef_right_id].J.block(0, 0, 3, env_->model.nv);
        c_right = jx_right * frame_fks[eef_right_id].c.head<3>();
        all_J.push_back(J_right);
        all_metric.push_back(Metrics_right);
        all_accel.push_back(Accels_right);
        all_c.push_back(c_right);

        // // 定义所有 frame id（建议初始化时就完成）
        // std::map<std::string, int> eef_left_frame_ids = {
        //     {"xp", eef_left_xp_id},
        //     {"xn", eef_left_xn_id},
        //     {"yp", eef_left_yp_id},
        //     {"yn", eef_left_yn_id},
        //     {"zp", eef_left_zp_id},
        //     {"zn", eef_left_zn_id},
        // };
        // std::map<std::string, int> eef_right_frame_ids = {
        //     {"xp", eef_right_xp_id},
        //     {"xn", eef_right_xn_id},
        //     {"yp", eef_right_yp_id},
        //     {"yn", eef_right_yn_id},
        //     {"zp", eef_right_zp_id},
        //     {"zn", eef_right_zn_id},
        // };

        // // 遍历方向 ±
        // std::vector<std::string> dirs_L_temp = {"xp", "xn", "yp", "yn", "zp", "zn"};
        // std::vector<std::string> dirs_R_temp = {"xp", "xn", "yp", "yn", "zp", "zn"};
        // std::vector<std::string> dirs_L = {"xp", "xn", "yp", "yn", "zp", "zn"};
        // std::vector<std::string> dirs_R = {"xp", "xn", "yp", "yn", "zp", "zn"};

        // for (int i = 0; i < 3; ++i)
        // {
        //     if (left_constraints[i] == 1)
        //     {
        //         dirs_L.push_back(dirs_L_temp[i * 2]);     // positive direction
        //         dirs_L.push_back(dirs_L_temp[i * 2 + 1]); // negative direction
        //     }
        //     if (right_constraints[i] == 1)
        //     {
        //         dirs_R.push_back(dirs_R_temp[i * 2]);     // positive direction
        //         dirs_R.push_back(dirs_R_temp[i * 2 + 1]); // negative direction
        //     }
        // }

        // for (const auto &dir : dirs_L)
        // {
        //     int frame_id = eef_left_frame_ids[dir];
        //     Eigen::Vector3d target = eef_left_targets[dir]; // std::map<std::string, Eigen::Vector3d>

        //     Eigen::Vector3d eef_pos = frame_fks[frame_id].oMf.translation();
        //     Eigen::VectorXd x_left_dir = attractor_task_map(eef_pos, target);
        //     Eigen::MatrixXd j_x_dir = attractor_task_jacobian(x_left_dir);
        //     Eigen::VectorXd v_left_dir = frame_fks[frame_id].v.linear();
        //     Eigen::VectorXd x_dot_left_dir = j_x_dir * v_left_dir;

        //     Eigen::MatrixXd Metrics_dir_L;
        //     Eigen::VectorXd Accels_dir_L;
        //     Eigen::VectorXd c_left_dir;

        //     dir_task_fabric(x_left_dir, x_dot_left_dir, Metrics_dir_L, Accels_dir_L, speed_scale_L * max_attractor_strength);

        //     Eigen::MatrixXd J_dir_L = j_x_dir * frame_fks[frame_id].J.block(0, 0, 3, env_->model.nv);
        //     c_left_dir = j_x_dir * frame_fks[frame_id].c.head<3>();

        //     all_J.push_back(J_dir_L);
        //     all_metric.push_back(Metrics_dir_L);
        //     all_accel.push_back(Accels_dir_L);
        //     all_c.push_back(c_left_dir);
        // }

        // // 右臂同理

        // for (const auto &dir : dirs_R)
        // {
        //     int frame_id = eef_right_frame_ids[dir];
        //     Eigen::Vector3d target = eef_right_targets[dir]; // std::map<std::string, Eigen::Vector3d>

        //     Eigen::Vector3d eef_pos = frame_fks[frame_id].oMf.translation();
        //     Eigen::VectorXd x_right_dir = attractor_task_map(eef_pos, target);
        //     Eigen::MatrixXd j_x_dir = attractor_task_jacobian(x_right_dir);
        //     Eigen::VectorXd v_right_dir = frame_fks[frame_id].v.linear();
        //     Eigen::VectorXd x_dot_right_dir = j_x_dir * v_right_dir;

        //     Eigen::MatrixXd Metrics_dir_R;
        //     Eigen::VectorXd Accels_dir_R;
        //     Eigen::VectorXd c_right_dir;

        //     dir_task_fabric(x_right_dir, x_dot_right_dir, Metrics_dir_R, Accels_dir_R, speed_scale_R * max_attractor_strength);

        //     Eigen::MatrixXd J_dir_R = j_x_dir * frame_fks[frame_id].J.block(0, 0, 3, env_->model.nv);
        //     c_right_dir = j_x_dir * frame_fks[frame_id].c.head<3>();

        //     all_J.push_back(J_dir_R);
        //     all_metric.push_back(Metrics_dir_R);
        //     all_accel.push_back(Accels_dir_R);
        //     all_c.push_back(c_right_dir);
        // }
    }

    void gen_tool_direction_frames()
    {
        float offset_ratio = 0.4;
        // ===== 左臂 =====
        {
            const auto &eef_frame = env_->model.frames[eef_left_id];
            const auto &parent_joint_id = eef_frame.parentJoint;
            const auto &T_parentjoint_eef = eef_frame.placement;

            // 定义相对于 eef 的正负偏移
            const std::vector<std::pair<std::string, Eigen::Vector3d>> offsets = {
                {"x_pos", Eigen::Vector3d(offset_ratio, 0.0, 0.0)},
                {"x_neg", Eigen::Vector3d(-offset_ratio, 0.0, 0.0)},
                {"y_pos", Eigen::Vector3d(0.0, offset_ratio, 0.0)},
                {"y_neg", Eigen::Vector3d(0.0, -offset_ratio, 0.0)},
                {"z_pos", Eigen::Vector3d(0.0, 0.0, offset_ratio)},
                {"z_neg", Eigen::Vector3d(0.0, 0.0, -offset_ratio)}};
            // 存储 Frame ID
            std::map<std::string, pinocchio::FrameIndex> left_frame_ids;

            for (const auto &[suffix, offset] : offsets)
            {
                const pinocchio::SE3 T_eef(Eigen::Matrix3d::Identity(), offset);
                const pinocchio::SE3 T_parentjoint = T_parentjoint_eef * T_eef;

                const std::string frame_name = "Link7_L_" + suffix + "_axis";
                const pinocchio::Frame new_frame(
                    frame_name,
                    parent_joint_id,
                    eef_left_id,
                    T_parentjoint,
                    pinocchio::FrameType::OP_FRAME);

                left_frame_ids[suffix] = env_->model.addFrame(new_frame);
            }

            // 如果你需要保存这些ID
            eef_left_xp_id = left_frame_ids["x_pos"];
            eef_left_xn_id = left_frame_ids["x_neg"];
            eef_left_yp_id = left_frame_ids["y_pos"];
            eef_left_yn_id = left_frame_ids["y_neg"];
            eef_left_zp_id = left_frame_ids["z_pos"];
            eef_left_zn_id = left_frame_ids["z_neg"];
        }

        // === 右臂部分（可选）同理 ===
        {
            const auto &eef_frame = env_->model.frames[eef_right_id];
            const auto &parent_joint_id = eef_frame.parentJoint;
            const auto &T_parentjoint_eef = eef_frame.placement;

            const std::vector<std::pair<std::string, Eigen::Vector3d>> offsets = {
                {"x_pos", Eigen::Vector3d(offset_ratio, 0.0, 0.0)},
                {"x_neg", Eigen::Vector3d(-offset_ratio, 0.0, 0.0)},
                {"y_pos", Eigen::Vector3d(0.0, offset_ratio, 0.0)},
                {"y_neg", Eigen::Vector3d(0.0, -offset_ratio, 0.0)},
                {"z_pos", Eigen::Vector3d(0.0, 0.0, offset_ratio)},
                {"z_neg", Eigen::Vector3d(0.0, 0.0, -offset_ratio)}};

            std::map<std::string, pinocchio::FrameIndex> right_frame_ids;

            for (const auto &[suffix, offset] : offsets)
            {
                const pinocchio::SE3 T_eef(Eigen::Matrix3d::Identity(), offset);
                const pinocchio::SE3 T_parentjoint = T_parentjoint_eef * T_eef;

                const std::string frame_name = "Link7_R_" + suffix + "_axis";
                const pinocchio::Frame new_frame(
                    frame_name,
                    parent_joint_id,
                    eef_right_id,
                    T_parentjoint,
                    pinocchio::FrameType::OP_FRAME);

                right_frame_ids[suffix] = env_->model.addFrame(new_frame);
            }

            eef_right_xp_id = right_frame_ids["x_pos"];
            eef_right_xn_id = right_frame_ids["x_neg"];
            eef_right_yp_id = right_frame_ids["y_pos"];
            eef_right_yn_id = right_frame_ids["y_neg"];
            eef_right_zp_id = right_frame_ids["z_pos"];
            eef_right_zn_id = right_frame_ids["z_neg"];
        }

        // === 更新 data ===
        env_->data = pinocchio::Data(env_->model);

        // 所有端点定义
        points = {
            {"eef_left_xp", eef_left_targets["xp"]},
            {"eef_left_xn", eef_left_targets["xn"]},
            {"eef_left_yp", eef_left_targets["yp"]},
            {"eef_left_yn", eef_left_targets["yn"]},
            {"eef_left_zp", eef_left_targets["zp"]},
            {"eef_left_zn", eef_left_targets["zn"]},
            {"eef_right_xp", eef_right_targets["xp"]},
            {"eef_right_xn", eef_right_targets["xn"]},
            {"eef_right_yp", eef_right_targets["yp"]},
            {"eef_right_yn", eef_right_targets["yn"]},
            {"eef_right_zp", eef_right_targets["zp"]},
            {"eef_right_zn", eef_right_targets["zn"]}};
    }

    void update_target_direction_frames()
    {
        float offset_ratio = 0.4;
        const std::vector<std::pair<std::string, Eigen::Vector3d>> directions_L = {
            {"xp", Eigen::Vector3d(offset_ratio * left_constraints[0], 0, 0)},
            {"xn", Eigen::Vector3d(-offset_ratio * left_constraints[0], 0, 0)},
            {"yp", Eigen::Vector3d(0, offset_ratio * left_constraints[1], 0)},
            {"yn", Eigen::Vector3d(0, -offset_ratio * left_constraints[1], 0)},
            {"zp", Eigen::Vector3d(0, 0, offset_ratio * left_constraints[2])},
            {"zn", Eigen::Vector3d(0, 0, -offset_ratio * left_constraints[2])}};

        const std::vector<std::pair<std::string, Eigen::Vector3d>> directions_R = {
            {"xp", Eigen::Vector3d(offset_ratio * right_constraints[0], 0, 0)},
            {"xn", Eigen::Vector3d(-offset_ratio * right_constraints[0], 0, 0)},
            {"yp", Eigen::Vector3d(0, offset_ratio * right_constraints[1], 0)},
            {"yn", Eigen::Vector3d(0, -offset_ratio * right_constraints[1], 0)},
            {"zp", Eigen::Vector3d(0, 0, offset_ratio * right_constraints[2])},
            {"zn", Eigen::Vector3d(0, 0, -offset_ratio * right_constraints[2])}};
        // === 左臂 ===
        {
            Eigen::Quaterniond q_left(target_left[6], target_left[3], target_left[4], target_left[5]);
            q_left.normalize();
            Eigen::Matrix3d R_left = q_left.toRotationMatrix();
            Eigen::Vector3d base_left = frame_fks[eef_left_id].oMf.translation();

            for (auto &[key, dir] : directions_L)
                eef_left_targets[key] = target_left.head<3>() + R_left * dir;
            // eef_left_targets[key] =  frame_fks[eef_left_id].oMf.translation() + R_left * dir;
        }

        // === 右臂 ===
        {
            Eigen::Quaterniond q_right(target_right[6], target_right[3], target_right[4], target_right[5]);
            q_right.normalize();
            Eigen::Matrix3d R_right = q_right.toRotationMatrix();
            Eigen::Vector3d base_right = frame_fks[eef_right_id].oMf.translation();

            for (auto &[key, dir] : directions_R)
                eef_right_targets[key] = target_right.head<3>() + R_right * dir;
        }
        publish_all_eef_targets();
        // std::cout<<new_translation<<std::endl;
        // Eigen::Vector3d test_diff = eef_left_x_target - frame_fks[eef_left_x_id].oMf.translation();
        // std::cout<<"Test diff for eef x axis frame: "<<test_diff.transpose()<< std::endl;
    }
    void publish_all_eef_targets()
    {
        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker marker_template;
        marker_template.header.frame_id = "base_link";
        marker_template.header.stamp = this->now();
        marker_template.type = visualization_msgs::msg::Marker::SPHERE;
        marker_template.action = visualization_msgs::msg::Marker::ADD;
        marker_template.scale.x = 0.01;
        marker_template.scale.y = 0.01;
        marker_template.scale.z = 0.01;
        marker_template.color.a = 1.0;
        marker_template.lifetime = rclcpp::Duration(0, 200000000); // 0.2s

        int id = 0;
        for (const auto &[name, pos] : points)
        {
            visualization_msgs::msg::Marker marker = marker_template;
            marker.ns = name;
            marker.id = id++;
            marker.pose.position.x = pos[0];
            marker.pose.position.y = pos[1];
            marker.pose.position.z = pos[2];

            // 按命名规则染色
            if (name.find("left") != std::string::npos)
            {
                marker.color.r = 1.0f;
                marker.color.g = 0.0f;
                marker.color.b = 0.0f; // 红：左臂
            }
            else
            {
                marker.color.r = 0.0f;
                marker.color.g = 0.0f;
                marker.color.b = 1.0f; // 蓝：右臂
            }

            if (name.find("x") != std::string::npos)
                marker.scale.x = marker.scale.y = marker.scale.z = 0.015; // x方向稍大
            else
                marker.scale.x = marker.scale.y = marker.scale.z = 0.01;

            marker_array.markers.push_back(marker);
        }

        marker_pub_2_->publish(marker_array);
    }

    void floor_collision_fabric()
    {
        for (const auto &[link_id, pair] : env_->link_sphere_map)
        {
            const auto &link_sphere_ids = pair.first;
            for (int i : link_sphere_ids)
            {
                Eigen::Vector3d link_pos = frame_fks[env_->sphere_to_frame_indices[i]].oMf.translation();
                float floor_height = -0.35;
                double radius = env_->sphere_radii[i];

                Eigen::VectorXd x = floor_repulsion_task_map(link_pos, floor_height, radius);
                if (x[0] > 2.0) // 超过影响范围，跳过
                    continue;

                Eigen::MatrixXd jx = floor_avoidance_task_jacobian(link_pos);
                Eigen::VectorXd x_dot = jx * frame_fks[env_->sphere_to_frame_indices[i]].v.linear();

                Eigen::MatrixXd Metrics_oa;
                Eigen::VectorXd Accels_oa;
                Eigen::VectorXd c_sphere;
                floor_repulsion_fabric(x, x_dot, Metrics_oa, Accels_oa);

                Eigen::MatrixXd J = jx * frame_fks[env_->sphere_to_frame_indices[i]].J.block(0, 0, 3, env_->model.nv);
                c_sphere = jx * frame_fks[env_->sphere_to_frame_indices[i]].c.head<3>();

                all_J.push_back(J);
                all_metric.push_back(Metrics_oa);
                all_accel.push_back(Accels_oa);
                all_c.push_back(c_sphere);
            }
        }
    }
    void collision_fabric()
    {
        int coll_pairs = 0;
        for (const auto &[link_id, pair] : env_->link_sphere_map)
        {
            const auto &link_sphere_ids = pair.first;
            const auto &obs_sphere_ids = pair.second;

            if (link_sphere_ids.empty() || obs_sphere_ids.empty())
                continue;

            int n_link = link_sphere_ids.size();
            int n_obs = obs_sphere_ids.size();
            int nq = env_->model.nv; // generalized coordinates DOF

            // === Positions
            Eigen::MatrixXd link_positions(n_link, 3);
            Eigen::MatrixXd obs_positions(n_obs, 3);

            Eigen::VectorXd vi, vj, vi_j;

            for (int i : link_sphere_ids)
            {
                Eigen::MatrixXd jx, J;
                Eigen::VectorXd x;
                Eigen::MatrixXd Metrics_oa;
                Eigen::VectorXd Accels_oa;
                Eigen::VectorXd c_sphere;
                Eigen::VectorXd x_dot;
                // J_q_list.push_back(frame_fks[env_->sphere_to_frame_indices[k]].J.block(0, 0, 3, nq));

                for (int j : obs_sphere_ids)
                {
                    Eigen::Vector3d link_pos = frame_fks[env_->sphere_to_frame_indices[i]].oMf.translation();
                    Eigen::Vector3d obs_pos = frame_fks[env_->sphere_to_frame_indices[j]].oMf.translation();
                    vi = frame_fks[env_->sphere_to_frame_indices[i]].v.linear();
                    vj = frame_fks[env_->sphere_to_frame_indices[j]].v.linear();
                    vi_j = vi - vj;
                    double radius = (env_->sphere_radii[i] + env_->sphere_radii[j]) / 2.0;

                    x = obstacle_avoidance_task_map(
                        link_pos,
                        obs_pos,
                        radius);
                    if (x[0] > 2.0) // 超过影响范围，跳过
                        continue;
                    jx = obstacle_avoidance_task_jacobian(
                        link_pos,
                        obs_pos,
                        radius);
                    x_dot = jx * vi_j;

                    J = jx * frame_fks[env_->sphere_to_frame_indices[i]].J.block(0, 0, 3, env_->model.nv);
                    c_sphere = jx * (frame_fks[env_->sphere_to_frame_indices[i]].c.head<3>() - frame_fks[env_->sphere_to_frame_indices[j]].c.head<3>());

                    obstacle_avoidance_fabric(
                        x,
                        x_dot,
                        Metrics_oa,
                        Accels_oa);
                    all_J.push_back(J);
                    all_metric.push_back(Metrics_oa);
                    all_accel.push_back(Accels_oa);
                    all_c.push_back(c_sphere);
                    coll_pairs += 1;
                }
            }
        }
        // std::cout << "Total collision sphere pairs processed: " << coll_pairs << std::endl;
    }

    void compute_combined_fabric_multi(
        const std::vector<MatrixXd> &all_J,
        const std::vector<MatrixXd> &all_metric,
        const std::vector<VectorXd> &all_accel,
        MatrixXd &M_r,
        VectorXd &f_r)
    {
        int n = all_J[0].cols(); // joint维度
        M_r = MatrixXd::Zero(n, n);
        f_r = VectorXd::Zero(n);

        for (size_t i = 0; i < all_J.size(); ++i)
        {
            const MatrixXd &J = all_J[i];
            const MatrixXd &M = all_metric[i];
            const VectorXd &a = all_accel[i] - all_c[i];

            M_r += J.transpose() * M * J;
            f_r += J.transpose() * M * a;
        }
    }

    
    void pub_joint_cmdAB(const Eigen::VectorXd &jv)
    {
        if (jv.size() != 14)
        {
            RCLCPP_WARN(this->get_logger(), "Expected 14 joint values, got %zu", jv.size());
            return;
        }

        // Publish to A
        auto joint_state_msgA = marvin_msgs::msg::Jointcmd();
        joint_state_msgA.header.stamp = this->now();
        joint_state_msgA.header.frame_id = "base_link";

        for (size_t i = 0; i < 7; ++i)
        {
            joint_state_msgA.positions[i] = jv[i]; // radians to degrees
        }
        joint_cmd_publisherA_->publish(joint_state_msgA);

        // Publish to B
        auto joint_state_msgB = marvin_msgs::msg::Jointcmd();
        joint_state_msgB.header.stamp = this->now();
        joint_state_msgB.header.frame_id = "base_link";

        for (size_t i = 0; i < 7; ++i)
        {
            joint_state_msgB.positions[i] = jv[i + 7]; // radians to degrees
        }
        joint_cmd_publisherB_->publish(joint_state_msgB);
    }
    void pub_joint_cmdA(const Eigen::VectorXd &jv)
    {
        if (jv.size() != 14)
        {
            RCLCPP_WARN(this->get_logger(), "Expected 14 joint values, got %zu", jv.size());
            return;
        }

        // Publish to A
        auto joint_state_msgA = marvin_msgs::msg::Jointcmd();
        joint_state_msgA.header.stamp = this->now();
        joint_state_msgA.header.frame_id = "base_link";

        for (size_t i = 0; i < 7; ++i)
        {
            joint_state_msgA.positions[i] = jv[i]; // radians to degrees
        }
        joint_cmd_publisherA_->publish(joint_state_msgA);
    }
    void pub_joint_cmdB(const Eigen::VectorXd &jv)
    {
        if (jv.size() != 14)
        {
            RCLCPP_WARN(this->get_logger(), "Expected 14 joint values, got %zu", jv.size());
            return;
        }

        // Publish to B
        auto joint_state_msgB = marvin_msgs::msg::Jointcmd();
        joint_state_msgB.header.stamp = this->now();
        joint_state_msgB.header.frame_id = "base_link";

        for (size_t i = 0; i < 7; ++i)
        {
            joint_state_msgB.positions[i] = jv[i + 7]; // radians to degrees
        }
        joint_cmd_publisherB_->publish(joint_state_msgB);
    }
    void marker_pub(
        const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr &pub,
        const std::vector<Eigen::Vector3d> &centers,
        const std::vector<double> &radii)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        int id = 0;

        for (size_t i = 0; i < centers.size(); ++i)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "base_link"; // change if needed
            marker.header.stamp = this->get_clock()->now();
            marker.ns = "collision_spheres";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            // Position
            marker.pose.position.x = centers[i].x();
            marker.pose.position.y = centers[i].y();
            marker.pose.position.z = centers[i].z();
            marker.pose.orientation.w = 1.0;

            // Scale (diameter)
            double r = radii[i];
            marker.scale.x = 2.0 * r;
            marker.scale.y = 2.0 * r;
            marker.scale.z = 2.0 * r;

            // Color (light green)
            marker.color.r = 0.1f;
            marker.color.g = 0.8f;
            marker.color.b = 0.2f;
            marker.color.a = 0.6f;

            marker.lifetime = rclcpp::Duration(0, 0); // persist

            marker_array.markers.push_back(marker);
        }

        pub->publish(marker_array);
    }
    // --- Members ---
    std::unique_ptr<RobotEnv> env_;
    VectorXd q_init_, v_init_;
    std::vector<double> home_joints;
    std::vector<double> acc_limit_ = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5,
                                      0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5}; // 14 DOF
    double stop_thres_linear = 0.01;
    double stop_thres_angular = 0.05;
    double left_tcp_err = 1.0, right_tcp_err = 1.0;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr eef_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_L_, target_sub_R_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr left_DS_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr right_DS_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fabric_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr robot_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_arm_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr eef_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr mode_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_2_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_3_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr eef_constraint_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr speed_scale_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_arm_service;
    rclcpp::Service<marvin_msgs::srv::MoveJ>::SharedPtr movej_service;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_A_pub_, pose_B_pub_;
    rclcpp::Publisher<marvin_msgs::msg::Jointcmd>::SharedPtr joint_cmd_publisherA_, joint_cmd_publisherB_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr test_pub_;
    rclcpp::Subscription<marvin_msgs::msg::Jointfeedback>::SharedPtr joint_fb_subscriber_;
    rclcpp::Subscription<marvin_msgs::msg::Jointcmd>::SharedPtr joint_tgt_sub_A_, joint_tgt_sub_B_;
    std::array<double, 7> left_joint_cmd, right_joint_cmd;
    std::array<double, 7> left_ik_ref, right_ik_ref;
    // rclcpp::Subscription<marvin_msgs::msg::Extforce>::SharedPtr ext_force_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool both_reached = true;
    bool left_reached = true;
    bool right_reached = true;
    bool useFabric = true;
    bool homming_in_progress = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FabricNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
