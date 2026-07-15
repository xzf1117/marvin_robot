import os
import mujoco
import mujoco.viewer
import mink
import numpy as np
from ament_index_python.packages import get_package_share_directory

package_path = get_package_share_directory('marvin_teleop')
xml = os.path.join(package_path, 'config', 'mjcf', 'teleop_scene_pro.xml')

def main():
    model = mujoco.MjModel.from_xml_path(xml)
    configuration = mink.Configuration(model)
    configuration.update_from_keyframe("home")
    model = configuration.model
    data = configuration.data

    hands = ["left_tool_site", "right_tool_site"]
    mocap_names = ["left_tool_target", "right_tool_target"]  # mocap body names in XML

    posture_task = mink.PostureTask(model, cost=0.05)
    posture_task.set_target_from_configuration(configuration)
    damping_task = mink.DampingTask(model, cost=0.1)

    hand_tasks = [
        mink.FrameTask(
            frame_name=hand,
            frame_type="site",
            position_cost=1.0,
            orientation_cost=0.1,
            lm_damping=1.0,
        )
        for hand in hands
    ]

    mujoco.mj_kinematics(model, data)
    for hand_task in hand_tasks:
        hand_task.set_target_from_configuration(configuration)

    tasks = [posture_task, damping_task] + hand_tasks

    collision_pairs = [
        (["Link4_L_collision", "Link4_R_collision"], ["base_link_collision"]),
        (["Link5_L_collision", "Link5_R_collision"], ["base_link_collision"]),
    ]
    limits = [
        mink.ConfigurationLimit(model),
        mink.CollisionAvoidanceLimit(
            model=model,
            geom_pairs=collision_pairs,
            minimum_distance_from_collisions=0.005,
            collision_detection_distance=0.15,
        ),
        mink.VelocityLimit(model, {
            "Joint1_L": 1.5*np.pi, "Joint2_L": 2.5*np.pi, "Joint3_L": 2.5*np.pi,
            "Joint4_L": 2.5*np.pi, "Joint5_L": 2.5*np.pi, "Joint6_L": 2.5*np.pi,
            "Joint1_R": 1.5*np.pi, "Joint2_R": 2.5*np.pi, "Joint3_R": 2.5*np.pi,
            "Joint4_R": 2.5*np.pi, "Joint5_R": 2.5*np.pi, "Joint6_R": 2.5*np.pi,
        }),
    ]

    # Get mocap IDs once
    mocap_ids = [model.body(name).mocapid[0] for name in mocap_names]

    # Initialize mocap bodies at the current site poses
    for i, hand in enumerate(hands):
        mink.move_mocap_to_frame(model, data, mocap_names[i], hand, "site")

    solver = "daqp"
    dt = 1.0 / 200.0

    from loop_rate_limiters import RateLimiter
    rate = RateLimiter(frequency=200.0, warn=False)

    with mujoco.viewer.launch_passive(
        model=model, data=data,
        show_left_ui=False, show_right_ui=False
    ) as viewer:
        mujoco.mjv_defaultFreeCamera(model, viewer.cam)

        while viewer.is_running():
            # Update hand task targets from mocap body poses
            for i, task in enumerate(hand_tasks):
                task.set_target(mink.SE3.from_mocap_id(data, mocap_ids[i]))

            vel = mink.solve_ik(configuration, tasks, dt, solver,
                                damping=1e-1, limits=limits)
            configuration.integrate_inplace(vel, dt)

            mujoco.mj_camlight(model, data)
            mujoco.mj_fwdPosition(model, data)
            mujoco.mj_sensorPos(model, data)
            viewer.sync()
            rate.sleep()


if __name__ == '__main__':
    main()