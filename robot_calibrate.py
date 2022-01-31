# COMPAS IMPORTS
import compas_rrc as rrc

# LOCAL IMPORTS
from src.RRC_CONNECT import connect_to_robot
from src.io import save_frame_as_matrix_yaml, save_image_zdf_png, load_config_json


def _move_to_config(rob_num, abb, configs):
    abb.send(rrc.PrintText("Moving..."))
    speed = 100

    for config in configs:
        if rob_num == 1:
            axis = rrc.ExternalAxes(config['r1_cart_joint'])
            joints = rrc.RobotJoints(
                config['r1_joint_1'],
                config['r1_joint_2'],
                config['r1_joint_3'],
                config['r1_joint_4'],
                config['r1_joint_5'],
                config['r1_joint_6'],
            )
        elif rob_num == 2:
            axis = rrc.ExternalAxes(config['r2_cart_joint'])
            joints = rrc.RobotJoints(
                config['r2_joint_1'],
                config['r2_joint_2'],
                config['r2_joint_3'],
                config['r2_joint_4'],
                config['r2_joint_5'],
                config['r2_joint_6'],
            )

        abb.send(rrc.MoveToJoints(joints, axis, speed, rrc.Zone.FINE))


def calibration(abb, rob_num, pose_range):
    print("START CALIBRATION")

    for i in pose_range:
        print('\n--calibration pose #{}\n'.format(i))

        # --move to pre-saved config --#
        config = load_config_json(
            "calibration_configs/calibration_configs_R{}",
            "calibration_config_{0:0{width}}.json",
            rob_num, i)
        _move_to_config(rob_num, abb, config)

        abb.send(rrc.PrintText("MOVE TO CONFIG_{0:0{width}} COMPLETE, play to take image".format(i, width=3)))
        abb.send_and_wait(rrc.Stop(feedback_level=rrc.FeedbackLevel.DONE))

        # --save frame and image data to be used for calibration --#
        abb.send(rrc.PrintText("Saving frame and taking image..."))
        f_at_config = abb.send_and_wait(rrc.GetFrame(), timeout=3)

        save_frame_as_matrix_yaml("calibration_data", "pos{:02d}.yaml", f_at_config, i=i)
        save_image_zdf_png(i)

        abb.send(rrc.PrintText("IMAGE COMPLETE, press play to continue"))
        abb.send_and_wait(rrc.Stop(feedback_level=rrc.FeedbackLevel.DONE))

    print("\nCALIBRATION DONE")


if __name__ == "__main__":
    rob_num = 2
    robot, abb = connect_to_robot(rob_num)

    pose_range = range(1, 31)
    calibration(abb, rob_num, pose_range)
