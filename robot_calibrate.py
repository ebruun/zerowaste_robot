# COMPAS IMPORTS
import compas_rrc as rrc

# LOCAL IMPORTS
from src.RRC_CONNECT import connect_to_robot
from src.robot_commands import _configs_to_move
from src.io import (
    save_frame_as_matrix_yaml,
    save_image_zdf_png,
    load_config_json,
)


def calibration(abb, rob_num, pose_range):
    print("START CALIBRATION")

    for i in pose_range:
        print("\n--calibration pose #{}\n".format(i))

        # --move to pre-saved config --#
        config = load_config_json(
            "configs/calibration/R{}",
            "calibration_config_{0:0{width}}.json",
            rob_num,
            i,
        )
        _configs_to_move(abb, rob_num, config)

        abb.send(rrc.PrintText("MOVE CONFIG_{0:03} DONE, play for image".format(i)))
        abb.send_and_wait(rrc.Stop(feedback_level=rrc.FeedbackLevel.DONE))

        # --save frame and image data to be used for calibration --#
        abb.send(rrc.PrintText("Saving frame and taking image..."))
        f_at_config = abb.send_and_wait(rrc.GetFrame(), timeout=3)

        save_frame_as_matrix_yaml(
            "calibration_data", "pos{:02d}.yaml", f_at_config, i=i
        )
        save_image_zdf_png(i)

        abb.send(rrc.PrintText("IMAGE COMPLETE, press play to continue"))
        abb.send_and_wait(rrc.Stop(feedback_level=rrc.FeedbackLevel.DONE))

    print("\nCALIBRATION DONE")


if __name__ == "__main__":
    rob_num = 2
    robot, abb = connect_to_robot(rob_num)

    pose_range = range(1, 31)
    calibration(abb, rob_num, pose_range)
