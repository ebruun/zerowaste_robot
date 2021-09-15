# Before running this example, make sure to run
# "docker compose up" on the docker/moveit folder
from compas_fab.backends import RosClient
import compas_rrc as rrc

from compas.robots import Configuration

def move_by_frame(abb, speed):
    
    abb.send(rrc.PrintText('Move by frame'))
    frame = abb.send_and_wait(rrc.GetFrame(), timeout=10)

    # Change in Global Z axis (in millimeters)
    frame.point[2] += 100

    # Move robot the new pos
    return abb.send_and_wait(rrc.MoveToFrame(frame, speed, rrc.Zone.FINE, rrc.Motion.LINEAR),timeout=10)

def move_by_robtarget(abb, speed):
    abb.send(rrc.PrintText('Move by rob target'))
    frame, external_axes = abb.send_and_wait(rrc.GetRobtarget())

    # Change X value of the frame
    frame.point[0] -= 50

    # Move track forward
    external_axes[0] += 200

    # Move robot the new pos
    return abb.send_and_wait(rrc.MoveToRobtarget(frame, external_axes, speed, rrc.Zone.FINE),timeout=10)

def move_by_joints(abb, speed):
    abb.send(rrc.PrintText('Move by joints'))

    # # zero position
    # robot_joints, external_axes = [-45, 0, 0, 0, 0, 0.],  [0]

    joint_presets = {
    "ECL_parking_high": ([-90, -21, 46, 0.0, 66, -90.],[3900]),
    "ECL_parking_mid": ([-90, -46, 44, 0.0, 90, -90.],[3900]),
    "ECL_parking_low": ([-90, -66, 63, 0.0, 90, -90.],[3900]),
    "zero_position": ([0, 0, 0, 0, 0, 0.],[3900.]),
    "camera_attach": ([-90, -10, 55, 91, 90, 133.], [3900]),
    }

    robot_joints, external_axes = joint_presets["camera_attach"]

    # Move robot to start position
    return abb.send_and_wait(rrc.MoveToJoints(robot_joints, external_axes, speed, rrc.Zone.FINE),timeout=10)    


# Load robot without geometry

def robot_connect(rob_num):

    with RosClient('localhost') as ros:

        # set Robot to connect to
        abb = rrc.AbbClient(ros, rob_num)
        abb.send(rrc.PrintText("CONNECTING TO {}".format(rob_num)))
        abb.send(rrc.Stop(feedback_level=rrc.FeedbackLevel.DONE))

        # Set work object
        abb.send(rrc.SetWorkObject('rob1_acadia_EBIT_01'))
        #abb.send(rrc.SetWorkObject('ECL_platform'))
        #abb.send(rrc.SetWorkObject('wobj0'))

        # Set speed [mm/s]
        speed = 100

        # Execute command
        abb.send(rrc.PrintText('Starting...'))

        #done = move_by_frame(abb,speed)
        #done = move_by_robtarget(abb,speed)
        done = move_by_joints(abb,speed)

        # Print feedback
        print('Feedback = ', done)

        # End of Code
        print('Finished')

        # Close client
        ros.close()
        ros.terminate()

if __name__ == "__main__":
    robot_connect('/rob1')
