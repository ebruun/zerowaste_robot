# Before running this example, make sure to run
# "docker compose up" on the docker/moveit folder
from compas_fab.backends import RosClient
import compas_rrc as rrc

from compas.robots import Configuration

# Load robot without geometry
with RosClient('localhost') as ros:
    robot = ros.load_robot(load_geometry=False)

    abb = rrc.AbbClient(ros, '/rob1')
    print('Connected.')

    #abb.send(rrc.SetWorkObject('rob1_acadia_EBIT_01'))
    #abb.send(rrc.SetWorkObject('ECL_platform'))
    abb.send(rrc.SetWorkObject('wobj0'))

    # Set speed [mm/s]
    speed = 100

    abb.send(rrc.PrintText('Starting...'))


    # abb.send(rrc.PrintText('Move negative z'))
    frame = abb.send_and_wait(rrc.GetFrame(), timeout=10)

    print(frame)

    # # Change a X axis (in millimiters)
    # frame.point[2] += 100

    # # Move robot the new pos
    # done = abb.send_and_wait(rrc.MoveToFrame(frame, speed, rrc.Zone.FINE, rrc.Motion.LINEAR),timeout=10)


    # Read value of joints
    robot_joints, external_axes = [0, 0, 0, 0, 0, 0.],  [0]

    # Print received values
    print(robot_joints, external_axes)

    done = abb.send_and_wait(rrc.MoveToJoints(robot_joints, external_axes, 1000, rrc.Zone.FINE), timeout=10)
    
    # Print feedback
    print('Feedback = ', done)

    # End of Code
    print('Finished')

    # Close client
    ros.close()
    ros.terminate()
