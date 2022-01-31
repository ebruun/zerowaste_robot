# COMPAS IMPORTS
import compas_rrc as rrc


# Connect to robot
def connect_to_robot(rob_num):

    ros = rrc.RosClient()
    ros.run()
    robot = ros.load_robot()

    abb = rrc.AbbClient(ros, '/rob'+str(rob_num))
    abb.send(rrc.PrintText("CONNECTED TO {}, press play to continue...".format('/rob'+str(rob_num))))
    abb.send_and_wait(rrc.Stop(feedback_level=rrc.FeedbackLevel.DONE))

    if rob_num == 1:
        abb.send(rrc.SetWorkObject('rob1_acadia_EBIT_01'))
        # abb.send(rrc.SetWorkObject('ECL_platform'))
        # abb.send(rrc.SetWorkObject('wobj0'))
    elif rob_num == 2:
        abb.send(rrc.SetWorkObject('rob2_acadia_EBIT_01'))
        # abb.send(rrc.SetWorkObject('ECL_platform'))
        # abb.send(rrc.SetWorkObject('wobj0'))

    abb.send(rrc.PrintText('Starting...'))
    return robot, abb
