# COMPAS IMPORTS
import compas_rrc as rrc


# Connect to robot
def _connect_to_single_robot(rob_num):
    """connect to a single robot"""

    ros = rrc.RosClient()
    ros.run()
    robot = ros.load_robot()

    abb = rrc.AbbClient(ros, "/rob" + str(rob_num))

    if rob_num == 1:
        abb.send(rrc.SetWorkObject("rob1_acadia_EBIT_01"))
        abb.send(rrc.SetTool("ecl_pneumatic_gripper"))
        # abb.send(rrc.SetMaxSpeed(override=100, max_tcp=10000))
        # abb.send(rrc.SetWorkObject('ECL_platform'))
        # abb.send(rrc.SetWorkObject('wobj0'))
    elif rob_num == 2:
        abb.send(rrc.SetWorkObject("rob2_acadia_EBIT_01"))
        abb.send(rrc.SetTool("ecl_pneumatic_gripper"))
        # abb.send(rrc.SetMaxSpeed(override=100, max_tcp=10000))
        # abb.send(rrc.SetWorkObject('ECL_platform'))
        # abb.send(rrc.SetWorkObject('wobj0'))

    abb.send(rrc.PrintText("CONNECTED TO {}, press play to cont...".format("/rob" + str(rob_num))))
    abb.send_and_wait(rrc.Stop(feedback_level=rrc.FeedbackLevel.DONE))
    return robot, abb


def _connect_to_multiple_robot(rob_nums):
    "connect to to multiple robots"
    robots = []
    abbs = []
    for rob_num in rob_nums:
        robot, abb = _connect_to_single_robot(rob_num)
        robots.append(robot)
        abbs.append(abb)

    return abbs, robots


def connect_to_robots(rob_nums):
    return _connect_to_multiple_robot(rob_nums)


if __name__ == "__main__":
    rob_nums = [1, 2]
    abbs, robots = connect_to_robots(rob_nums)

    print(robots)
