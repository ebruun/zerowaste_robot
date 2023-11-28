# LOCAL IMPORTS
from src.RRC_CONNECT import connect_to_robots
from src.io import load_config_json
from src.robot_commands import configs_to_move

rob_nums = [1, 2]
preset_name = [
    "tool_attach",
    "zero_position",
    "ECL_park_high",
    "ECL_park_mid",
    "ECL_park_low",
    "ECL_demo",
    "wobj_x2",
    "dropoff",
]

abbs, _ = connect_to_robots(rob_nums)

configs = []
for abb, rob_num in zip(abbs, rob_nums):

    configs.append(
        load_config_json(
            "configs/presets/R{}".format(rob_num),
            preset_name[0] + ".json",
        )
    )

for abb, rob_num, config in zip(abbs, rob_nums, configs):
    configs_to_move(abb, rob_num, config)
