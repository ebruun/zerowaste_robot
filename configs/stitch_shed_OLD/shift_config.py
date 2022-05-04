# LOCAL IMPORTS
from src.io import _create_file_path, read_data_from_json, save_config_json

from compas_fab.robots import Configuration

folders = ["configs/stitch_shed_OLD/R{}", "configs/stitch_shed/R{}"]
filenames = ["stitch_shed_{0:0{width}}.json"]

rob_num = 2
for i in range(1, 60):
    filename = _create_file_path(
        folder=folders[0].format(rob_num), filename=filenames[0].format(i, width=3)
    )

    data = read_data_from_json(filename)
    config = Configuration.from_data(data)

    shift = 340  # mm

    if rob_num == 1:
        config["r{}_cart_joint".format(rob_num)] += shift
    elif rob_num == 2:
        config["r{}_cart_joint".format(rob_num)] -= shift

    save_config_json(
        config,
        folder=folders[1].format(rob_num),
        output_file=filenames[0].format(i, width=3),
    )
