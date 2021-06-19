import numpy as np

from src.camera.convert import convert2png, load_pointcloud
from src.utility.io import file_name

nums = np.arange(1,20)

for i in nums:
	name = "img{:02d}".format(i)

	pc = load_pointcloud(
		folder = "dataset",
		input_file = file_name(name, ".zdf")
	)

	img_png = convert2png(
		pointcloud = pc,
		folder = "dataset_img",
		output_file = file_name(name, "_rgb.png"),
		)