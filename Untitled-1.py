from src.camera.convert import convert2png, load_pointcloud
from src.utility.io import file_name

name = "img05"

pc = load_pointcloud(
	folder = "dataset",
	input_file = file_name(name, ".zdf")
)

img_png = convert2png(
	pointcloud = pc,
	folder = "dataset",
	output_file = file_name(name, "_rgb.png"),
	)