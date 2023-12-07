# zerowaste_robot

## Notes

the project was completed with compas_fab 0.25.0, from the ziv261_cfab250 environemt

On September 8th, 2022 (after completion of project) the visualizer component was updated: compas_fab 0.27.0 was installed from pre-compiled (unreleased) version, linked to rhino

Make sure to unlink and relink old version if needed in future...or just upgrade to official release now

Excerpt from slack convo:

* "the easiest option is to create an environment as usual, install compas_fab as usual from conda, and then clone the repo and run pip install -e . on the root folder of the repo which will replace the conda install with the source install after that, run python -m compas_rhino.install (as usual)"
* "let me send you a pre-compiled compas_fab from source + components" (this is so components get updated in GH)
* "So, here's a pre-release 0.27 compas_fab"
* "download it and then from the command line, with the conda environment conda_build_update activated, and then install with pip install compas_fab-0.27.0.tar.gz"

This was to fix an issue with ACM not showing up for robot2, wasn't fixed, but this was patched in a live session with Gonzalo

Issue was made on github about this:

https://github.com/compas-dev/compas_fab/issues/372#issue-1366740799

11/12/2022: Upgrade to Zivid SDK V2.8.1, the zivid2 no longer works through USB. Works fine through ethernet, but then have to change the network settings from the robot control setup. Using ziv281_cfab250 environment now

## Folder Structure

| Folder Name    | Explanation |
| -------- | ------- |
| 3dm  | has the 3d model and the grasshopper script for structural analysis and path planning. Has RH custom display settings for MAC and WIN, and embedded files. Also contains a script that can be used to generate configs file planes for the calibration process. |
| configs | Holds the pre-saved robot configurations to move to locations for various robot move tasks: calibration, robot move presets, camera captures for shed/members/demo. |
| data    | holds the data output from the camera captures. Individaul PCD captures and the stitched together PCDs for ECL_demo, members_phase1, members_phase2, shed_original, shed_new. For the HD versions of the PCDs see the ZeroWaste folder on Google Drive.|
| data_path_plan    | the path planned robot configs for the various steps in each of the four disassembly/reassembly phases.|
| input_settings    | zivid camera capture settings as yml files.|
| src    | source code that is run from scripts in main folder.|
| transformations    | camera transformation matrices to transform PCD into same coordinate frame, 4 step process.|


## Code Explanation

This is the code to run the robots and capture the PCDs of the structure

`f1_rob_move_to_presets.py`
  * Move the robots to preset locations
  * The presets are saved for each robot as joint values in the .json files in `configs\presets\RX`

`f2_rob_cam_calibration.py`
  * Perform the camera calibration when mounted on to robots
  * The goal here is to generate the H2 transformation from tool0 -> cam based on how the camera is mounted in relation to the tool0
  * Two main functions here
  * `aquisition_calibration`
    * captures the PCDs of the calibration board for the locations specified in the `configs\calibration\R{}` folder
    * can be run in save_config mode or in normal mode
    * save_config = TRUE, does not take any camera captures, just outputs the .json configs of the robots. Do this only once when you are first setting up the calibration locations you want the robot to move to.
    * save_config = FALSE, execute PCD capture, and save to `data\calibration\R{}` folder. Make sure the R1 and R2 folders exists here, currently not.
    * The aquisitions happen without transforming the PCDs, need them in original location as that is whole point of the calibration, saved as a `{}.zdf` file.
  * `perform_calibration`
    * execute the calibration process after aquisitions completed
    * load the PCDs saved in the `data\calibration` folder
    * Use the code in the Zivid SDK, `zivid.calibration.calibrate_eye_in_hand()`
    * Each capture must have the checkerboard visible
    * Outputs the  `R{}_H2_tool0_cam` file as the result of calibration to `transformations` folder
    * Note: there exist some .bat files in the `data\calibration` folder, which basically do the same thing as the `perform_calibration` function


There are three `rob_cam_XXX` tasks, that are similar in that they just move the robot to a series of pre-recorded places and capture PCDs:
  * `f3a_rob_cam_zerowaste.py`: move both robots around the shed structure and capture this geometry
  * `f3b_rob_cam_members.py`: move R1 to points around where the removed members are stored
  * `f3c_rob_cam_DEMO.py`: move both robots to points around the little demo structure


`f3a_rob_cam_zerowaste.py`
  * The main file to run the zerowaste robot + camera capture routine
  * There are 3 main functions here
  * `aquisition_zerowaste`
    * similar to the other aquisition functions, captures the PCDs of the shed structure at the series of locations specified in the `configs\stitch_shed\R{}` folders
    * performs a transformation on the captured pointcloud (so they all end up in the same coordinate system). R2 and R4 are the fixed transformations saved in `transformations` folder. R3 changes as it is the current frame of the robot represented as a 4x4 matrix in the `pos{}.yaml` file saved at the position of the robot at each capture.
    * saves the individual separate PCDs as `{}_trns.ply` file to the R1 or R2 folders in the `data\stitch_shed` folder.
  * `stitch_zerowaste`
    * stitch together the individual PCD captures into a single file
    * Perform downsampling and other processing of the pointclouds, all set by the values of the variables. Some of these reduction functions are commented off.
    * can stitch together all the captures for a signle robot, if stitch = TRUE
    * can stitch together all the captures for both robots, if stitch_full = TRUE
    * For both can comment on/off either the High-Def or the Low-Def version of the combined file
    * Save as `_R{}_pcd_stitched.pts` for single robot, or `pcd_full.pts` for both robots combined in the `data\stitch_shed` folder. Can read these files into Rhino and manually clean and output at .ply
