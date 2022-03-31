echo Running Zivid calibration .bat file
CD /D C:\Program Files\Zivid\bin
SET dataset=C:\Users\Edvard\Documents\GitHub\zerowaste_robot\data\calibration\R1
SET results=C:\Users\Edvard\Documents\GitHub\zerowaste_robot\transformations
ZividExperimentalHandEyeCalibration.exe --eih -d "%dataset%" --tf "%results%\R1_H2_robot_cam.yaml" --rf "%results%\residuals_R1.yaml
pause