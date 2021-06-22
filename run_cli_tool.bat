echo Running Zivid calibration .bat file
CD /D C:\Program Files\Zivid\bin
SET dataset=C:\Users\Edvard\Documents\GitHub\zerowaste_robot\calibration_data
SET results=C:\Users\Edvard\Documents\GitHub\zerowaste_robot\transformations
ZividExperimentalHandEyeCalibration.exe --eih -d "%dataset%" --tf "%results%\H2_tcp_cam.yaml" --rf "%results%\residuals.yaml
pause