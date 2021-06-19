echo Running Zivid calibration .bat file
CD /D C:\Program Files\Zivid\bin
SET dataset=C:\Users\Edvard\Documents\GitHub\zerowaste_robot\dataset
ZividExperimentalHandEyeCalibration.exe --eih -d "%dataset%" --tf "%dataset%\tf.yaml" --rf "%dataset%\rf.yaml
pause