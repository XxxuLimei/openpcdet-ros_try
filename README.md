# openpcdet-ros_try  
-------------------------

0303:  
try to run commend`roslaunch openpcdet 3d_object_detector.launch` in zsh, but get  
```
Traceback (most recent call last):
  File "/home/xilm/fuxian/workspace/OpenPCDet_ws/src/OpenPCDet_ros/src/inference.py", line 30, in <module>
    import torch
ModuleNotFoundError: No module named 'torch'
[object_3d_detector_node-2] process has died [pid 206938, exit code 1, cmd /home/xilm/fuxian/workspace/OpenPCDet_ws/src/OpenPCDet_ros/src/inference.py __name:=object_3d_detector_node __log:=/home/xilm/.ros/log/8df7978a-b96e-11ed-9991-95647510a683/object_3d_detector_node-2.log].
log file: /home/xilm/.ros/log/8df7978a-b96e-11ed-9991-95647510a683/object_3d_detector_node-2*.log
```

Have not found the proper solution...  
To be continued...
