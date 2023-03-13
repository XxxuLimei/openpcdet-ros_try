# openpcdet-ros_try  
-------------------------

0302:  
`catkin_tools`安装方法：[链接](https://catkin-tools.readthedocs.io/en/latest/installing.html)  

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
Another solution: `source activate base(this is my env name)`, enter conda environment, then make.  
0312：
Solution find!  
Some path code has been modified.  
BTW, when run rosbag, notice if there is "RUNNING". If not, press space to turn state as running so that the RViz can show the outcome.  
