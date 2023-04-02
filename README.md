# openpcdet-ros_try  
-------------------------

## 0302:   
`catkin_tools`安装方法：[链接](https://catkin-tools.readthedocs.io/en/latest/installing.html)  

## 0303:  
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

## 0312：
Solution find!  
Some path code has been modified.  
BTW, when run rosbag, notice if there is "RUNNING". If not, press space to turn state as running so that the RViz can show the outcome.  

## 0317： 开始尝试复现其他框架  
1. 将原来的sequence.bag换为kitti里其他的raw data：  
- 在终端运行`rosbag play kitti_sequence11_half.bag -r 1 --pause`后，按下空格，显示为running;接着运行`rostopic list`查看bag发布了什么话题：  
```
(base) xilm@xilm-MS-7D17:~$ rostopic list
/clock
/rosout
/rosout_agg
/velodyne_points
```  
- 使用从kitti官网下载的raw data：2011_09_26_0005，利用kitti2bag转为bag后，将bag进行play查看发布的topic：  
```
(base) xilm@xilm-MS-7D17:~$ rostopic list
/clock
/kitti/camera_color_left/camera_info
/kitti/camera_color_left/image_raw
/kitti/camera_color_right/camera_info
/kitti/camera_color_right/image_raw
/kitti/camera_gray_left/camera_info
/kitti/camera_gray_left/image_raw
/kitti/camera_gray_right/camera_info
/kitti/camera_gray_right/image_raw
/kitti/oxts/gps/fix
/kitti/oxts/gps/vel
/kitti/oxts/imu
/kitti/velo/pointcloud
/rosout
/rosout_agg
/tf
/tf_static
```  
可以看到，这里的点云节点名称从`/velodyne_points`改为了`/kitti/velo/pointcloud`,由此，将`config.yaml`和`.rviz`文件中的名称改过来  
改完之后发现rviz上无法显示点云，但是能进行检测  
- 突然发现，这里运行rosbag时`rosbag play kitti_2011_09_26_drive_0005_synced.bag -r 0.1 --pause`,-r后面的参数为0.1是帧率，只有帧率低了，检测框才能和点云帧对齐，因为检测框发布的很慢。据此修改了pointpillar_ros的代码，把launch文件里直接运行rosbag的命令去掉，在终端自己运行kitti的bag，帧率调整为0.1就可以看到好的检测结果了。  
- 这么说，还需要解决如何让检测框快速发布的问题，不然根本不可能进行实时检测。。。  


## 0318:  
1. 解决了更换一个kitti bag的问题，就是通过修改之前的两个文件，然后在rviz上修改一下global options的fixed frame,将其从velodyne改为velo_link  
![1679105917(1)](https://user-images.githubusercontent.com/96283702/226078139-bb5ada82-88e9-42c9-a00a-1215c9a58df6.png)  
2. 接下来准备再更换几个kitti的bag，看看有没有共性问题，方法就是下载raw data后使用kitti2bag转换  
- 下载raw data，这里我使用0084；  
- 接着把他们解压，解压后应为2011_09_26-->synced+calib，然后回到2011_09_26前一个目录下进行kitti2bag；  
- 查看它的rostopic  
```
(base) xilm@xilm-MS-7D17:~$ rostopic list
/clicked_point
/clock
/detect_3dbox
/initialpose
/kitti/camera_color_left/camera_info
/kitti/camera_color_left/image_raw
/kitti/camera_color_right/camera_info
/kitti/camera_color_right/image_raw
/kitti/camera_gray_left/camera_info
/kitti/camera_gray_left/image_raw
/kitti/camera_gray_right/camera_info
/kitti/camera_gray_right/image_raw
/kitti/oxts/gps/fix
/kitti/oxts/gps/vel
/kitti/oxts/imu
/kitti/velo/pointcloud
/move_base_simple/goal
/rosout
/rosout_agg
/tf
/tf_static
```  
3. 改完发现总是漏检好多，于是尝试改检测阈值：从0.8改到0.5，发现检测框突然多了很多，但是还是有很多当前帧明明存在物体却不予显示检测结果，或者是明明没有物体但检测出了物体，观察发现一段时间内检测出的结果是正确的，即如果当前存在物体，有时会显示，有时不会显示，会显示的帧数多于不显示的帧数。  
4. 接着更换测试的模型文件，之前使用的是pv_rcnn，现在要对openpcdet里面的其他文件都进行一个测试：  
- 先替换pointpillar模型: 在config.yaml文件中修改；发现pointpillar的检测结果确实逊。。。于pvrcnn（/狗头   
- 看看SECOND模型： 在config.yaml文件中修改；发现它的检测结果更逊。。。于前两个（/狗头   
- 看看SECOND-iou模型： 比SECOND的结果改善了很多；  
- 看看pointrcnn模型： 比pointpillar, SECOND以及SECOND-iou都差。。。好多检测不到的；  
- 看看pointrcnn-iou模型： 比pointrcnn出现的误检框少了很多，但是检测效果还是比较差，好多该检测出来的没有检测出来；  
- 看看Part-A2-free模型： 还是好多没检测出来的，真的不如pv-rcnn；  
- 看看Part-A2-Anchor模型： 比上面的free版本好太多了，但是依然有许多误检，漏检少了很多；  
- 看看Voxel-Rcnn模型： 误检还挺多，但是漏检不太多了，总体比不上pv-rcnn；  
5. 接下来尝试把视觉融合的模型放上去，先试试pointpainting：  
## 0323:  
- 之前把pointpainting复现了一下，梳理了该算法的思路：首先是对图像进行语义分割，接着对点云进行painting，之后再对点云进行目标检测；  
- 由于在RViz上涉及到的只有点云的可视化和绘制检测框，而点云的可视化目前在openpcdet——ros框架下已经完成，因此重点考虑检测框；  
- 对于一台安装有camera和velodyne的小车来说，进行pointpainting的内部流程有四个步骤：1.camera拍摄照片，车载gpu进行语义分割算法；2.velodyne采集点云，使用painting.py文件对点云着色；3.运行test.py，获得检测框坐标；4.将检测框显示在RViz上。  
- 这可以分为两种情况：1.使用KITTI数据集；2.实际情况下的小车运行（初步估计应该不能实时检测，延时太高了）；  
- 对于第一种情况：使用KITTI数据集，如果不更换语义分割的模型，那么就意味着painted的点云不变-->如果不需要训练，仅仅是测试的话，只要给了训练好的权重文件，看看如何把test.py运行产生的3D检测框发布到RViz上即可；  
## 0324:  
- 将openpcdet的config.yaml文件修改了模型的配置，然后直接用09_26_0084的bag运行，发现检测框没有显示，此时终端已经开始报错，总体原因就是这个raw_data没有painted。  
- 考虑怎么利用0084的图像去着色0084的点云，有了着色的点云之后，怎么把他转为bag，这些着色的部分如何发布。  
- 本来准备修改pointpainting中的painting.py脚本的，结果发现KITTI的3D目标检测数据集和raw_data数据给的方式不大一样。。。得先好好学习一下raw_Data数据组织方式是什么了，再看看painting.py怎么适应它；  
## 0402:  
- 真的很奇怪，只要新安装一个框架，里面如果含有pcdet，那么我的pcdet的环境就会改变。。。  
- 没啥事，发现pointpainting的pcdet没有voxel_rcnn，所以换了个别的模型，成功了。  
- *想把kitti_raw_data的数据进行painted*；  
