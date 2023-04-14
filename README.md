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
1. **想把kitti_raw_data的数据进行painted**；  
- 首先把数据路径都改过来；  
- 准备calibration文件：对照train中的calib进行编写，这里有一篇[博客](https://zhuanlan.zhihu.com/p/99114433)说明了train中的calib各个参数的含义，对照着修改一下；  
- calibration文件修改完毕，具体做法是：首先打开`calib_cam_to_cam.txt`文件，将`P_rect_00, P_rect_01, P_rect_02, P_rect_03`这几个参数作为calib文件的`P0, P1, P2, P3`，再将`R_rect_00`作为calib文件的`R0_rect`；打开`calib_velo_to_cam.txt`文件，将`R, T`参数作为`Tr_velo_to_cam`参数，具体来说是每3个`R`放一个`T`，这样就把3\*3的`R`和1\*3的`T`变成了3\*4的`Tr_velo_to_cam`；再打开`calib_imu_to_velo.txt`文件，将`R, T`参数作为`Tr_imu_to_velo`参数，具体来说是每3个`R`放一个`T`，这样就把3\*3的`R`和1\*3的`T`变成了3\*4的`Tr_imu_to_velo`。我修正好的calibration文件上传到了calib文件夹中。  
- 接下来使用`painting_raw_data.py`文件进行painting过程。  
- 然后使用`demo.py`文件对结果进行可视化，如下图所示。  
![1679105917(1)](https://github.com/XxxuLimei/openpcdet-ros_try/blob/main/pictures/20230402_1.png)  
2. 整个实现的流程应该是：相机图像、激光雷达点云数据发布-->订阅这两个话题-->获取点云-->获取图像并进行语义分割-->进行图像和点云的calibration-->涂抹点云，生成涂抹后的点云帧-->使用pointpillar_painted进行点云检测-->将检测好的框画出来  
- 如果实现了以上几步的话，再考虑进行对涂抹的点云着色，在RViz上发布颜色不同的点云；  
- **对相机图像和激光雷达点云数据进行发布**：✓  
- **订阅这两个发布的话题**：✓  

## 0403：
1. **进行语义分割**：剩下的分为以下几个步骤：获取左方相机检测到的图像-->get score（即图像分割）-->获取右方相机检测到的图像-->get score（即图像分割）-->获取点云-->进行点云涂抹-->调用模型进行检测-->发布检测框；  
- **获取左方相机检测到的图像，然后转换为可以用于目标检测的img格式**：  
## 0409:  
1. 将之前的工作整理步骤出来：  
- 打开终端，运行`roscore`;  
- 在`/home/xilm/pointpainting_ros/catkin_ws路径下运行`source devel/setup.bash`，然后运行`rviz`;  
- 在`/home/xilm/pointpainting_ros/catkin_ws路径下运行`source devel/setup.bash`，然后`cd src/pointpainting_ros/src/`,接着运行`rosrun pointpainting_ros kitti_raw_data.py`,就可以对左视相机和右视相机的图像，以及点云数据进行发布了；  
- 在rviz下选择`File->Open config`，然后选择`/home/xilm/pointpainting_ros/catkin_ws/src/pointpainting_ros/launch`路径下的`default_pointpainting.rviz`文件，就可以在Rviz下看到发布的数据了。  
![](https://github.com/XxxuLimei/openpcdet-ros_try/blob/main/pictures/Screenshot%20from%202023-04-09%2018-49-10.png)  
- 接着对图像数据和点云数据进行订阅：打开一个终端，运行`source devel/setup.bash`,然后`cd src/pointpainting_ros/src/`,再`rosrun pointpainting_ros pointpainting_detect.py`,就可以看到以下信息：  
```
(base) xilm@xilm-MS-7D17:~/pointpainting_ros/catkin_ws/src/pointpainting_ros/src$ rosrun pointpainting_ros pointpainting_detect.py 
[INFO] [1681037714.261403]: get left image!
[INFO] [1681037714.262279]: get right image!
[INFO] [1681037714.343794]: get point cloud!
[INFO] [1681037714.364087]: get right image!
[INFO] [1681037714.369660]: get left image!
[INFO] [1681037714.446046]: get point cloud!
[INFO] [1681037714.466271]: get left image!
[INFO] [1681037714.467332]: get right image!
[INFO] [1681037714.548663]: get point cloud!
```  
- 需要找到如何将接收到的ros图像转为PIL图像: 找到一个[教程](https://blog.csdn.net/weixin_40863346/article/details/80430251),使用CvBridge将接收到的图像转为cv图像，然后使用cv2.imshow()发布出来，可以看到实时显示的图像。  
```
cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
cv2.imshow("Image window", cv_image)
cv2.waitKey(3)
```  
![](https://github.com/XxxuLimei/openpcdet-ros_try/blob/main/pictures/Screenshot%20from%202023-04-09%2019-15-24.png)  
- 接下来进行语义分割：  
## 0411：  
1. 前两天完成了deeplabv3+在ros上的复现，可以对图像进行接收，然后进行语义分割，并将分割结果发布出去；其逻辑是：
- 首先定义subsrcier,并加载deeplabv3+模型；  
- 定义一个图像publisher，用于后续发布分割的图像，同时开启接收器；  
- 一旦接收器接收到图像，立刻调用处理函数，该函数用于对图像进行语义分割，并将分割好的图像publish出去。  
2. 在pointpainting中首先进行双目图像的语义分割。  
## 0412：  
1. 尝试了`inference_segmentor`函数可以传入`cv.imread()`的图像；  
2. opencv使用的是BGR编码，Matplotlib使用的是RGB编码，注意转换；  
3. 传过来的图片总是没办法正常读取，这里找到一个[PIL和Opencv相互转化](https://blog.csdn.net/weixin_50113231/article/details/123004037)的文章。  
## 0413：  
1. mmseg的`show_result`函数不知道为什么总报错：  
```
File "/home/xilm/pointpainting_ros/catkin_ws/src/pointpainting_ros/src/mmseg/models/segmentors/base.py", line 257, in show_result
    color_seg[seg == label, :] = color
IndexError: too many indices for array: array is 3-dimensional, but 4 were indexed
```  
通过打印tensor的shape，发现是因为处理后的图像是`N, C, H, W`，把它squeeze(0)，去掉N（Batch dim）即可。  
2. 还是不行，发现语义分割得到的结果是all black。尝试用最简单的脚本分割了kitti，是没问题的，图片在下面。  
![](https://github.com/XxxuLimei/openpcdet-ros_try/blob/main/pictures/img_screenshot_13.04.2023.png)  
3. 用脚本测试了deeplabv3+分割的kitti_sequence结果如下：  
![](https://github.com/XxxuLimei/openpcdet-ros_try/blob/main/pictures/img_screenshot_deeplabv3%2B_13.04.2023.png)  
## 0414：  
1. 终于把语义分割搞好了！错误原因在于`inference_segmentor(seg_model, cv_image)`后需要添加`max(1)[1].cpu().numpy()[0]`，也就是说从19个类中选出最大的一类用于语义分割。  
- 注意：`inference_segmentor(seg_model, cv_image)`后的result的shape为`(N, 19, H, W)`，这里由于每次只接收一张图，所以N=1，那么`max(1)`就是从19个类别中选出最大的一类。  
- 输入`show_result`的变量形状应该是`[(H, W)]`。  
- 最终效果如图：  
![](https://github.com/XxxuLimei/openpcdet-ros_try/blob/main/pictures/Screenshot%20from%202023-04-14%2010-34-23.png)  
2. 将语义分割的结果以ROS图像的形式发布出去，在RViz上进行接收。  
- 添加了以下四行实现了该功能：  
```
image_pub_l = rospy.Publisher('/image_seg_l', Image, queue_size=1)  # define publisher
ros_frame = bridge.cv2_to_imgmsg(seg_img, "bgr8")  # convert cv2image to imgmsg
ros_frame.header.stamp = rospy.Time.now()  # get time frame
image_pub_l.publish(ros_frame)  # use publisher to publish imgmsg
```  
3. **进行图像和点云的calibration**：  
- 定义`get_calib_file()`函数，在其中声明全局变量`calib`，并在主函数开始时调用；  
4. **涂抹点云，生成涂抹后的点云帧**：  
- 
