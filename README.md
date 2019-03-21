# Videos stabilizer node for ROS

This ROS node is able to stabilize a video.
The node locates four predefined reference points and transforms the image so that the reference points always are in the same location.
The location os the four reference points in the initial frame is specied in the constructor to the VideoStabilizer class.

The test.mp4 video that was used to test the package is not part of the git repository.
It can however be downloaded from youtube on the [link](https://youtu.be/UA1zPvjSbjs).


## Getting started

To get the system in a working state, the following steps should be taken.

1. Make a ROS workspace

```
mkdir -p ~/ros_workspace/src
```

2. Clone the git repository of the project and its dependencies

```
cd ~/ros_workspace/src
git clone git@github.com:henrikmidtiby/video_stabilizer_node.git
git clone git@github.com:ros-drivers/video_stream_opencv.git
```

3. Download the youtube video using [youtube-dl](https://pypi.org/project/youtube_dl/)
    
```
cd ~/ros_workspace/src/video_stabilizer_node/data
youtube-dl -o test.mp4 https://youtu.be/UA1zPvjSbjs
```

4. Run catkin build

```
cd ~/ros_workspace
source /opt/ros/melodic/setup.bash 
catkin build
```

5. Run the stabilizer, 
    
```
cd ~/ros_workspace
source devel/setup.bash
roslaunch video_stabilizer_node test.launch
```

