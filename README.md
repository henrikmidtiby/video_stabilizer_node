# Videos stabilizer node for ROS

This ROS node is able to stabilize a video.
The node locates four predefined reference points and transforms the image so that the reference points always are in the same location.
The location os the four reference points in the initial frame is specied in the constructor to the VideoStabilizer class.

