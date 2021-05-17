# Homography-based 2D Visual Servoing with Remote Center of Motion
This code implements a [remote center of motion](https://github.com/RViMLab/rcm) with [homography based visual servo](https://github.com/RViMLab/h_vs), see below
<br/>
<img src="img/h_rcm_vs_nodes.png" width="800"/>
Two main nodes are provided
- [h_rcm_vs_server_node](src/h_rcm_vs_server_node.cpp) has a [HRCMVSActionServer](include/h_rcm_vs/h_rcm_vs_action_server.h) that implements a [RCMActionServer](https://github.com/RViMLab/rcm/blob/master/rcm_impl/include/rcm_impl/rcm_action_server.h) with task functions designed specifically for homography-based control. It sends joint goals to the robot. It reads in parameters from [rcm_params.yml](config/rcm_params.yml)
- [h_rcm_vs_client_node](src/h_rcm_vs_client_node.cpp) has a [HRCMVSActionClient](include/h_rcm_vs/h_rcm_vs_action_client.h) that sends goals to the [HRCMVSActionServer](include/h_rcm_vs/h_rcm_vs_action_server.h). It subscribes to `visual_servo/twist`, the desired twist velocity in the camera frame, and to `rcm/ptrocar`, the desired trocar position

## Examples
The provided nodes execute any desired homography, as generated from [h_vs](https://github.com/RViMLab/h_vs). Some examples with launch files are provided in a the sample workspace [h_rcm_vs_ws](https://github.com/RViMLab/h_rcm_vs_ws)
