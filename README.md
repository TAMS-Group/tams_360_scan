# TAMS head mesh

## Calibration board

![](calibration_board/combine.svg)

|![](resources/tag_detection.png)|![](resources/tag_detection_result.png)|
|:----------:|:----------:|
|**standalone tags**|**bundle tag**|

## Covert multi-view camera shots to a complete point cloud
- `roslaunch tams_head_mesh single_image_server.launch`
- `rosrun tams_head_mesh convert_mesh2.py` (need pickle file that contain png and depth as a dict)

|![](resources/points1.png)|![](resources/points2.png)|
|:----------:|:----------:|
|**Result 1**|**Result 2**|
