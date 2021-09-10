# TAMS head mesh

## Calibration board

![](calibration_board/combine.svg)

|![](resources/tag_detection.png)|![](resources/tag_detection_result.png)|
|:----------:|:----------:|
|**standalone tags**|**bundle tag**|

## Covert multi-view camera shots to a complete point cloud
- `roslaunch tams_360_scan single_image_server.launch`
- prepare data and put the `*.pickle` to `data/shot_1/` (the pickle file contain png and depth as a dict)
- `rosrun tams_360_scan convert_mesh.py`

|![](resources/points1.png)|![](resources/points2.png)|
|:----------:|:----------:|
|**Result 1**|**Result 2**|

|![](resources/remove_outlier.png)|![](resources/normal_estimation.png)|
|:----------:|:----------:|
|**Remove outlier**|**Normal estimation**|
