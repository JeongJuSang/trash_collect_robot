This Repo include code for mobile robot

2d_to_3d.py is a code that extracts the z value (distance) of the center point (pixel coordinates) by projecting the center point of the bounding box of the 2d image_frame to the 3d_depth_frame using the d435i camera.(use yolov5_ros)

move_color.py is a code that allows the mobile robot to follow a path in a designated position at a designated position by entering map_frame coordinates and quaternion values.

trash_detect_move5.py is a code that utilizes the distance value extracted from 2d_to_3d.py to obtain the angle using the vector of object coordination and camera coordination, and automatically publishes the goal coordinates on map_frame using the resulting angle and distance value. (use ros1 navigation package)

Trash_detect_move5.py will continue to be modified in the future, and since the median value extraction method of the bounding box has difficulty in extracting an accurate distance value, we plan to update it using a segmentation method.

All of this has been tested on ros1 noetic.
