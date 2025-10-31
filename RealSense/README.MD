# Recording with RealSense
- Ensure version of python is 3.11 (latest version pyrealsense2 supports).
- Install the following packages:
``` bash
pip install pyrealsense2
pip install opencv-python
pip install pillow
```

## Record for BundleSDF
- Records RGB and depth data, and saves intrinsic matrix of camera.
- Saves RGB data in both PNG and JPEG format as XMem requires JPEG (for mask generation).
- Allows manual annotation of mask in first frame.
- Tips for recording:
    * Show all object surfaces to camera.
    * Keep object > 30cm away from camera.
    * Do not place object on flat surface as this leads to lack of contrast in depth data.
    * Avoid using reflective objects (e.g. metal objects).
``` bash
python record_for_BundleSDF.py
```
## Record for FoundationPose
- Records RGB and depth data, and saves intrinsic matrix of camera.
- Allows manual annotation of mask in first frame.
- FoundationPose is fairly robust to quality of data. Just ensure metalic object in scene is not corrupting depth data.
``` bash
python record_for_FoundationPose.py
```
## Capture Pose
- Same as record_for_FoundationPose.py but only records one frame.
- Useful if only initial object pose estimation in first frame is needed (not tracking).
``` bash
python capture_pose.py
```