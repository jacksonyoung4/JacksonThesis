# Adapted from: K.Shah, "Object_Reconstruction," GitHub repository, 2024. [Online].
# Available: https://github.com/Kaivalya192/Object_Reconstruction/blob/main/rec_con_mask.py

import pyrealsense2 as rs
import numpy as np
import cv2
import os
import time

base_dir = 'input'
sub_dirs = ['depth', 'rgb']

for sub_dir in sub_dirs:
    os.makedirs(os.path.join(base_dir, sub_dir), exist_ok=True)

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15) 
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
profile = pipeline.start(config)

depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

clipping_distance_in_meters = 1  # 1 meter
clipping_distance = clipping_distance_in_meters / depth_scale

align_to = rs.stream.color
align = rs.align(align_to)

intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

with open(os.path.join(base_dir, 'cam_K.txt'), 'w') as f:
    f.write(f"{intr.fx:.7f} 0 {intr.ppx:.7f}\n")
    f.write(f"0 {intr.fy:.7f} {intr.ppy:.7f}\n")
    f.write("0 0 1\n")

print("Press Enter to start recording the sequence.")
while True:
    cv2.namedWindow('Press Enter to start recording', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('Press Enter to start recording', np.zeros((100, 300), np.uint8))
    if cv2.waitKey(1) == 13:
        break
cv2.destroyAllWindows()

time.sleep(2)

print("Recording started. Press Enter to stop recording.")
frame_count = 0
recording = True

try:
    while recording:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not aligned_depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        filename = f"{frame_count:08d}.png"
        rgb_filename = os.path.join(base_dir, 'rgb', filename)
        depth_filename = os.path.join(base_dir, 'depth', filename)

        cv2.imwrite(rgb_filename, color_image)
        cv2.imwrite(depth_filename, (depth_image * depth_scale * 1000).astype(np.uint16))

        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        images = np.hstack((color_image, depth_colormap))
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        if cv2.waitKey(1) == 13:
            recording = False

        frame_count += 1
finally:
    pipeline.stop()
    cv2.destroyAllWindows()

points = []

def select_points(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        points.append((x, y))
        cv2.circle(image_display, (x, y), 3, (0, 255, 0), -1)
        cv2.imshow("Image", image_display)

def create_mask(image, points):
    mask = np.zeros(image.shape[:2], dtype=np.uint8)
    points_array = np.array(points, dtype=np.int32)
    cv2.fillPoly(mask, [points_array], 255)
    return mask

# Use an image from input/rgb for mask creation
image_path = './input/rgb/00000001.png'
image = cv2.imread(image_path)
image_display = image.copy()

cv2.namedWindow("Image")
cv2.setMouseCallback("Image", select_points)

print("Click on the image to select points. Press Enter when done.")

while True:
    cv2.imshow("Image", image_display)
    key = cv2.waitKey(1) & 0xFF
    if key == 13:  # Enter key
        break

mask = create_mask(image, points)

# Make folder if not exist
os.makedirs('./input/masks', exist_ok=True)

# Save the mask image
cv2.imwrite('./input/masks/00000000.png', mask)

cv2.imshow('Mask', mask)
cv2.waitKey(0)
cv2.destroyAllWindows()

