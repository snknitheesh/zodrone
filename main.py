import pyrealsense2 as rs
import numpy as np
import cv2
from sklearn.linear_model import RANSACRegressor

BAG_FILE = "recorded_bag/20250520_195504.bag" 
PLANE_STD_THRESHOLD = 2.0  

pipeline = rs.pipeline()
config = rs.config()
config.enable_device_from_file(BAG_FILE)
config.enable_stream(rs.stream.depth)
config.enable_stream(rs.stream.color)

profile = pipeline.start(config)

align_to = rs.stream.color
align = rs.align(align_to)

for _ in range(30):
    pipeline.wait_for_frames()

try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(depth_frame.get_data()).astype(np.float32)
        color_image = np.asanyarray(color_frame.get_data())

        depth_image = cv2.medianBlur(depth_image, 5)

        h, w = depth_image.shape
        x1, y1 = 500, 250  
        x2, y2 = 700, 450
        
        y1, y2 = max(0, y1), min(h, y2)
        x1, x2 = max(0, x1), min(w, x2)
        roi = depth_image[y1:y2, x1:x2].copy()
        depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()

        yy, xx = np.indices(roi.shape)
        X = np.column_stack((xx.ravel(), yy.ravel()))
        Z = roi.ravel()
        mask = ~np.isnan(Z) & (Z > 0)

        if np.count_nonzero(mask) < 100:  # minimum valid points
            surface_type = "UNKNOWN"
            error = float('nan')
        else:
            ransac = RANSACRegressor().fit(X[mask], Z[mask])
            Z_pred = ransac.predict(X[mask])
            error = np.std(Z[mask] - Z_pred)
            surface_type = "EVEN" if error < PLANE_STD_THRESHOLD else "UNEVEN"
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET
        )
        cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        label_color = (0, 255, 0) if surface_type.startswith("EVEN") else (0, 0, 255) if surface_type.startswith("UNEVEN") else (0, 255, 255)
        cv2.putText(color_image, f"{surface_type}", (x1, y1 - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.8, label_color, 2)
        cv2.imshow('Color', color_image)
        cv2.imshow('Depth', depth_colormap)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
