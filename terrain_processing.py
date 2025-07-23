import pyrealsense2 as rs
import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib
from sklearn.linear_model import RANSACRegressor

# Set matplotlib backend for better compatibility
matplotlib.use('TkAgg')  # or 'Qt5Agg' if TkAgg doesn't work

BAG_FILE = "recorded_terrain/tile_1.bag"

# Surface analysis parameters
FLATNESS_THRESHOLD = 0.05  # meters
INCLINATION_THRESHOLD = 15  # degrees
SAMPLE_RATIO = 0.1  # Use 10% of valid points for plane fitting

# Global variables for data collection
residual_errors = []
altitude_data = []
flatness_heatmap_data = []
side_view_data_list = []

# Global figure for persistent plotting
analysis_fig = None
analysis_axes = None

def clear_all_data():
    """Clear all collected data for a fresh start"""
    global residual_errors, altitude_data, flatness_heatmap_data, side_view_data_list, analysis_fig, analysis_axes
    
    residual_errors.clear()
    altitude_data.clear()
    flatness_heatmap_data.clear()
    side_view_data_list.clear()
    
    # Close existing analysis figure
    if analysis_fig is not None:
        plt.close(analysis_fig)
        analysis_fig = None
        analysis_axes = None
    
    print("All data cleared for fresh analysis.")

def fit_plane_ransac(points_3d, sample_ratio=0.1):
    """
    Fit a plane to 3D points using RANSAC
    Returns: plane normal, residual errors for sampled points, fitted plane coefficients (a,b,c,d)
    """
    if len(points_3d) < 3:
        return None, None, None
    
    # Sample random points for fitting
    n_samples = max(100, int(len(points_3d) * sample_ratio))
    if n_samples > len(points_3d):
        n_samples = len(points_3d)
    
    indices = np.random.choice(len(points_3d), n_samples, replace=False)
    sampled_points = points_3d[indices]
    
    # Fit plane using RANSAC (ax + by + cz = d format)
    X = sampled_points[:, :2]  # x, y coordinates
    Z = sampled_points[:, 2]   # z (depth) values
    
    ransac = RANSACRegressor(random_state=42, max_trials=1000)
    ransac.fit(X, Z)
    
    # Get plane coefficients for equation: z = ax + by + c
    # Rearranged to: ax + by - z + c = 0
    a, b = ransac.estimator_.coef_
    c = -1  # coefficient of z
    d = ransac.estimator_.intercept_  # constant term
    
    # Normalize the normal vector
    normal = np.array([a, b, c])
    normal = normal / np.linalg.norm(normal)
    
    # Calculate residual errors for sampled points only
    predicted_z = ransac.predict(X)
    residuals = np.abs(Z - predicted_z)
    
    return normal, residuals, (a, b, c, d)

def calculate_inclination(normal):
    """Calculate inclination angle from vertical (in degrees)"""
    vertical = np.array([0, 0, 1])
    cos_angle = np.dot(normal, vertical) / (np.linalg.norm(normal) * np.linalg.norm(vertical))
    cos_angle = np.clip(cos_angle, -1, 1)  # Handle numerical errors
    angle_rad = np.arccos(np.abs(cos_angle))
    angle_deg = np.degrees(angle_rad)
    return angle_deg

def analyze_surface(depth_image, depth_scale):
    """
    Perform comprehensive surface analysis
    Returns: surface_type, inclination_angle, mean_residual, flatness_map, side_view_data
    """
    h, w = depth_image.shape
    
    # Convert depth image to 3D points
    fy, fx = np.indices(depth_image.shape)
    
    # Camera intrinsics (approximate - adjust based on your camera)
    cx, cy = w // 2, h // 2
    fx_cam = fy_cam = 400  # Approximate focal length
    
    # Convert to real world coordinates
    x = (fx - cx) * depth_image * depth_scale / fx_cam
    y = (fy - cy) * depth_image * depth_scale / fy_cam
    z = depth_image * depth_scale
    
    # Filter out invalid points
    valid_mask = (depth_image > 0) & (depth_image < 5000)  # Valid depth range
    valid_points = np.column_stack((x[valid_mask], y[valid_mask], z[valid_mask]))
    
    if len(valid_points) < 100:
        return "INSUFFICIENT_DATA", 0, 0, np.zeros((h, w)), None
    
    # Fit plane using sampled points
    normal, sampled_residuals, plane_coeffs = fit_plane_ransac(valid_points, SAMPLE_RATIO)
    
    if normal is None:
        return "PLANE_FIT_FAILED", 0, 0, np.zeros((h, w)), None
    
    # Calculate residuals for ALL valid points (not just sampled ones)
    # Recreate the plane equation from coefficients
    a, b, c, d = plane_coeffs
    
    # For all valid points, calculate residual from fitted plane
    all_valid_x = x[valid_mask]
    all_valid_y = y[valid_mask] 
    all_valid_z = z[valid_mask]
    
    # Calculate predicted z values using plane equation: z = (ax + by + d) / (-c)
    predicted_z = (a * all_valid_x + b * all_valid_y + d) / (-c)
    all_residuals = np.abs(all_valid_z - predicted_z)
    
    # Calculate inclination
    inclination_angle = calculate_inclination(normal)
    
    # Calculate mean residual error
    mean_residual = np.mean(all_residuals)
    
    # Determine surface type
    is_flat = mean_residual < FLATNESS_THRESHOLD
    is_level = inclination_angle < INCLINATION_THRESHOLD
    
    if is_flat and is_level:
        surface_type = "FLAT_LEVEL"
    elif is_flat and not is_level:
        surface_type = "FLAT_INCLINED"
    elif not is_flat and is_level:
        surface_type = "ROUGH_LEVEL"
    else:
        surface_type = "ROUGH_INCLINED"
    
    # Create flatness heatmap - map residuals back to image coordinates
    flatness_map = np.zeros((h, w))
    flatness_map[valid_mask] = all_residuals
    
    # Create side view data for plotting
    # Take a horizontal slice through the middle of the image
    mid_row = h // 2
    slice_depth = depth_image[mid_row, :] * depth_scale
    slice_x = np.arange(w) * depth_scale / fx_cam  # Convert pixel coordinates to real world
    
    # Calculate fitted plane values for the same slice
    slice_y = (mid_row - cy) * depth_scale / fy_cam  # y coordinate for the slice
    slice_fitted = (a * slice_x + b * slice_y + d) / (-c)
    
    # Filter out invalid points in the slice
    valid_slice_mask = slice_depth > 0
    side_view_data = {
        'x': slice_x[valid_slice_mask],
        'actual_depth': slice_depth[valid_slice_mask],
        'fitted_depth': slice_fitted[valid_slice_mask]
    }
    
    print(f"Flatness map stats: min={flatness_map.min():.4f}, max={flatness_map.max():.4f}, non-zero={np.count_nonzero(flatness_map)}")
    
    return surface_type, inclination_angle, mean_residual, flatness_map, side_view_data

def update_data_collection(mean_altitude, residual_error, flatness_map, side_view_data):
    """Update global data for analysis plots"""
    global residual_errors, altitude_data, flatness_heatmap_data, side_view_data_list
    
    residual_errors.append(residual_error)
    altitude_data.append(mean_altitude)
    flatness_heatmap_data.append(flatness_map)
    if side_view_data is not None:
        side_view_data_list.append(side_view_data)

def plot_analysis_results():
    """Generate and display analysis plots"""
    global residual_errors, altitude_data, flatness_heatmap_data, side_view_data_list, analysis_fig, analysis_axes
    
    try:
        if len(residual_errors) < 2:
            print("Not enough data for plotting yet...")
            return
        
        print(f"Plotting with {len(residual_errors)} data points")
        
        # Create figure only once
        if analysis_fig is None:
            plt.ion()  # Enable interactive mode
            analysis_fig, analysis_axes = plt.subplots(2, 2, figsize=(12, 8))
            analysis_fig.suptitle('Real-time Terrain Analysis')
            analysis_fig.show()  # Explicitly show the figure
            print("Created new analysis figure")
        
        # Clear all axes
        for ax in analysis_axes.flat:
            ax.clear()
        
        ax1, ax2, ax3, ax4 = analysis_axes.flat
        
        # 1. Histogram of residual errors
        ax1.hist(residual_errors, bins=20, alpha=0.7, color='blue')
        ax1.set_title('Histogram of Residual Errors')
        ax1.set_xlabel('Residual Error (m)')
        ax1.set_ylabel('Frequency')
        ax1.grid(True)
        
        # 2. Altitude vs Residual Error
        ax2.scatter(altitude_data, residual_errors, alpha=0.6, color='red', s=10)
        ax2.set_title('Altitude vs Residual Error')
        ax2.set_xlabel('Mean Altitude (m)')
        ax2.set_ylabel('Residual Error (m)')
        ax2.grid(True)
        
        # 3. Side view of terrain (replaced heatmap)
        if side_view_data_list and len(side_view_data_list) > 0:
            latest_side_view = side_view_data_list[-1]
            if latest_side_view and len(latest_side_view['x']) > 0:
                ax3.plot(latest_side_view['x'], latest_side_view['actual_depth'], 
                        'b-', label='Actual Depth', linewidth=2, alpha=0.8)
                ax3.plot(latest_side_view['x'], latest_side_view['fitted_depth'], 
                        'r--', label='Fitted Plane', linewidth=2)
                ax3.set_title('Terrain Side View (Cross-section)')
                ax3.set_xlabel('Distance (m)')
                ax3.set_ylabel('Depth (m)')
                ax3.legend()
                ax3.grid(True)
                ax3.invert_yaxis()  # Invert y-axis so depth increases downward
            else:
                ax3.text(0.5, 0.5, 'No side view data', ha='center', va='center', transform=ax3.transAxes)
                ax3.set_title('Terrain Side View')
        else:
            ax3.text(0.5, 0.5, 'No data yet', ha='center', va='center', transform=ax3.transAxes)
            ax3.set_title('Terrain Side View')
        
        # 4. Residual error over time
        ax4.plot(range(len(residual_errors)), residual_errors, color='green', linewidth=1)
        ax4.set_title('Residual Error Over Time')
        ax4.set_xlabel('Frame Number')
        ax4.set_ylabel('Residual Error (m)')
        ax4.grid(True)
        
        plt.tight_layout()
        analysis_fig.canvas.draw()
        analysis_fig.canvas.flush_events()
        plt.pause(0.001)  # Small pause to ensure rendering
        print("Plot updated successfully")
        
    except Exception as e:
        print(f"Error in plotting: {e}")
        import traceback
        traceback.print_exc()  

pipeline = rs.pipeline()
config = rs.config()
config.enable_device_from_file(BAG_FILE)
config.enable_stream(rs.stream.depth)
config.enable_stream(rs.stream.color)

# Disable looping of the bag file
config.disable_all_streams()
config.enable_device_from_file(BAG_FILE, repeat_playback=False)
config.enable_stream(rs.stream.depth)
config.enable_stream(rs.stream.color)

profile = pipeline.start(config)

align_to = rs.stream.color
align = rs.align(align_to)

for _ in range(30):
    pipeline.wait_for_frames()

try:
    frame_count = 0
    processing_complete = False
    final_combined_image = None
    final_depth_colormap = None
    bag_ended = False
    
    print(f"Starting analysis of bag file: {BAG_FILE}")
    print("Press 'q' to quit, 'r' to restart analysis with cleared data")
    
    while True:
        if not processing_complete and not bag_ended:
            # Process frames from bag file
            try:
                frames = pipeline.wait_for_frames()
                aligned_frames = align.process(frames)
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()

                if not depth_frame or not color_frame:
                    continue

                depth_image = np.asanyarray(depth_frame.get_data()).astype(np.float32)
                color_image = np.asanyarray(color_frame.get_data())

                depth_image = cv2.medianBlur(depth_image, 5)
                
                # Get depth scale for real-world measurements
                depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
                
                # Perform surface analysis
                surface_type, inclination_angle, mean_residual, flatness_map, side_view_data = analyze_surface(depth_image, depth_scale)
                
                # Debug information every 30 frames
                if frame_count % 30 == 0:
                    print(f"Frame {frame_count}: Surface={surface_type}, Inclination={inclination_angle:.1f}°, Residual={mean_residual:.4f}m")
                
                # Calculate mean altitude
                valid_depths = depth_image[depth_image > 0]
                mean_altitude = np.mean(valid_depths) * depth_scale if len(valid_depths) > 0 else 0
                
                # Update data collection
                update_data_collection(mean_altitude, mean_residual, flatness_map, side_view_data)
                
                # Create visualizations
                depth_colormap = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET
                )
                
                # Create combined RGB + Heatmap visualization
                h, w = color_image.shape[:2]
                
                if flatness_map.max() > 0:
                    # Normalize the flatness map for better visualization
                    normalized_flatness = cv2.normalize(flatness_map, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                    flatness_colored = cv2.applyColorMap(normalized_flatness, cv2.COLORMAP_HOT)
                    
                    # Create side-by-side view: RGB on left, heatmap on right
                    combined_image = np.zeros((h, w * 2, 3), dtype=np.uint8)
                    combined_image[:, :w] = color_image
                    combined_image[:, w:] = flatness_colored
                    
                    # Add labels
                    cv2.putText(combined_image, "RGB", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                    cv2.putText(combined_image, "Flatness Heatmap", (w + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                else:
                    # If no heatmap data, just show RGB with a placeholder
                    combined_image = np.zeros((h, w * 2, 3), dtype=np.uint8)
                    combined_image[:, :w] = color_image
                    cv2.putText(combined_image, "RGB", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                    cv2.putText(combined_image, "No Heatmap Data", (w + 10, h//2), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                
                # Add text overlays with analysis results to the combined image
                text_y = 60
                cv2.putText(combined_image, f"Surface: {surface_type}", (10, text_y), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                text_y += 30
                cv2.putText(combined_image, f"Inclination: {inclination_angle:.1f} deg", (10, text_y), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                text_y += 30
                cv2.putText(combined_image, f"Residual Error: {mean_residual:.3f} m", (10, text_y), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                text_y += 30
                cv2.putText(combined_image, f"Altitude: {mean_altitude:.2f} m", (10, text_y), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Store the final images for frozen display
                final_combined_image = combined_image.copy()
                final_depth_colormap = depth_colormap.copy()
                
                frame_count += 1
                
                # Update analysis plots every 30 frames
                if frame_count % 30 == 0:
                    print(f"Frame {frame_count}: Updating plots with {len(residual_errors)} data points")
                    plot_analysis_results()
                
                # Display current frame
                cv2.imshow('RGB + Flatness Analysis', combined_image)
                # cv2.imshow('Depth', depth_colormap)
                
            except RuntimeError:
                # Bag file has ended
                print(f"\nBag file processing complete! Processed {frame_count} frames.")
                print("Analysis complete. Press 'q' to exit or 'r' to restart with fresh data.")
                processing_complete = True
                bag_ended = True
                
                # Show final analysis plots
                plot_analysis_results()
                
                # Add completion message to the combined image
                if final_combined_image is not None:
                    cv2.putText(final_combined_image, "COMPLETE - Press 'q' to exit, 'r' to restart", 
                               (10, final_combined_image.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # Display the frozen images (only if they exist)
        if final_combined_image is not None:
            cv2.imshow('RGB + Flatness Analysis', final_combined_image)
        
        # Check for keys
        key = cv2.waitKey(30) & 0xFF
        if key == ord('q'):
            print("Exiting...")
            break
        elif key == ord('r') and processing_complete:
            print("Restarting analysis with cleared data...")
            clear_all_data()
            
            # Reset pipeline
            pipeline.stop()
            
            # Restart pipeline
            pipeline = rs.pipeline()
            config = rs.config()
            config.disable_all_streams()
            config.enable_device_from_file(BAG_FILE, repeat_playback=False)
            config.enable_stream(rs.stream.depth)
            config.enable_stream(rs.stream.color)
            profile = pipeline.start(config)
            
            # Skip initial frames
            for _ in range(30):
                try:
                    pipeline.wait_for_frames()
                except RuntimeError:
                    break
            
            # Reset variables
            frame_count = 0
            processing_complete = False
            bag_ended = False
            final_combined_image = None
            final_depth_colormap = None

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    
    # Display final analysis plots
    if len(residual_errors) > 0:
        # Close any existing analysis window
        if analysis_fig is not None:
            plt.close(analysis_fig)
        
        # Create final summary plot
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
        fig.suptitle('Final Terrain Analysis Summary')
        
        # 1. Histogram of residual errors
        ax1.hist(residual_errors, bins=30, alpha=0.7, color='blue')
        ax1.set_title('Histogram of Residual Errors')
        ax1.set_xlabel('Residual Error (m)')
        ax1.set_ylabel('Frequency')
        ax1.grid(True)
        
        # 2. Altitude vs Residual Error
        ax2.scatter(altitude_data, residual_errors, alpha=0.6, color='red')
        ax2.set_title('Altitude vs Residual Error')
        ax2.set_xlabel('Mean Altitude (m)')
        ax2.set_ylabel('Residual Error (m)')
        ax2.grid(True)
        
        # 3. Side view of terrain (replaced heatmap)
        if side_view_data_list and len(side_view_data_list) > 0:
            latest_side_view = side_view_data_list[-1]
            if latest_side_view and len(latest_side_view['x']) > 0:
                ax3.plot(latest_side_view['x'], latest_side_view['actual_depth'], 
                        'b-', label='Actual Depth', linewidth=2, alpha=0.8)
                ax3.plot(latest_side_view['x'], latest_side_view['fitted_depth'], 
                        'r--', label='Fitted Plane', linewidth=2)
                ax3.set_title('Final Terrain Side View')
                ax3.set_xlabel('Distance (m)')
                ax3.set_ylabel('Depth (m)')
                ax3.legend()
                ax3.grid(True)
                ax3.invert_yaxis()  # Invert y-axis so depth increases downward
            else:
                ax3.text(0.5, 0.5, 'No side view data', ha='center', va='center', transform=ax3.transAxes)
                ax3.set_title('Terrain Side View')
        else:
            ax3.text(0.5, 0.5, 'No data available', ha='center', va='center', transform=ax3.transAxes)
            ax3.set_title('Terrain Side View')
        
        # 4. Residual error over time
        ax4.plot(residual_errors, color='green')
        ax4.set_title('Residual Error Over Time')
        ax4.set_xlabel('Frame Number')
        ax4.set_ylabel('Residual Error (m)')
        ax4.grid(True)
        
        plt.tight_layout()
        plt.show()  # Keep final plot open
        
        # Print summary statistics
        print("\n=== TERRAIN ANALYSIS SUMMARY ===")
        print(f"Total frames analyzed: {len(residual_errors)}")
        print(f"Mean residual error: {np.mean(residual_errors):.4f} m")
        print(f"Std residual error: {np.std(residual_errors):.4f} m")
        print(f"Mean altitude: {np.mean(altitude_data):.2f} m")
        print(f"Altitude range: {np.min(altitude_data):.2f} - {np.max(altitude_data):.2f} m")
