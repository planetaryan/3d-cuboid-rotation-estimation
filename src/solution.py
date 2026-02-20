import numpy as np
import open3d as o3d
from scipy.spatial import ConvexHull
from rosbags.highlevel import AnyReader
from pathlib import Path
import csv

# --- CONFIGURATION ---
BAG_PATH = Path('depth') 
TOPIC_NAME = '/depth'

# Camera Intrinsics (Standard Pinhole fallback)
FX, FY = 525.0, 525.0
CX, CY = 319.5, 239.5

def depth_to_pointcloud(depth_img):
    """
    Projects 2D depth to 3D and applies a robust ROI crop to isolate the box.
    """
    rows, cols = depth_img.shape
    v, u = np.indices((rows, cols))
    
    # 1. Depth Mask: Focus on the 'Action Zone' (1.0m to 2.5m)
    mask = (depth_img > 1.0) & (depth_img < 2.5)
    
    z = depth_img[mask]
    x = (u[mask] - CX) * z / FX
    y = (v[mask] - CY) * z / FY
    
    # 2. Refined Spatial ROI: Limits based on visual inspection of box motion
    # Horizontal (X): +/- 0.8m | Vertical (Y): -0.7m to +0.5m
    spatial_mask = (y < 0.5) & (y > -0.7) & (x < 0.8) & (x > -0.8)
    
    return np.column_stack((x[spatial_mask], y[spatial_mask], z[spatial_mask]))

def get_hull_area(points, normal):
    """
    Calculates the area of the visible face using 2D Convex Hull projection.
    """
    if len(points) < 3: return 0.0
    
    # Create a local 2D basis for the plane
    u_vec = np.cross(normal, [1, 0, 0])
    if np.linalg.norm(u_vec) < 1e-6: 
        u_vec = np.cross(normal, [0, 1, 0])
    u_vec /= np.linalg.norm(u_vec)
    v_vec = np.cross(normal, u_vec)
    
    # Project 3D points to the 2D plane basis
    proj = points @ np.column_stack((u_vec, v_vec))
    try:
        return ConvexHull(proj).volume # Volume of 2D hull is Area
    except:
        return 0.0

def process_bag():
    normals_history = []
    
    # Prepare the Results Table (CSV) with Image Number, Angle, Area, and Normal Vector
    with open('results_table.csv', 'w', newline='') as f:
        fieldnames = ['Image_Number', 'Angle_deg', 'Area_m2', 'Normal_Vector']
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        print(f"{'Img':<4} | {'Angle (deg)':<12} | {'Area (m^2)'} | {'Normal'}")
        print("-" * 70)

        with AnyReader([BAG_PATH]) as reader:
            connections = [x for x in reader.connections if x.topic == TOPIC_NAME]
            if not connections:
                print(f"Error: Topic {TOPIC_NAME} not found.")
                return

            for idx, (conn, ts, raw) in enumerate(reader.messages(connections=connections)):
                msg = reader.deserialize(raw, conn.msgtype)
                depth = np.ndarray(shape=(msg.height, msg.width), dtype=np.uint16, buffer=msg.data)
                
                # SI Unit Check: Handle mm (uint16) to meters conversion
                depth_m = depth.astype(np.float32) / 1000.0 if np.max(depth) > 100 else depth.astype(np.float32)

                # Process Geometry: ROI Crop and 3D Projection
                pts = depth_to_pointcloud(depth_m)
                if len(pts) < 100:
                    continue

                # RANSAC Plane Detection (Find the largest visible face)
                pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts))
                plane, inliers = pcd.segment_plane(0.02, 3, 1000)
                
                # Orient normal to face the camera (Negative Z direction)
                normal = np.array(plane[:3])
                if normal[2] > 0: normal = -normal
                
                # Calculate Metrics (Angle, Visible Area)
                angle = np.degrees(np.arccos(np.clip(np.dot(normal, [0,0,1]), -1, 1)))
                inlier_pts = np.asarray(pcd.select_by_index(inliers).points)
                area = get_hull_area(inlier_pts, normal)
                
                # Output to Terminal and CSV
                print(f"{idx:<4} | {angle:<12.2f} | {area:<10.4f} | {normal}")
                writer.writerow({
                    'Image_Number': idx, 
                    'Angle_deg': round(angle, 4), 
                    'Area_m2': round(area, 4),
                    'Normal_Vector': list(normal)
                })
                
                normals_history.append(normal)

    # Output Rotation Axis using Singular Value Decomposition (SVD)
    if len(normals_history) > 2:
        vecs = np.array(normals_history)
        centered = vecs - np.mean(vecs, axis=0)
        u, s, vh = np.linalg.svd(centered)
        axis = vh[2, :] # Smallest variance direction is the axis
        
        # Save results to rotation_axis.txt
        with open("rotation_axis.txt", "w") as f:
            f.write(f"Axis of Rotation Vector (Camera Frame): {axis.tolist()}\n")
            f.write(f"Magnitude: {np.linalg.norm(axis)}\n")
            f.write("Note: Derived via SVD of surface normal trajectory.")
            
        print(f"\nFinal Rotation Axis: {axis}")
        print("Deliverables 'results_table.csv' and 'rotation_axis.txt' are ready.")

if __name__ == "__main__":
    process_bag()