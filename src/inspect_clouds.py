import open3d as o3d
import numpy as np
from pathlib import Path
from rosbags.highlevel import AnyReader
from geometry_utils import PointCloudProcessor

# ---------------- CONFIGURATION ----------------
BAG_FOLDER = Path('depth') 
TOPIC_NAME = '/depth'
# Intrinsics fallback
FX, FY, CX, CY = 525.0, 525.0, 319.5, 239.5

class DataInspector:
    def __init__(self):
        self.processor = PointCloudProcessor()
        self.frames = []
        self.current_frame_idx = 0
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.is_rotating = False
        
        print("Loading frames...")
        with AnyReader([BAG_FOLDER]) as reader:
            connections = [x for x in reader.connections if x.topic == TOPIC_NAME]
            for connection, timestamp, rawdata in reader.messages(connections=connections):
                msg = reader.deserialize(rawdata, connection.msgtype)
                depth_img = np.ndarray(shape=(msg.height, msg.width), dtype=np.uint16, buffer=msg.data)
                
                # Unit Verification: Convert mm to meters if uint16 max > 100
                depth_m = depth_img.astype(np.float32) / 1000.0 if np.max(depth_img) > 100 else depth_img.astype(np.float32)
                self.frames.append((timestamp, depth_m))
        print(f"Loaded {len(self.frames)} frames.")

    def get_geometry(self):
        """Process current frame and return geometries using LOOSENED ROI"""
        if not self.frames: return []
        timestamp, depth = self.frames[self.current_frame_idx]
        
        # LOOSENED ROI to prevent 'black gaps'
        # Depth: 1.0 to 2.5m | Vertical (Y): -0.7 to 0.5m | Horizontal (X): -0.8 to 0.8m
        points = self.processor.depth_to_pointcloud(depth)
        
        pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
        if len(points) < 100:
            pcd.paint_uniform_color([0.2, 0.2, 0.2])
            return [pcd]

        # RANSAC face detection
        plane, inliers = pcd.segment_plane(0.02, 3, 1000)
        
        # Orient normal toward camera
        normal = np.array(plane[:3])
        if normal[2] > 0: normal = -normal
        
        # Print metrics to terminal for live verification
        metrics = self.processor.fit_plane_and_measure(points)
        if metrics:
            print(f"\n--- FRAME {self.current_frame_idx} ---")
            print(f"Area: {metrics['area_m2']:.4f} m^2 | Angle: {metrics['angle_deg']:.2f}°")
            print(f"Normal: {metrics['normal']}")

        # Coloring: Red Box Face, Gray Noise
        box_cloud = pcd.select_by_index(inliers)
        box_cloud.paint_uniform_color([1, 0, 0]) 
        other_cloud = pcd.select_by_index(inliers, invert=True)
        other_cloud.paint_uniform_color([0.5, 0.5, 0.5]) 
        
        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=[0,0,0])
        return [box_cloud, other_cloud, axes]

    def update_view(self, vis):
        vis.clear_geometries()
        for g in self.get_geometry():
            vis.add_geometry(g, reset_bounding_box=False)
        return True 

    def next_frame(self, vis):
        self.current_frame_idx = (self.current_frame_idx + 1) % len(self.frames)
        return self.update_view(vis)

    def prev_frame(self, vis):
        self.current_frame_idx = (self.current_frame_idx - 1) % len(self.frames)
        return self.update_view(vis)

    def toggle_rotation(self, vis):
        self.is_rotating = not self.is_rotating
        return True

    def rotate_callback(self, vis):
        if self.is_rotating:
            vis.get_view_control().rotate(5.0, 0.0) 
            return True 
        return False

    def view_front(self, vis):
        ctr = vis.get_view_control()
        ctr.set_front([0, 0, -1]); ctr.set_lookat([0, 0, 1.5]); ctr.set_up([0, -1, 0]) 
        return True 

    def view_top(self, vis):
        ctr = vis.get_view_control()
        ctr.set_front([0, 1, 0]); ctr.set_lookat([0, 0, 1.5]); ctr.set_up([0, 0, 1]) 
        return True 

    def view_side(self, vis):
        ctr = vis.get_view_control()
        ctr.set_front([-1, 0, 0]); ctr.set_lookat([0, 0, 1.5]); ctr.set_up([0, -1, 0]) 
        return True 

    def run(self):
        self.vis.create_window(window_name="3D Final Inspection", width=1280, height=720)
        for g in self.get_geometry(): self.vis.add_geometry(g)
        self.vis.register_key_callback(ord('N'), self.next_frame)
        self.vis.register_key_callback(ord('P'), self.prev_frame)
        self.vis.register_key_callback(32, self.toggle_rotation) 
        self.vis.register_key_callback(ord('1'), self.view_front); self.vis.register_key_callback(ord('Z'), self.view_front)
        self.vis.register_key_callback(ord('2'), self.view_top); self.vis.register_key_callback(ord('Y'), self.view_top)
        self.vis.register_key_callback(ord('3'), self.view_side); self.vis.register_key_callback(ord('X'), self.view_side)
        self.vis.register_animation_callback(self.rotate_callback)
        self.vis.run(); self.vis.destroy_window()

if __name__ == "__main__":
    app = DataInspector(); app.run()