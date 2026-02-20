import numpy as np
import open3d as o3d
from scipy.spatial import ConvexHull

class PointCloudProcessor:
    def __init__(self, fx=525.0, fy=525.0, cx=319.5, cy=239.5):
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy

    def depth_to_pointcloud(self, depth_img, min_depth=0.5, max_depth=2.0):
        """
        Converts 2D depth image to 3D point cloud using vectorization.
        """
        rows, cols = depth_img.shape
        
        # Grid of (u, v) pixel coordinates
        v_grid, u_grid = np.indices((rows, cols))
        
        # Mask for valid pixels
        mask = (depth_img > min_depth) & (depth_img < max_depth)
        
        # Extract valid data
        z = depth_img[mask]
        u = u_grid[mask]
        v = v_grid[mask]
        
        # Pinhole Camera Model: Pixel -> 3D
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        
        # Combine into (N, 3) array
        points = np.column_stack((x, y, z))
        return points

    def fit_plane_and_measure(self, points):
        """
        Fits a plane using RANSAC and calculates metrics (Normal, Angle, Area).
        """
        if len(points) < 100:
            return None

        # 1. Open3D PointCloud creation
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # 2. RANSAC Plane Fitting
        # distance_threshold=0.01 (1cm tolerance)
        plane_model, inliers_idx = pcd.segment_plane(distance_threshold=0.01,
                                                     ransac_n=3,
                                                     num_iterations=1000)
        [a, b, c, d] = plane_model
        normal = np.array([a, b, c])

        # 3. Orient Normal towards Camera
        # The camera looks down +Z. A face looking AT the camera
        # has a normal pointing roughly -Z.
        if np.dot(normal, [0, 0, 1]) > 0:
            normal = -normal

        # 4. Calculate Angle vs Camera Axis (Z-axis)
        camera_z = np.array([0, 0, 1])
        # Clip for numerical stability
        cos_theta = np.clip(np.dot(normal, camera_z), -1.0, 1.0) 
        angle_deg = np.degrees(np.arccos(cos_theta))

        # 5. Calculate Visible Area (Convex Hull)
        # Extract only the points that belong to the plane
        inlier_cloud = pcd.select_by_index(inliers_idx)
        inlier_points = np.asarray(inlier_cloud.points)
        
        # Project 3D inliers to 2D plane to calculate area
        # Create a basis for the plane (u, v)
        # Find a vector 'u' orthogonal to normal
        u_vec = np.cross(normal, [1, 0, 0])
        if np.linalg.norm(u_vec) < 1e-6: # Handle edge case where normal is parallel to X
            u_vec = np.cross(normal, [0, 1, 0])
        u_vec = u_vec / np.linalg.norm(u_vec)
        
        # Find vector 'v' orthogonal to both
        v_vec = np.cross(normal, u_vec)
        
        # Project points onto these two vectors
        projected_2d = inlier_points @ np.column_stack((u_vec, v_vec))
        
        # Compute Convex Hull Area
        try:
            hull = ConvexHull(projected_2d)
            area = hull.volume # In 2D, scipy calls area "volume"
        except:
            area = 0.0

        return {
            "normal": normal,
            "angle_deg": angle_deg,
            "area_m2": area,
            "inlier_count": len(inliers_idx)
        }