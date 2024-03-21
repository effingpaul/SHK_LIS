# --- list of helperFunctions needed by me ---
# --- lastly update on march 21st 2024 ---

import numpy as np

def generate_cuboid_surface_points(center, dimensions, num_points_per_face):
    """
    Generate as point cloud of a 3D cuboid surface given its center, dimensions, and number of points per face.
    
    Args:
        center (tuple): Center coordinates of the cuboid in (x, y, z) format.
        dimensions (tuple): Dimensions of the cuboid in (length, width, height) format.
        num_points_per_face (int): Number of points to generate per face.
    
    Returns:
        np.array: Array of 3D points representing the cuboids surface.
    """
    length, width, height = dimensions
    x_min, y_min, z_min = center[0] - length / 2, center[1] - width / 2, center[2] - height / 2
    x_max, y_max, z_max = center[0] + length / 2, center[1] + width / 2, center[2] + height / 2

    # Generate points on each face
    x_face = np.linspace(x_min, x_max, num_points_per_face)
    y_face = np.linspace(y_min, y_max, num_points_per_face)
    z_face = np.linspace(z_min, z_max, num_points_per_face)

    # Generate points on other faces and convert to numpy arrays
    xy_points = np.array(np.meshgrid(x_face, y_face, [z_min, z_max])).T.reshape(-1, 3)
    xz_points = np.array(np.meshgrid(x_face, [y_min, y_max], z_face)).T.reshape(-1, 3)
    yz_points = np.array(np.meshgrid([x_min, x_max], y_face, z_face)).T.reshape(-1, 3)
    
    # Stack points together
    points = np.concatenate([xy_points, xz_points, yz_points])
    
    return points


def filter_points_outside_cuboid(point_cloud, cuboid_center, cuboid_dimensions):
    """
    Filter points from a point cloud that lie outside a fixed cuboid.
    
    Args:
        point_cloud (np.array): Array of 3D points in the shape (N, 3).
        cuboid_center (tuple): Center coordinates of the cuboid in (x, y, z) format.
        cuboid_dimensions (tuple): Dimensions of the cuboid in (length, width, height) format.
    
    Returns:
        np.array: Filtered array of points lying outside the cuboid.
    """
    length, width, height = cuboid_dimensions
    x_min, y_min, z_min = cuboid_center[0] - length / 2, cuboid_center[1] - width / 2, cuboid_center[2] - height / 2
    x_max, y_max, z_max = cuboid_center[0] + length / 2, cuboid_center[1] + width / 2, cuboid_center[2] + height / 2

    outside_cuboid_mask = np.logical_or.reduce((point_cloud[:, 0] < x_min, 
                                                point_cloud[:, 0] > x_max,
                                                point_cloud[:, 1] < y_min, 
                                                point_cloud[:, 1] > y_max,
                                                point_cloud[:, 2] < z_min, 
                                                point_cloud[:, 2] > z_max))
    
    return point_cloud[outside_cuboid_mask]