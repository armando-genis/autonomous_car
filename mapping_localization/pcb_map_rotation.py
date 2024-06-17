import open3d as o3d
import numpy as np
import os

def translate_point_cloud_z(pcd_path, translation_z, save_path):
    # Load the point cloud
    pcd = o3d.io.read_point_cloud(pcd_path)

    # Define the translation vector: No change in x, no change in y, increase z by 1.8 meters
    translation = np.array([0, 0, translation_z])

    # Translate the point cloud
    pcd.translate(translation, relative=True)

    # Save the translated point cloud
    o3d.io.write_point_cloud(save_path, pcd)
    print(f"Translated point cloud saved to: {save_path}")

# Path to the original point cloud
pcd_path = "/home/genis/ros2_ws/src/autonomous_car/pcd_maps/cloudGlobal.pcd"

# Amount to translate in the Z direction (1.8 meters upwards)
translation_z = 1.8  # Adjust this as needed

# Path to save the translated point cloud
save_path = os.path.join(os.path.dirname(pcd_path), "translated_cloudGlobal.pcd")

# Translate the point cloud
translate_point_cloud_z(pcd_path, translation_z, save_path)
