import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from pathlib import Path
import numpy as np
import h5py
import gc
from tqdm import tqdm

for obj in gc.get_objects():   # Browse through ALL objects
    if isinstance(obj, h5py.File):   # Just HDF5 files
        try:
            obj.close()
        except:
            pass # Was already closed

to_type = {
  0 : "Unlabeled",
  1 : "Roads",
  2 : "SideWalks",
  3 : "Buildings",
  4 : "Wall",
  5 : "Fence",
  6 : "Pole",
  7 : "TrafficLight",
  8 : "TrafficSign",
  9 : "Vegetation",
  10 : "Terrain",
  11 : "Sky",
  12 : "Pedestrian",
  13 : "Rider",
  14 : "Car",
  15 : "Truck",
  16 : "Bus",
  17 : "Train",
  18 : "Motorcycle",
  19 : "Bicycle",
  20 : "Static",
  21 : "Dynamic",
  22 : "Other",
  23 : "Water",
  24 : "RoadLine",
  25 : "Ground",
  26 : "Bridge",
  27 : "RailTrack",
  28 : "GuardRail", 
}
to_id = {v: k for k, v in to_type.items()}

def plot_box(ax, bx, by, bz, bxextend, byextend, bzextend, broll, bpitch, byaw, color='r', upper_right_corner=True):
    #print(f"Box: {box}\n\n")
    
    cords = np.zeros((8, 4))

    cords[0, :] = np.array([bxextend, byextend, -bzextend, 1])
    cords[1, :] = np.array([-bxextend, byextend, -bzextend, 1])
    cords[2, :] = np.array([-bxextend, -byextend, -bzextend, 1])
    cords[3, :] = np.array([bxextend, -byextend, -bzextend, 1])
    cords[4, :] = np.array([bxextend, byextend, bzextend, 1])
    cords[5, :] = np.array([-bxextend, byextend, bzextend, 1])
    cords[6, :] = np.array([-bxextend, -byextend, bzextend, 1])
    cords[7, :] = np.array([bxextend, -byextend, bzextend, 1])

    yaw = np.deg2rad(-byaw)
    rotation_matrix = np.array([
        [np.cos(yaw), -np.sin(yaw), 0, 0],
        [np.sin(yaw), np.cos(yaw), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    cords = rotation_matrix @ cords.T
    cords = cords.T
    cords[:, 0] += bx
    cords[:, 1] += by#
    cords[:, 2] += bz
    # Plot the edges of the bounding box
    edges = [(0,1), (1,2), (2,3), (3,0),
            (4,5), (5,6), (6,7), (7,4),
            (0,4), (1,5), (2,6), (3,7)]
    for edge in edges:
        ax.plot([cords[edge[0], 0], cords[edge[1], 0]],
                [cords[edge[0], 1], cords[edge[1], 1]],
                [cords[edge[0], 2], cords[edge[1], 2]], c=color)
    if upper_right_corner:
        ax.scatter(cords[4, 0], cords[4, 1], cords[4, 2], c='r', s=5)

def create_pcl_video(pcl_points, output_file,bbox_file, topics, frames=None):
    with h5py.File(pcl_points, 'r') as f, h5py.File(bbox_file, 'r') as f_bbox:
        print(f"reading file {pcl_points}...")

        fig = plt.figure(figsize=(10,8))
        ax = fig.add_subplot(111, projection='3d')

        # Set axes limits (adjust based on your LiDAR range)
        #ax.view_init(elev=20, azim=180)
        ax.set_xlim(-50, 50)
        ax.set_ylim(-50, 50)
        ax.set_zlim(-5, 5)
        # turn off the axes
        ax.axis('off')
        ax.set_title('LiDAR 3D Top-Down View')

        # Initialize scatter (empty)
        scat = ax.scatter([], [], [], c=[], s=1, cmap='viridis')

        # Update function for animation
        def update(frame_idx):
            print(f"Processing frame {frame_idx}...")
            try:
                ax.cla()  # clear previous points
                for topic in topics:
                    
                    points = f[f"{topic}/frame_{frame_idx:06d}"]
                    boxes = list(f_bbox[f"frame_{frame_idx:06d}/actors"])
                    static_boxes = []
                    if "static" in f_bbox[f"frame_{frame_idx:06d}"]:
                        static_boxes = list(f_bbox[f"frame_{frame_idx:06d}/static"])
                    x = points["x"]
                    y = points["y"]
                    z = points["z"]
                    i = [to_id[point.decode('utf8')] for point in points["ObjTag"]]

                    ax.scatter(x, y, z, c=i, s=1, cmap='viridis')

                #plot bounding boxes no rotation needed
                for box in boxes:
                    id, t, bx, by, bz, bxextend, byextend, bzextend, broll, bpitch, byaw = box
                    plot_box(ax, bx, by, bz, bxextend, byextend, bzextend, broll, bpitch, byaw)
                for box in static_boxes:
                    id, t, bx, by, bz, bxextend, byextend, bzextend, broll, bpitch, byaw = box
                    plot_box(ax, bx, by, bz, bxextend, byextend, bzextend, broll, bpitch, byaw, 'b')
                ax.scatter(
                    5, 5, 5, c='r', s=20
                )
                ax.scatter(
                    5, -5, 5, c='r', s=20
                )
                ax.set_xlim(-10,20)
                ax.set_ylim(-10,20)
                ax.set_zlim(-10,10)
                ax.set_xlabel('X [m]')
                ax.set_ylabel('Y [m]')
                ax.set_zlabel('Z [m]')
                ax.set_title(f"LiDAR Frame {frame_idx}")
                return []
            except KeyError:
                print(f"Frame {frame_idx} not found in the dataset.")
                return []

        print("Create animation...")
        frames = frames if frames is not None else len(list(f_bbox.keys())[1:])-1
        ani = FuncAnimation(fig, update, frames=frames, interval=50)

        print("Save as MP4...")
        ani.save(output_file, writer='ffmpeg', fps=20)
        plt.close(fig)

import cv2
from cv_bridge import CvBridge
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_msgs.msg import Image
import rclpy.serialization
import yaml

def create_camera_video(db_file, topic_name, output_file, fps=20):
    bridge = CvBridge()

    # Setup ROS 2 bag reader
    storage_options = StorageOptions(uri=db_file, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # Get all topics
    topic_types = reader.get_all_topics_and_types()
    topics_dict = {t.name: t.type for t in topic_types}

    video_writer = None
    frame_count = 0

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        if topic == topic_name:
            # Deserialize message
            img_msg = rclpy.serialization.deserialize_message(data, Image)
            cv_image = bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
            if video_writer is None:
                height, width, _ = cv_image.shape
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                video_writer = cv2.VideoWriter(output_file, fourcc, fps, (width, height))

            video_writer.write(cv_image)
            frame_count += 1
    
    if video_writer:
        video_writer.release()

    print(f"MP4 video created: {output_file}, total frames: {frame_count}")

def extract_pcl_topics(metadata_fpath):
    with open(metadata_fpath, 'r') as f:
        metadata = yaml.safe_load(f)

    topic_metadata = metadata["rosbag2_bagfile_information"]["topics_with_message_count"]

    topic_list = [topic["topic_metadata"]["name"].replace("/", "_") for topic in topic_metadata if topic["topic_metadata"]["type"] == "sensor_msgs/msg/PointCloud2"]
    return topic_list

if __name__ == '__main__':
    
    
    bbox_dir = Path('/home/npopkov/repos/IR2025/data/20251127_1754_30v_10w_265sp/')
    db_dir = bbox_dir / "db/"
    topic_list = extract_pcl_topics(db_dir / "metadata.yaml")
    
    create_pcl_video(
        str(db_dir / "lidar_ego_data.h5"),
        str(db_dir / "lidar_3d_video.mp4"),
        str(bbox_dir / "bbox_ego.h5"),
        topic_list,
        frames=None
    )
    # create_camera_video(
    #     db_file = str(db_dir / 'rosbag2_2025_10_11-19_24_30_0.db3'),
    #     topic_name = "/carla/ego_vehicle/rgb_view/image",
    #     output_file = str(db_dir / "carla_camera.mp4")
    # )