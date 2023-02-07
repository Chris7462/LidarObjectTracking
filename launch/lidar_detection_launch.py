from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from os.path import join


def generate_launch_description():
  params = join(
    get_package_share_directory("lidar_object_tracking"), "params", "lidar_detection_params.yaml"
  )

  ground_removal_node = Node(
    package="lidar_object_tracking",
    executable="ground_removal_node",
    name="ground_removal_node",
    parameters=[params]
  )

  cluster_object_node = Node(
    package="lidar_object_tracking",
    executable="cluster_object_node",
    name="cluster_object_node",
    parameters=[params]
  )

  bag_exec = ExecuteProcess(
    cmd=["ros2", "bag", "play", "/data/kitti_raw/ros2_bag/kitti_2011_09_26_drive_0014_synced"]
  )

# tf_node = Node(
#   package="tf2_ros",
#   executable="static_transform_publisher",
#   name="world2map_tf",
#   arguments=['0', '0', '0', '0', '0', '0', "world", "map"]
# )

  rviz_node = Node(
    package="rviz2",
    executable="rviz2",
    name="rviz2",
    arguments=["-d", join(get_package_share_directory("lidar_object_tracking"), "rviz/", "cluster.rviz")]
  )

  return LaunchDescription([
    ground_removal_node,
    cluster_object_node,
    bag_exec,
#   tf_node,
    rviz_node
  ])