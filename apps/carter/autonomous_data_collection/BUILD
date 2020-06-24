load("//engine/build:isaac.bzl", "isaac_app")

isaac_app(
    name = "carter_data_collection",
    data = [
        "//apps/assets/maps",
        "//apps/carter:2d_carter_subgraph",
        "//apps/carter/robots",
        "//packages/navigation/apps:differential_base_commander_subgraph",
        "//packages/navigation/apps:differential_base_navigation_subgraph",
        "//packages/record_replay/apps:record_subgraph",
    ],
    modules = [
        "imu",
        "navigation",
        "perception",
        "planner",
        "realsense",
        "rgbd_processing",
        "segway",
        "sensors:joystick",
        "velodyne_lidar",
        "viewers",
    ]
)