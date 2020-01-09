# Fast NDT SLAM

## 1. 原理
基于NDT正态分布变换,实现点云配准,从而通过激光SLAM的方法进行点云建图

## 2. Params
|Name|Type|Description|
|:---|:---|:---|
| laser_frame | String | 激光雷达的frame,velodye为`/velodye`, 镭神为`/laser_link` |
| base_frame | String | 车辆控制中心的frame,统一为`/base_link` |
| lidar_topic | String | 接收的点云topic,可以是原始点云,也可以是处理后的点云,目前为原始点云,镭神`/lslidar_point_cloud` |
| odom_topic | String | 里程计(imu+速度计)融合后的位置,目前未加入 |
| use_odom | Bool | 是否使用里程计输入,目前未加入,为false |
| ndt_method_type | Int | 0:pcl 1:cpu 2:gpu 3:omp 4:gpu_ptr |
| ndt_resolution | Double | ndt分辨率 |
| ndt_step_size | Double | ndt搜索步长 |
| ndt_trans_eps | Double | ndt精度 |
| ndt_max_iter | Int | ndt最大迭代次数 |
| voxel_leaf_size| Double | 通用格栅体素大小 |
| incremental_voxel_update | Bool | 是否使用动态vexel_size |
| min_scan_range | Double | 最小scan距离,和max_scan_range一起裁剪点云 | 
| max_scan_range | Double | 最大scan距离 |
| min_add_scan_shift | Double | 最小更新全局地图的距离步长 |
| min_update_target_map | Double | 最小更新slam-ndt匹配用地图的距离步长 |
| global_voxel_leafsize | Double | 用于voxel最终生成的点云地图的体素格栅大小 |
| extract_length | Double | Double | 和extract_width一起,抽取匹配用地图 |
| extract_width | Double | 注意,抽取匹配用地图的方法将被废弃 |
| tf_timeout | Double | 等待tf变换所用最长时间 |
| is_publish_map_full | Bool | 是否发布全局地图(集合成一个文件, 实时发布) |
| if_create_piece_map | Bool | 是否按照给定距离,创建一系列地图文件(该地图文件被处理后可用于动态地图的加载) |
| piece_map_folder | String | 保存piece_map的路径,注意以`/`结尾 |
| piece_step | Double | 所创建piece_map每片的大小piece_step×piece_step |
| is_publish_map_for_costmap | Bool | 是否发布cost_map(该cost_map已被滤除地面,后期可用于生成全局格栅地图) |
| cost_map_folder| String | 保存cost_map的路径,注意以`/`结尾 |
| costmap_voxel_size| Double | voxel该地图所用体素大小,该值一般比其他voxel值要小 |
| is_publish_map_for_planning| Bool | 是否发布用于抽取全局行驶轨迹的地图(该地图高度上被极大抽取,仅包含道路特征) |
| planning_map_folder| String | 保存planning_map的路径,注意以`/`结尾 |


## 3. Subscriber/Publisher & Topic
**Subscriber**  

| Topic | Type | Name | Description |
| :---  | :--- | :--- | :--- |
| lidar_topic | sensor_msgs::PoinCloud2 | "/lslidar_point_cloud" | 输入的点云,本处为原始点云 |
| odom_topic | | |

**Publisher**  

| Topic | Type | Name | Description |
| :---  | :--- | :--- | :--- |
| current_pose_pub | geometry_msgs::PoseStamped | `/ndt/current_pose` | 当前位置 |
| refiltered_map_pub |  sensor_msgs::PoinCloud2 | `/globalmap/map_full` | 最终生成的全局点云地图(单一地图)(注:在目标保存文件夹下运行命令`rosrun pcl_ros pointcloud_to_pcd input:=/globalmap/map_full`保存) |


## 4. Usage
> 1. 设置输入topic, 包括`laser_frame`, `lidar_topic`
> 2. 设置是否使用odom作为输入(目前为false)
> 3. 设置是否发布单一全局地图`/globalmap/map_full`, 参数为`is_publish_map_full`
(若需要保存单一全局点云地图,则需要在目标保存文件夹下运行命令`rosrun pcl_ros pointcloud_to_pcd input:=/globalmap/map_full`保存)
> 4. 设置是否保存分片地图piece_map,设置`if_create_piece_map`以及`piece_map_folder`,`piece_step`
> 5. 设置是否保存滤除地面的cost_map及其相应保存文件夹和voxel大小
> 6. 设置是否保存用于提取行驶轨迹的planning_map及其相应保存文件夹

**若仅使用ndt_mapping,其他参数请谨慎修改**

## 5. 备注
1. 室外场景下,vocel_size可在1.0-2.0之间设置,以防止生成地图过于稠密;室内小场景可以设置为0.3-1.0之间

**190723**  
`/globalmap/map_full`保留,以较大的voxel值进行降采样,仅作为在建图过程中展示是否漂移只用,最终采用的地图由下列方式获得
```c++
1. 修改piece_map相关的参数,包括folder, voxel_size, piece_max_size
2. 使用map_tools/pcd_grid_divider对piece_map进行GRID处理
3. 使用时,使用动态地图加载grip_map
```

**190724**  
pcd-files中的点如果相互没有交叉(即使用pcd_grid_divider进行GRID之后), 其加载速度会多很多很多