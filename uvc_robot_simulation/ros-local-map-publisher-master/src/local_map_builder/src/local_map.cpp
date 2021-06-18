#include "local_map_include.h"
using namespace std;
nav_msgs::OccupancyGrid global_map;
nav_msgs::Odometry pose;
pcl::PointCloud<pcl::PointXYZI> obstacle_pcl;
class localMap
{
private:
    nav_msgs::MapMetaData info;
    nav_msgs::Odometry m_pose;

public:
    localMap() {}
    localMap(nav_msgs::OccupancyGrid source_map)
    {
        this->info = source_map.info;
    }
    void initialize(nav_msgs::OccupancyGrid source_map)
    {
        this->info = source_map.info;
    }
    tf::StampedTransform transform;
    int x_axis;
    int y_axis;
    void set_xy(double x, double y)
    {
        x_axis = (int)(x / info.resolution);
        y_axis = (int)(y / info.resolution);
    }
    const char *m_source_frame;
    const char *m_child_frame;
    void set_transform(const char *source_frame, const char *child_frame, double tolerance);
    void Get_local_map(nav_msgs::OccupancyGrid *local_map, std::string method);
    void costmap(nav_msgs::OccupancyGrid *local_map, double lane_boundary);
    void set_pose(nav_msgs::Odometry arg_pose);
};

void localMap::costmap(nav_msgs::OccupancyGrid *local_map, double lane_boundary)
{
    //TODO: cost map
    int lane_boundary_pixel = (int)(lane_boundary / local_map->info.resolution);
    cv::Mat temp_map_image(local_map->info.width, local_map->info.height, CV_8UC1, cv::Scalar(0));
    cv::Mat temp_obstacle_image(local_map->info.width, local_map->info.height, CV_8UC1, cv::Scalar(0));
    for (int i = 0; i < local_map->info.width; i++)
    {
        for (int j = 0; j < local_map->info.height; j++)
        {
            int local_index = local_map->info.width * local_map->info.height - (j * local_map->info.width + i) - 1;
            temp_map_image.at<unsigned char>(i, j) = (int)local_map->data.at(local_index);
        }
    }
    for (int i = 0; i < local_map->info.width; i++)
    {
        for (int j = 0; j < local_map->info.height; j++)
        {
            int local_index = local_map->info.width * local_map->info.height - (j * local_map->info.width + i) - 1;
            if ((int)local_map->data.at(local_index) > 20 || (int)local_map->data.at(local_index) == -1)
            {
                cv::circle(temp_map_image, cv::Point(j, i), lane_boundary_pixel, cv::Scalar(100), CV_FILLED, 8);
            }
        }
    }
    for (int i = 0; i < local_map->info.width; i++)
    {
        for (int j = 0; j < local_map->info.height; j++)
        {
            int local_index = local_map->info.width * local_map->info.height - (j * local_map->info.width + i) - 1;
            local_map->data.at(local_index) = (int)temp_map_image.at<unsigned char>(i, j);
        }
    }
}

void localMap::set_transform(const char *source_frame, const char *target_frame, double tolerance)
{
    m_source_frame = source_frame;
    m_child_frame = target_frame;
    tf::TransformListener listener;
    tf::StampedTransform transform;
    try
    {
        // listener.lookupTransform(target_frame, source_frame, ros::Time(0), transform);
        listener.waitForTransform(source_frame, target_frame, ros::Time(0), ros::Duration(tolerance));
        listener.lookupTransform(source_frame, target_frame, ros::Time(0), transform);
        // cout << "tf!\n";
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    this->transform = transform;
}

void localMap::set_pose(nav_msgs::Odometry arg_pose)
{
    this->m_pose = arg_pose;
}
/*
tf method
Get_local_map(nav_msgs::OccupancyGrid *local_map)
*/
void localMap::Get_local_map(nav_msgs::OccupancyGrid *local_map, std::string method)
{
    //TODO: info = this->info
    // header = this->header;
    local_map->header.stamp = ros::Time::now();
    local_map->header.frame_id = "/base_footprint";
    local_map->info.resolution = info.resolution;
    local_map->info.height = 2 * x_axis;
    local_map->info.width = 2 * y_axis;
    local_map->info.origin.orientation.w = 1.0;
    local_map->info.map_load_time = ros::Time::now();
    // local_map->info.resolution = global_map.info.resolution;
    //TODO: maybe some problem in here! - origin
    // local_map->info.origin.position.x = (-(double)x_axis*(info.resolution));
    // local_map->info.origin.position.y = (-(double)y_axis*(info.resolution));
    local_map->info.origin.position.y = (-(double)x_axis * (info.resolution));
    local_map->info.origin.position.x = (-(double)y_axis * (info.resolution));
    local_map->data.assign(x_axis * 2 * y_axis * 2, 0);
    double current_x;
    double current_y;
    double current_th;
    tf::Quaternion q;
    double roll, pitch, yaw;
    if (method == "odom")
    {
        current_x = pose.pose.pose.position.x;
        current_y = pose.pose.pose.position.y;
        q.setW(pose.pose.pose.orientation.w);
        q.setX(pose.pose.pose.orientation.x);
        q.setY(pose.pose.pose.orientation.y);
        q.setZ(pose.pose.pose.orientation.z);
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        current_th = yaw;
    }
    else if (method == "tf")
    {
        current_x = this->transform.getOrigin().getX();
        current_y = this->transform.getOrigin().getY();
        q = this->transform.getRotation();
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        current_th = yaw;
    }

    double resolution = local_map->info.resolution;
    double cos_th = cos(current_th);
    double sin_th = sin(current_th);
    double robot_x, robot_y, rot_x, rot_y, global_x, global_y;
    robot_x = y_axis;
    robot_y = x_axis;
    unsigned int map_x, map_y;
    int map_index, local_index;
    for (int i = 0; i < local_map->info.width; i++)
    {
        for (int j = 0; j < local_map->info.height; j++)
        {
            rot_x = cos_th * (((double)i - robot_x) * resolution) - sin_th * (((double)j - robot_y) * resolution);
            rot_y = sin_th * (((double)i - robot_x) * resolution) + cos_th * (((double)j - robot_y) * resolution);
            map_x = ((current_x - rot_x) - global_map.info.origin.position.x) / resolution;
            map_y = ((current_y - rot_y) - global_map.info.origin.position.y) / resolution;
            map_index = map_y * global_map.info.width + map_x;
            local_index = local_map->info.width * local_map->info.height - (j * local_map->info.width + i) - 1;
            unsigned int original_map_data;
            if (map_index >= global_map.info.height * global_map.info.width)
            {
                original_map_data = -1;
            }
            else
            {
                original_map_data = global_map.data.at(map_index);
            }
            local_map->data.at(local_index) = original_map_data;
        }
    }
}

void mapCallback(const nav_msgs::OccupancyGrid msg)
{
    global_map = msg;
    cout << "map callback!!\n\n\n";
}

void odomCallback(const nav_msgs::Odometry msg)
{
    pose = msg;
    // cout << "pose : " << pose.pose.pose.position.x << ", " << pose.pose.pose.position.y << "\n";
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "local_costmap");
    ros::NodeHandle n;
    ros::Publisher local_costmap_pub = n.advertise<nav_msgs::OccupancyGrid>("/local_map", 1);
    // set parameter
    std::string source_frame;
    std::string child_frame;
    std::string map_name;
    bool is_static_map;
    std::string method;
    std::string odom_topic;
    std::vector<double> map_size;
    double tolerance;
    double cost_meter;
    int update_frequency;
    n.getParam("/source_frame", source_frame);
    n.getParam("/child_frame", child_frame);
    n.getParam("/map_topic", map_name);
    n.getParam("global_map_static", is_static_map);
    n.getParam("method", method);
    n.getParam("odom_topic", odom_topic);
    n.getParam("map_size", map_size);
    n.getParam("tolerance", tolerance);
    n.getParam("cost", cost_meter);
    n.getParam("update_frequency", update_frequency);

    ros::Subscriber pose_sub = n.subscribe(odom_topic.c_str(), 10, odomCallback);
    ros::Subscriber global_map_sub = n.subscribe(map_name, 10, mapCallback);
    sleep(1);
    //end set parameter
    // ros::Subscriber pose_sub = n.subscribe("/gps_utm_odom", 10, poseCallback);
    // ros::Subscriber obstacle_sub = n.subscribe("/Lidar/obj_pcl", 10, obstacleCallback);
    bool map_loaded = false;
    localMap local_costmap;
    if (is_static_map)
    {
        ros::ServiceClient map_client1 = n.serviceClient<nav_msgs::GetMap>("/static_map");
        nav_msgs::GetMap srv;
        cout << "static map!\n";
        if (map_client1.call(srv))
        {
            global_map = srv.response.map;
            global_map.data = srv.response.map.data;
            cout << global_map.info.width << " x " << global_map.info.height << endl;
            map_loaded = true;
            cout << "map loaded!\n";
        }
        else
        {
            cout << "map load fail!\n";
            return 1;
        }
        local_costmap.initialize(global_map);
        local_costmap.set_xy(map_size.at(1), map_size.at(0));
    }
    else
    {
        cout << "publishing map!\n";
        // ros::Subscriber global_map_sub = n.subscribe(map_name, 10, mapCallback);
    }

    std::string file_path = ros::package::getPath("global_path_planner");
    ros::Rate r(update_frequency); //frequency
    bool state_ok = true;
    while (ros::ok())
    {
        ros::Time tic = ros::Time::now();
        tf::StampedTransform transform;
        if (!is_static_map)
        {
            local_costmap.initialize(global_map);
            local_costmap.set_xy(map_size.at(1), map_size.at(0));
        }
        if (method == "tf")
        {
            local_costmap.set_transform(source_frame.c_str(), child_frame.c_str(), tolerance);
        }
        nav_msgs::OccupancyGrid temp;
        local_costmap.Get_local_map(&temp, method);
        local_costmap.costmap(&temp, cost_meter);
        //TODO: combined method
        local_costmap_pub.publish(temp);
        ros::Time toc = ros::Time::now();
        ros::Duration cost_time = toc - tic;
        cout << "cost : " << cost_time << "\n";
        r.sleep();
        ros::spinOnce();
    }
    ros::spin();
    return 0;
}
