//
// Created by soulde on 24-3-18.
//

#ifndef R3LIVE_LIDAR_FRONT_END_H
#define R3LIVE_LIDAR_FRONT_END_H

#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/sensor_msgs/msg/point_cloud2.h>
#include <livox_ros_driver2/msg/custom_msg.hpp>


using namespace std;

#define IS_VALID(a) ( ( abs( a ) > 1e8 ) ? true : false )

typedef pcl::PointXYZINormal PointType;


enum LID_TYPE {
    MID,
    HORIZON,
    VELO16,
    OUST64
};

enum Feature {
    Nor,
    Poss_Plane,
    Real_Plane,
    Edge_Jump,
    Edge_Plane,
    Wire,
    ZeroPoint
};
enum Surround {
    Prev,
    Next
};
enum E_jump {
    Nr_nor,
    Nr_zero,
    Nr_180,
    Nr_inf,
    Nr_blind
};

struct orgtype {
    double range;
    double dista;
    double angle[2];
    double intersect;
    E_jump edj[2];
    Feature ftype;

    orgtype() {
        range = 0;
        edj[Prev] = Nr_nor;
        edj[Next] = Nr_nor;
        ftype = Nor;
        intersect = 2;
    }
};


class LidarFrontEnd : public rclcpp::Node {
public:
    LidarFrontEnd();

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_full, pub_surf, pub_corn;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_points_livox;
    const double rad2deg = 180 * M_1_PI;

    int lidar_type;
    double blind, inf_bound;
    int N_SCANS;
    int group_size;
    double disA, disB;
    double limit_maxmid, limit_midmin, limit_maxmin;
    double p2l_ratio;
    double jump_up_limit, jump_down_limit;
    double cos160;
    double edgea, edgeb;
    double smallp_intersect, smallp_ratio;
    int point_filter_num;
    int g_if_using_raw_point = 1;
    int g_LiDAR_sampling_point_step = 3;

    void mid_handler(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    void horizon_handler(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);

    void velo16_handler(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    void velo16_handler1(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    void oust64_handler(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    void give_feature(pcl::PointCloud<PointType> &pl, vector<orgtype> &types, pcl::PointCloud<PointType> &pl_corn,
                      pcl::PointCloud<PointType> &pl_surf);

    void pub_func(pcl::PointCloud<PointType> &pl, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
                  const rclcpp::Time &ct);

    int plane_judge(const pcl::PointCloud<PointType> &pl, vector<orgtype> &types, uint i, uint &i_nex,
                    Eigen::Vector3d &curr_direct);

    bool small_plane(const pcl::PointCloud<PointType> &pl, vector<orgtype> &types, uint i_cur, uint &i_nex,
                     Eigen::Vector3d &curr_direct);

    bool edge_jump_judge(const pcl::PointCloud<PointType> &pl, vector<orgtype> &types, uint i, Surround nor_dir);
};

#endif //R3LIVE_LIDAR_FRONT_END_H
