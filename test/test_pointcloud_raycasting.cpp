#include <ros/ros.h>
#include "ESDF_Map.h"
#include "raycast.h"
#include <vector>
#include <iostream>
#include <Eigen/Eigen>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <algorithm>
#include <unordered_set>
#include <omp.h>
#include <thread>
#include <tuple>
#include <queue>

#include "timing.h"

#define COW_AND_LADY
//#define SIGNED_NEEDED
using std::cout;
using std::endl;
using std::vector;
using std::thread;
ros::Publisher occ_pub;
ros::Publisher dist_pub;
ros::Publisher slice_pub;
ros::Publisher occ_pointcloud_pub, pose_pub;
Eigen::Vector3d pos, s_pos, origin, c_pos;
Eigen::Quaterniond q, s_q;
int count = 0;
double resolution;
Eigen::Vector3d lCornor, rCornor, map_size;
ESDF_Map *esdf_map;
Eigen::Matrix4d T_B_C, T_D_B, T;
Eigen::Matrix3d R_B_C, R_D_B;
Eigen::Vector3d t_B_C, t_D_B;
double min_ray_length_m = 0.1;
double max_ray_length_m = 8.0;
#ifdef SIGNED_NEEDED
ESDF_Map *inv_esdf_map;
#endif
double xMin, yMin, zMin, xMax, yMax, zMax;
std::queue<std::tuple<ros::Time, Eigen::Vector3d, Eigen::Quaterniond>> transformQueue;
std::queue<sensor_msgs::PointCloud2::ConstPtr> imageQueue;

visualization_msgs::Marker occ_marker, slice_marker;

vector<visualization_msgs::Marker> dis_markers;
#ifdef PROBABILISTIC
pcl::PointCloud<pcl::PointXYZRGB> cloud;
#else
pcl::PointCloud<pcl::PointXYZ> cloud;
#endif
sensor_msgs::PointCloud pc;
#ifndef PROBABILISTIC
sensor_msgs::PointCloud2::ConstPtr s_pc;
bool newMsg = false;
double range_sensor = 3.0;
Eigen::Vector3d radius(range_sensor, range_sensor, range_sensor / 2.0);
#endif

void visulization(ESDF_Map *esdf_map) {
//    timing::Timing::Print(std::cout);
//    esdf_map->getOccupancyMarker(occ_marker, 0, Eigen::Vector4d(1.0, 0, 0, 0.8));
//    occ_pub.publish(occ_marker);
    cout << "VISULIZATION" << endl;
    esdf_map->getPointCloud(pc);
    occ_pointcloud_pub.publish(pc);
//    Eigen::Vector3i s_vox;
//    esdf_map->pos2vox(c_pos, s_vox);
    esdf_map->getSliceMarker(slice_marker, 0, 100, Eigen::Vector4d(0, 1.0, 0, 1));
    slice_pub.publish(slice_marker);

    // visualize distance field
//    esdf_map->getESDFMarker(dis_markers, 1, Eigen::Vector3d(0.0, 1.0, 0));
//    for (int i = 0; i < int(dis_markers.size()); ++i) {
//        dist_pub.publish(dis_markers[i]);
//        ros::Duration(0.1).sleep();
//    }
//    ros::Duration(1.0).sleep();
}

#ifdef HASH_TABLE
std::unordered_set<int> uset, uset_s;
#else
std::vector<int> uset, uset_s;
#endif
int tot = 0;

#ifdef PROBABILISTIC

void raycastProcess(int i, int part, int tt) {

    Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
    for (int idx = part * i; idx < part * (i + 1); idx++) {
        std::vector<Eigen::Vector3d> output;
        if (idx > cloud.points.size()) break;
        pcl::PointXYZRGB pt = cloud.points[idx];
        int cnt = 0;
        if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z))
            continue;
        Eigen::Vector4d tmp = T * Eigen::Vector4d(pt.x, pt.y, pt.z, 1);
        Eigen::Vector3d point = Eigen::Vector3d(tmp(0), tmp(1), tmp(2)) / tmp(3);

//        Eigen::Vector3d point = Eigen::Vector3d(pt.x, pt.y, pt.z);
        int tmp_idx;
        double length = (point - origin).norm();
        if (length >= min_ray_length_m && length <= max_ray_length_m)
            tmp_idx = esdf_map->setOccupancy((Eigen::Vector3d) point, 1);
#ifdef SIGNED_NEEDED
        tmp_idx = inv_esdf_map->setOccupancy((Eigen::Vector3d) point, 0);
#endif
        if (tmp_idx != -1) {
#ifdef HASH_TABLE
            if (uset_s.find(tmp_idx) != uset_s.end())
                continue;
            else uset_s.insert(tmp_idx);
#else
            if (uset_s[tmp_idx] == tt)
                continue;
            else uset_s[tmp_idx] = tt;
#endif
        }

        Raycast(origin / resolution - half, point / resolution - half, lCornor, rCornor, &output);

        for (int i = output.size() - 2; i >= 0; i--) {
            Eigen::Vector3d tmp = (output[i] + half) * resolution;

            length = (tmp - origin).norm();
            if (length < min_ray_length_m)
                break;
            if (length > max_ray_length_m)
                continue;
            int tmp_idx;
            tmp_idx = esdf_map->setOccupancy(tmp, 0);
#ifdef SIGNED_NEEDED
            tmp_idx = inv_esdf_map->setOccupancy(tmp, 1);
#endif
            if (tmp_idx != -1) {
#ifdef HASH_TABLE
                if (uset.find(tmp_idx) != uset.end()) {
                    if (++cnt >= 1) break;
                } else {
                    uset.insert(tmp_idx);
                    cnt = 0;
                }
#else
                if (uset[tmp_idx] == tt) {
                    if (++cnt >= 1) break;
                } else {
                    uset[tmp_idx] = tt;
                    cnt = 0;
                }
#endif
            }
        }
    }

}

#endif
//void pointcloudCallBack(const sensor_msgs::PointCloud2::ConstPtr &pointcloud_map) {
////    cout << "POINTCLOUD: " << pointcloud_map->header.stamp << endl;
//    ros::Time t1, t2, t3;
//
//    int tt = ++tot;
//    cout << "PC: " << tt << endl;
//}

void pointcloudCallBack(const sensor_msgs::PointCloud2::ConstPtr &pointcloud_map) {
//    cout << "POINTCLOUD: " << pointcloud_map->header.stamp << endl;
    ros::Time t1, t2, t3;

    int tt = ++tot;
    cout << "PC: " << tt << endl;
    imageQueue.push(pointcloud_map);

    //if (cnt == 1500) std::thread(save).join();
    // std::cout << "Image timestamp\t" << msg->header.stamp << "\t" << std::endl;

    Eigen::Vector3d _position;
    Eigen::Quaterniond _orientation;
    ros::Time _poseTime, _imageTime;
    while (!imageQueue.empty()) {
        bool newPos = false;
        _imageTime = imageQueue.front()->header.stamp;
        while (transformQueue.size() > 1 && std::get<0>(transformQueue.front()) <= _imageTime + ros::Duration(1e-3)) {
            _poseTime = std::get<0>(transformQueue.front());
            _position = std::get<1>(transformQueue.front());
            _orientation = std::get<2>(transformQueue.front());
            transformQueue.pop();
            newPos = true;
        }
        if (transformQueue.empty() || std::get<0>(transformQueue.front()) <= _imageTime + ros::Duration(1e-3)) {
            break;
        }
        if (!newPos) {
            imageQueue.pop();
            continue;
        }
#ifndef PROBABILISTIC
        s_pc = pointcloud_map;
        newMsg = true;

        return;
#else
        T.block<3, 3>(0, 0) = s_q.normalized().toRotationMatrix();
        T.block<3, 1>(0, 3) = s_pos;
        T(3, 0) = T(3, 1) = T(3, 2) = 0;
        T(3, 3) = 1;
//    cout << R << endl;
        T = T * T_D_B * T_B_C;
        origin = Eigen::Vector3d(T(0, 3), T(1, 3), T(2, 3)) / T(3, 3);


        pcl::fromROSMsg(*pointcloud_map, cloud);

        if ((int) cloud.points.size() == 0)
            return;

        // TODO: when using vector, this is not needed
#ifdef HASH_TABLE
        uset.clear();
        uset_s.clear();
#endif
        cout << cloud.points.size() << endl;
        timing::Timer raycastingTimer("raycasting");
//#pragma omp parallel for
//    for (int idx = 0; idx < (int) cloud.points.size(); idx++) {
//        std::vector<Eigen::Vector3d> output;
//        pcl::PointXYZRGB pt = cloud.points[idx];
//        if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z))
//            continue;
//        Eigen::Vector3d point = R * Eigen::Vector3d(pt.x, pt.y, pt.z) + s_pos;
//        Raycast(s_pos / resolution - half, point / resolution - half, lCornor, rCornor, &output);
//    }
        t1 = ros::Time::now();
#ifdef HASH_TABLE
        int numThread = 1;
        int part = cloud.points.size() / numThread;
        raycastProcess(0, part, tt);
#else
        int numThread = std::thread::hardware_concurrency();
        int part = cloud.points.size() / numThread;
    //
        std::list<std::thread> integration_threads;

        for (size_t i = 0; i < numThread; ++i) {
            integration_threads.emplace_back(&raycastProcess, i, part, tt);
        }

    //    cout << integration_threads.size() << endl;
        for (std::thread &thread : integration_threads) {
            thread.join();
        }
#endif

        raycastingTimer.Stop();
        t2 = ros::Time::now();
//    if (tot%10 == 0) {

//    }
        t3 = ros::Time::now();
        cout << (t2 - t1).toSec() << "s\t" << (t3 - t2).toSec() << "s\t" << (t3 - t1).toSec() << 's' << endl;

//    if (!esdf_map->check()) {
//        cout << "Fatal Error !!" << endl;
//        exit(-1);
//    }

        ++count;

        if (count % 10 == 0) {
            cout << "Count: " << count << endl;
//        cout << xMin << ' ' << yMin << ' ' << zMin << endl;
//        cout << xMax << ' ' << yMax << ' ' << zMax << endl;
//        visulization(esdf_map);
        }
        imageQueue.pop();
    }
#endif
}

#ifdef PROBABILISTIC

void transformCallback(const geometry_msgs::TransformStamped::ConstPtr &msg) {
    cout << "TRANSFORM: " << msg->header.stamp << endl;
    pos = Eigen::Vector3d(msg->transform.translation.x,
                          msg->transform.translation.y,
                          msg->transform.translation.z);
    q = Eigen::Quaterniond(msg->transform.rotation.w,
                           msg->transform.rotation.x,
                           msg->transform.rotation.y,
                           msg->transform.rotation.z);
    transformQueue.push(std::make_tuple(msg->header.stamp, pos, q));


//
//    if (++count % 100 == 0) {
//        cout << count << endl;
//        cout << xMin << ' ' << yMin << ' ' << zMin << endl;
//        cout << xMax << ' ' << yMax << ' ' << zMax << endl;
//    }
}

#else
void transformCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    cout << "TRANSFORM: " << msg->header.stamp << endl;
    pos = Eigen::Vector3d(msg->pose.pose.position.x,
                          msg->pose.pose.position.y,
                          msg->pose.pose.position.z);
    q = Eigen::Quaterniond(msg->pose.pose.orientation.w,
                           msg->pose.pose.orientation.x,
                           msg->pose.pose.orientation.y,
                           msg->pose.pose.orientation.z);
    transformQueue.push(std::make_tuple(msg->header.stamp, pos, q));

}
#endif
int esdf_cnt = 0;

void updateESDFEvent(const ros::TimerEvent & /*event*/) {
#ifndef PROBABILISTIC
    if (!newMsg) return;
    newMsg = false;
    timing::Timer handlePCTimer("handlePointCloud");
    pcl::fromROSMsg(*s_pc, cloud);

    c_pos = s_pos;
    esdf_map->setUpdateRange(c_pos - radius, c_pos + radius);
    esdf_map->setAway();

    Eigen::Vector3i tmp_vox;
    Eigen::Vector3d tmp_pos;
    for (int i = 0; i < cloud.size(); i++) {
        tmp_pos = Eigen::Vector3d(cloud[i].x, cloud[i].y, cloud[i].z);
        esdf_map->setOccupancy(tmp_pos);
    }
    esdf_map->setBack();
    handlePCTimer.Stop();
#endif
    esdf_cnt++;
    cout << "TOT = " << esdf_cnt << endl;
//    if (esdf_map->checkUpdate()) {
    if (esdf_map->checkUpdate()) {
        timing::Timer updateESDFTimer("updateESDF");
        esdf_map->updateOccupancy();
//        Eigen::Vector3i s_vox;
//        esdf_map->pos2vox(s_pos, s_vox);
        esdf_map->updateESDF();
#ifdef SIGNED_NEEDED
        inv_esdf_map->pos2vox(s_pos, s_vox);
        inv_esdf_map->updateESDF(s_vox);
#endif
        updateESDFTimer.Stop();
        timing::Timing::Print(std::cout);
//        if (!esdf_map->checkWithGT()){
//            exit(0);
//        }
        if (esdf_cnt % 1 == 0) {
            visulization(esdf_map);
        }
    }
//    if (esdf_cnt == 300) {
//        esdf_map->checkWithGT();
////            exit(0);
//    }
//        visulization(esdf_map);
//    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Pointcloud_Raycasting_Test");
    ros::NodeHandle node;
//    cout << std::thread::hardware_concurrency() << endl;

    xMin = yMin = zMin = 1e100;
    xMax = yMax = zMax = -1e100;

    double map_size_ = 40.0;
    lCornor << -map_size_ / 2, -map_size_ / 2, 0.0;
    rCornor << map_size_ / 2, map_size_ / 2, 5.0;

//    lCornor << -11.0, -11.0, -7.0;
//    rCornor << 11.0, 11.0, 7.0;
    map_size = rCornor - lCornor;
    resolution = 0.10;
#ifdef HASH_TABLE
    esdf_map = new ESDF_Map(Eigen::Vector3d(0, 0, 0), resolution, 100);
#ifdef SIGNED_NEEDED
    inv_esdf_map = new ESDF_Map(Eigen::Vector3d(0, 0, 0), resolution, 10000000);
#endif
#else
    esdf_map = new ESDF_Map(lCornor, resolution, map_size);
#ifdef SIGNED_NEEDED
    inv_esdf_map = new ESDF_Map(lCornor, resolution, map_size);
#endif
#endif
    // to be used in raycasting
    lCornor /= resolution;
    rCornor /= resolution;
#ifndef HASH_TABLE
    uset.resize(esdf_map->grid_total_size);
    uset_s.resize(esdf_map->grid_total_size);
    std::fill(uset.begin(), uset.end(), 0);
    std::fill(uset_s.begin(), uset_s.end(), 0);
#endif
//#ifdef COW_AND_LADY
//    ros::Subscriber subpoints = node.subscribe("/camera/depth_registered/points", 1000, pointcloudCallBack);
//    ros::Subscriber sub = node.subscribe("/kinect/vrpn_client/estimated_transform", 1000, transformCallback);
//    T_B_C << 1, 0, 0, 0,
//            0, 1, 0, 0,
//            0, 0, 1, 0,
//            0, 0, 0, 1;
//    T_D_B << 0.971048, -0.120915, 0.206023, 0.00114049,
//            0.15701, 0.973037, -0.168959, 0.0450936,
//            -0.180038, 0.196415, 0.96385, 0.0430765,
//            0.0, 0.0, 0.0, 1.0;
//#else
////    ros::Subscriber subpoints = node.subscribe("/dense_stereo/pointcloud", 1000, pointcloudCallBack);
////    ros::Subscriber sub = node.subscribe("/vicon/firefly_sbx/firefly_sbx", 1000, transformCallback);
////    T_B_C << 0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
////            0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
////            -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
////            0.0, 0.0, 0.0, 1.0;
////    T_D_B << 0.33638, -0.01749, 0.94156, 0.06901,
////            -0.02078, -0.99972, -0.01114, -0.02781,
////            0.94150, -0.01582, -0.33665, -0.12395,
////            0.0, 0.0, 0.0, 1.0;
////    T_D_B = T_D_B.inverse();
//
//
//    ros::Subscriber subpoints = node.subscribe("/camera/depth_registered/points", 1000, pointcloudCallBack);
//    ros::Subscriber sub = node.subscribe("kinect/vrpn_client/estimated_transform", 1000, transformCallback);
//    T_B_C << 1, 0, 0, 0,
//            0, 1, 0, 0,
//            0, 0, 1, 0,
//            0, 0, 0, 1;
//    T_D_B << 1, 0, 0, 0,
//            0, 1, 0, 0,
//            0, 0, 1, 0,
//            0, 0, 0, 1;
//#endif
    ros::Subscriber subpoints = node.subscribe("/camera/depth_registered/points", 1000, pointcloudCallBack);
    ros::Subscriber sub = node.subscribe("/kinect/vrpn_client/estimated_transform", 1000, transformCallback);
    T_B_C << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    T_D_B << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    occ_pub = node.advertise<visualization_msgs::Marker>("EDFS_Map/occ", 1, true);
    dist_pub = node.advertise<visualization_msgs::Marker>("EDFS_Map/dist", 1, true);
    slice_pub = node.advertise<visualization_msgs::Marker>("EDFS_Map/slice", 1, true);
    occ_pointcloud_pub = node.advertise<sensor_msgs::PointCloud>("EDFS_Map/occ_pc", 1, true);
//    pose_pub = node.advertise<visualization_msgs::Marker>("EDFS_Map/pose", 1, true);
    pose_pub = node.advertise<nav_msgs::Odometry>("EDFS_Map/pose", 10);

    float update_mesh_every_n_sec = 0.5f;
    ros::Timer update_mesh_timer_ =
            node.createTimer(ros::Duration(update_mesh_every_n_sec),
                             &updateESDFEvent);
    ros::spin();
    delete esdf_map;
#ifdef SIGNED_NEEDED
    delete inv_esdf_map;
#endif

    return 0;
}