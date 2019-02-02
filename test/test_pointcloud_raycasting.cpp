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
#include <algorithm>
#include <unordered_set>
#include <omp.h>
#include <thread>
#include <tuple>
#include <queue>

#include "timing.h"

using std::cout;
using std::endl;
using std::vector;
using std::thread;
ros::Publisher occ_pub;
ros::Publisher dist_pub;
Eigen::Vector3d pos, s_pos;
Eigen::Quaterniond q, s_q;
int count = 0;
double resolution;
Eigen::Vector3d lCornor, rCornor, map_size;
ESDF_Map *esdf_map;
double xMin, yMin, zMin, xMax, yMax, zMax;
std::queue<std::tuple<ros::Time, Eigen::Vector3d, Eigen::Quaterniond>> transformQueue;
visualization_msgs::Marker occ_marker;

vector<visualization_msgs::Marker> dis_markers;
pcl::PointCloud<pcl::PointXYZRGB> cloud;

void visulization(ESDF_Map esdf_map) {
    timing::Timing::Print(std::cout);
    esdf_map.getOccupancyMarker(occ_marker, 0, Eigen::Vector4d(1.0, 0, 0, 0.8));
    occ_pub.publish(occ_marker);
//    // visualize distance field
//    esdf_map.getESDFMarker(dis_markers, 1, Eigen::Vector3d(0.0, 1.0, 0));
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


void raycastProcess(int i, int part, int tt, const Eigen::Matrix3d &R) {
    Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
    for (int idx = part * i; idx < part * (i + 1); idx++) {
        std::vector<Eigen::Vector3d> output;
        if (idx > cloud.points.size()) break;
        pcl::PointXYZRGB pt = cloud.points[idx];
        int cnt = 0;
        if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z))
            continue;
        Eigen::Vector3d point = R * Eigen::Vector3d(pt.x, pt.y, pt.z) + s_pos;
        int tmp_idx;
        tmp_idx = esdf_map->setOccupancy((Eigen::Vector3d) point);
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

        Raycast(s_pos / resolution - half, point / resolution - half, lCornor, rCornor, &output);

        for (int i = output.size() - 1; i >= 0; i--) {
            Eigen::Vector3d tmp = (output[i] + half) * resolution;
            int tmp_idx;
            tmp_idx = esdf_map->setOccupancy(tmp, 0);
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

void pointcloudCallBack(const sensor_msgs::PointCloud2::ConstPtr &pointcloud_map) {
    cout << "POINTCLOUD: " << pointcloud_map->header.stamp << endl;
    ros::Time t1, t2, t3;

    int tt = ++tot;
    bool newPos = false;
    cout << transformQueue.size() << " -> ";
    while (!transformQueue.empty() && std::get<0>(transformQueue.front()) <= pointcloud_map->header.stamp) {
        s_pos = std::get<1>(transformQueue.front());
        s_q = std::get<2>(transformQueue.front());
        transformQueue.pop();
        newPos = true;
    }
    cout << transformQueue.size() << "\n";
    if (!newPos) return;
    Eigen::Matrix3d R = s_q.normalized().toRotationMatrix();
//    cout << R << endl;

    pcl::fromROSMsg(*pointcloud_map, cloud);

    if ((int) cloud.points.size() == 0)
        return;

    // TODO: when using vector, this is not needed
#ifdef HASH_TABLE
    uset.clear();
    uset_s.clear();
#endif

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

    int numThread = std::thread::hardware_concurrency();
//    numThread = 1;
    int part = cloud.points.size() / numThread;
//    raycastProcess(0, part, tt, cloud, R);
    std::list<std::thread> integration_threads;

    for (size_t i = 0; i < numThread; ++i) {
        integration_threads.emplace_back(&raycastProcess, i, part, tt, R);
    }

//    cout << integration_threads.size() << endl;
    for (std::thread &thread : integration_threads) {
        thread.join();
    }

//#pragma omp parallel for
//    for (int i = 0; i < numThread; i++)
//        raycastProcess(i, part, tt, R);
//        blankFunc(i, part, tt);
//        for (int idx = part * i; idx < part * (i + 1); idx++) {
//        std::vector<Eigen::Vector3d> output;
//            if (idx > cloud.points.size()) break;
//        pcl::PointXYZRGB pt = cloud.points[idx];
//        int cnt = 0;
//        if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z))
//            continue;
//        Eigen::Vector3d point = R * Eigen::Vector3d(pt.x, pt.y, pt.z) + s_pos;
//        int tmp_idx;
//        tmp_idx = esdf_map->setOccupancy((Eigen::Vector3d) point);
//        if (tmp_idx != -1) {
//            if (uset_s[tmp_idx] == tt)
//                continue;
//            else uset_s[tmp_idx] = tt;
//        }

//        Raycast(s_pos / resolution - half, point / resolution - half, lCornor, rCornor, &output);
//        for (int i = output.size() - 1; i >= 0; i--) {
//            Eigen::Vector3d tmp = (output[i] + half) * resolution;
//            int tmp_idx;
//            tmp_idx = esdf_map->setOccupancy(tmp, 0);
//            if (tmp_idx != -1) {
//                if (uset[tmp_idx] == tt) {
//                    if (++cnt >= 1) break;
//                } else {
//                    uset[tmp_idx] = tt;
//                    cnt = 0;
//                }
//            }
//        }
//            xMin = std::min(xMin, point.x());
//            yMin = std::min(yMin, point.y());
//            zMin = std::min(zMin, point.z());
//            xMax = std::max(xMax, point.x());
//            yMax = std::max(yMax, point.y());
//            zMax = std::max(zMax, point.z());
//        }
//    }

    raycastingTimer.Stop();
    t2 = ros::Time::now();
    timing::Timer updateESDFTimer("updateESDF");
    esdf_map->updateESDF();
    updateESDFTimer.Stop();
    t3 = ros::Time::now();
    cout << (t2 - t1).toSec() << "s\t" << (t3 - t2).toSec() << "s\t" << (t3 - t1).toSec() << 's' << endl;

    if (!esdf_map->check()) {
        cout << "Fatal Error !!" << endl;
        exit(-1);
    }

    ++count;

    if (count % 10 == 0) {
        cout << "Count: " << count << endl;
//        cout << xMin << ' ' << yMin << ' ' << zMin << endl;
//        cout << xMax << ' ' << yMax << ' ' << zMax << endl;
        visulization(*esdf_map);
    }

}

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

int main(int argc, char **argv) {
    ros::init(argc, argv, "Pointcloud_Raycasting_Test");
    ros::NodeHandle node;
//    cout << std::thread::hardware_concurrency() << endl;

    xMin = yMin = zMin = 1e100;
    xMax = yMax = zMax = -1e100;


    lCornor << -11.0, -11.0, -7.0;
    rCornor << 11.0, 11.0, 7.0;
    map_size = rCornor - lCornor;
    resolution = 0.05;
#ifdef HASH_TABLE
    esdf_map = new ESDF_Map(Eigen::Vector3d(0, 0, 0), resolution, 10000000);
#else
    esdf_map = new ESDF_Map(lCornor, resolution, map_size);
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
    ros::Subscriber subpoints = node.subscribe("/camera/depth_registered/points", 1000, pointcloudCallBack);
    ros::Subscriber sub = node.subscribe("/kinect/vrpn_client/estimated_transform", 1000, transformCallback);

    occ_pub = node.advertise<visualization_msgs::Marker>("EDFS_Map/occ", 1, true);
    dist_pub = node.advertise<visualization_msgs::Marker>("EDFS_Map/dist", 1, true);

    ros::spin();
    delete esdf_map;
    return 0;
}