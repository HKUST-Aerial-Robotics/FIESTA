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
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <algorithm>
#include <unordered_set>
#include <omp.h>
#include <thread>
#include <tuple>
#include <queue>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

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
ros::Publisher occ_pointcloud_pub, text_pub, recpc_pub;
Eigen::Vector3d pos, s_pos, origin, c_pos;
Eigen::Quaterniond q, s_q;
int count = 0;
double resolution;
Eigen::Vector3d lCornor, rCornor, map_size;
ESDF_Map *esdf_map;
Eigen::Matrix4d T_B_C, T_D_B, T;
double min_ray_length_m;
double max_ray_length_m;
int visulize_every_n_updates, param_num_thread;
double slice_vis, max_dist, vis_lower_bound, vis_upper_bound, center_x, center_y, focal_x, focal_y;
double pHit, pMiss, pMin, pMax, pOcc;
double depth_filter_maxdist, depth_filter_mindist, depth_filter_tolerance;
int depth_filter_margin;
bool use_depth_filter;
#ifdef SIGNED_NEEDED
ESDF_Map *inv_esdf_map;
#endif
std::queue<std::tuple<ros::Time, Eigen::Vector3d, Eigen::Quaterniond>> transformQueue;
std::queue<sensor_msgs::PointCloud2::ConstPtr> imageQueue;
std::queue<sensor_msgs::Image::ConstPtr> depthMapQueue;

visualization_msgs::Marker occ_marker, slice_marker;

vector<visualization_msgs::Marker> dis_markers;
// #ifdef PROBABILISTIC
// pcl::PointCloud<pcl::PointXYZRGB> cloud;
// #else
pcl::PointCloud<pcl::PointXYZ> cloud;
// #endif
sensor_msgs::PointCloud pc;
#ifndef PROBABILISTIC
sensor_msgs::PointCloud2::ConstPtr s_pc;
#endif

// *************** params ****************
double radius_x, radius_y, radius_z;
Eigen::Vector3d radius;
bool globalVis, globalUpdate, globalMap;
// *************** params ****************

bool newMsg = false;

void visualization(ESDF_Map *esdf_map, bool globalVis, std::string text) {
//    timing::Timing::Print(std::cout);
//    esdf_map->getOccupancyMarker(occ_marker, 0, Eigen::Vector4d(1.0, 0, 0, 0.8));
//    occ_pub.publish(occ_marker);

    if (esdf_map != nullptr) {

        cout << "VISUALIZATION" << endl;
        if (globalVis)
            esdf_map->setOriginalRange();
        else
            esdf_map->setUpdateRange(c_pos - radius, c_pos + radius);
        esdf_map->getPointCloud(pc, (int) (vis_lower_bound / resolution), (int) (vis_upper_bound / resolution));
        occ_pointcloud_pub.publish(pc);
//    Eigen::Vector3i s_vox;
//    esdf_map->pos2vox(c_pos, s_vox);

        esdf_map->getSliceMarker(slice_marker, (int) (slice_vis / resolution), 100,
                                 Eigen::Vector4d(0, 1.0, 0, 1), max_dist);
        slice_pub.publish(slice_marker);
    }

//text viewer
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.id = 3456;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::MODIFY;

    marker.pose.position.x = 8.0;
    marker.pose.position.y = 2.0;
    marker.pose.position.z = 3.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.text = text;

    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.6;

    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;
    text_pub.publish(marker);

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
//int tot1 = 0, tot2 = 0;
#ifdef PROBABILISTIC

void raycastProcess(int i, int part, int tt) {
    int tot1 = 0, tot2 = 0;
    ros::Time t1 = ros::Time::now();
    Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
    for (int idx = part * i; idx < part * (i + 1); idx++) {
        std::vector<Eigen::Vector3d> output;
        if (idx > cloud.points.size()) break;
        // pcl::PointXYZRGB pt = cloud.points[idx];
        pcl::PointXYZ pt = cloud.points[idx];
        int cnt = 0;
        if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z))
            continue;
        Eigen::Vector4d tmp = T * Eigen::Vector4d(pt.x, pt.y, pt.z, 1);
        Eigen::Vector3d point = Eigen::Vector3d(tmp(0), tmp(1), tmp(2)) / tmp(3);

//        Eigen::Vector3d point = Eigen::Vector3d(pt.x, pt.y, pt.z);
        int tmp_idx;
        double length = (point - origin).norm();
        if (length < min_ray_length_m)
            continue;
        else if (length > max_ray_length_m) {
            point = (point - origin) / length * max_ray_length_m + origin;
//        cout << point << endl;
            tmp_idx = esdf_map->setOccupancy((Eigen::Vector3d) point, 0);
        } else
            tmp_idx = esdf_map->setOccupancy((Eigen::Vector3d) point, 1);
#ifdef SIGNED_NEEDED
        tmp_idx = inv_esdf_map->setOccupancy((Eigen::Vector3d) point, 0);
#endif
//         //TODO: -10000 ?

        if (tmp_idx != -10000) {
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
        tot1++;
        Raycast(origin / resolution - half, point / resolution - half, lCornor, rCornor, &output);

        for (int i = output.size() - 2; i >= 0; i--) {
            tot2++;
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
            //TODO: -10000 ?
            if (tmp_idx != -10000) {
#ifdef HASH_TABLE
                if (uset.find(tmp_idx) != uset.end()) {
                    if (++cnt >= 1) {cnt= 0;break;}
                } else {
                    uset.insert(tmp_idx);
                    cnt = 0;
                }
#else
//                if (!(tmp_idx >=0 && tmp_idx < esdf_map->grid_total_size)) {
//                    cout<<"error!\t" <<tmp_idx<<endl;
//                }
                if (uset[tmp_idx] == tt) {
                    if (++cnt >= 1) {
                        cnt = 0;
                        break;
                    }
                } else {
                    uset[tmp_idx] = tt;
                    cnt = 0;
                }
#endif
            }
        }
    }
    ros::Time t2 = ros::Time::now();
//    cout << "raycasting \t" << tot1 << "\t" << tot2 << "\t" << (t2 - t1).toSec() << endl;

}

#endif

int receivedCount = 0, dmCount = 0, pcCount = 0, posCount = 0;

void pointcloudCallBack(const sensor_msgs::PointCloud2::ConstPtr &pointcloud_map) {
//    cout << ++pcCount << '\t' << "POINTCLOUD: " << pointcloud_map->header.stamp << endl;
/*
    T.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
    T.block<3, 1>(0, 3) = pos;
    T(3, 0) = T(3, 1) = T(3, 2) = 0;
    T(3, 3) = 1;
    T = T * T_D_B * T_B_C;
    pcl::fromROSMsg(*pointcloud_map, cloud);
    sensor_msgs::PointCloud m;
    m.header.frame_id = "world";
    m.points.clear();
    for (int i = 0; i < cloud.points.size(); i++) {
        Eigen::Vector4d tmp = T * Eigen::Vector4d(cloud[i].x, cloud[i].y, cloud[i].z, 1);
        geometry_msgs::Point32 p;
        p.x = tmp(0) / tmp(3);
        p.y = tmp(1) / tmp(3);
        p.z = tmp(2) / tmp(3);
        m.points.push_back(p);
    }
    recpc_pub.publish(m);
        */
    int tt = ++tot;
//    cout << "PC: " << tt << endl;
//    cout << "size : " << sizeof(*pointcloud_map) << endl;
    imageQueue.push(pointcloud_map);
//    cout << "Queue Size\t" << imageQueue.size() << ' ' << transformQueue.size() << endl;
    //if (cnt == 1500) std::thread(save).join();
    // std::cout << "Image timestamp\t" << msg->header.stamp << "\t" << std::endl;

    ros::Time _poseTime, _imageTime;
    double timeDelay = 3e-3;
    while (!imageQueue.empty()) {
        bool newPos = false;
        _imageTime = imageQueue.front()->header.stamp;
        while (transformQueue.size() > 1 &&
               std::get<0>(transformQueue.front()) <= _imageTime + ros::Duration(timeDelay)) {
            _poseTime = std::get<0>(transformQueue.front());
            s_pos = std::get<1>(transformQueue.front());
            s_q = std::get<2>(transformQueue.front());
            transformQueue.pop();
            newPos = true;
        }
        if (transformQueue.empty() || std::get<0>(transformQueue.front()) <= _imageTime + ros::Duration(timeDelay)) {
            break;
        }
        if (!newPos) {
            imageQueue.pop();
            continue;
        }

//        cout << ++receivedCount << '\t' << fabs((_poseTime - _imageTime).toSec()) << endl;
        newMsg = true;
#ifndef PROBABILISTIC
        s_pc = imageQueue.front();
        return;
#else
        T.block<3, 3>(0, 0) = s_q.toRotationMatrix();
        T.block<3, 1>(0, 3) = s_pos;
        T(3, 0) = T(3, 1) = T(3, 2) = 0;
        T(3, 3) = 1;
        T = T * T_D_B * T_B_C;
        origin = Eigen::Vector3d(T(0, 3), T(1, 3), T(2, 3)) / T(3, 3);

        pcl::fromROSMsg(*imageQueue.front(), cloud);

        if ((int) cloud.points.size() == 0)
            return;

        // TODO: when using vector, this is not needed
#ifdef HASH_TABLE
        uset.clear();
        uset_s.clear();
#endif
//        cout << cloud.points.size() << endl;
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

        if (param_num_thread == 0) {
            int numThread = 1;
            int part = cloud.points.size() / numThread;
            // cout << part << endl;
            raycastProcess(0, part, tt);
        } else {
            int numThread = param_num_thread;//std::thread::hardware_concurrency();
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
        }
        // sensor_msgs::PointCloud m;
        // m.header.frame_id = "world";
        // m.points.clear();
        // for (int i = 0; i < cloud.points.size(); i++) {
        //     Eigen::Vector4d tmp = T * Eigen::Vector4d(cloud[i].x, cloud[i].y, cloud[i].z, 1);
        //     geometry_msgs::Point32 p;
        //     p.x = tmp(0) / tmp(3);
        //     p.y = tmp(1) / tmp(3);
        //     p.z = tmp(2) / tmp(3);
        //     m.points.push_back(p);
        // }
        // recpc_pub.publish(m);
        raycastingTimer.Stop();
        imageQueue.pop();
#endif
    }

}

cv::Mat img, lastImg;
Eigen::Matrix4d lastT;
bool firstImg = true;

double getInterpolation(const cv::Mat &img, double u, double v) {

    int vu = img.at<uint16_t>(v, u);
    int v1u = img.at<uint16_t>(v + 1, u);
    int vu1 = img.at<uint16_t>(v, u + 1);
    int v1u1 = img.at<uint16_t>(v + 1, u + 1);

    float a = u - (float) u;
    float c = v - (float) v;

    return (vu * (1.f - a) + vu1 * a) * (1.f - c) + (v1u * (1.f - a) + v1u1 * a) * c;
}

int sum_kkp = 0, cnt_kkp = 0, t1 = 0, t2 = 0;

void depthCallBack(const sensor_msgs::Image::ConstPtr &depth_map) {
//    cout << ++dmCount << '\t' << "DEPTH_MAP: " << depth_map->header.stamp << endl;
    int tt = ++tot;
//    cout << "PC: " << tt << endl;
//    cout << "size : " << sizeof(*depth_map) << endl;
    depthMapQueue.push(depth_map);
//    cout << "Queue Size\t" << depthMapQueue.size() << ' ' << transformQueue.size() << endl;
    //if (cnt == 1500) std::thread(save).join();
    // std::cout << "Image timestamp\t" << msg->header.stamp << "\t" << std::endl;

    ros::Time _poseTime, _imageTime;
    double timeDelay = 3e-3;
    while (!depthMapQueue.empty()) {
        bool newPos = false;
        _imageTime = depthMapQueue.front()->header.stamp;
        while (transformQueue.size() > 1 &&
               std::get<0>(transformQueue.front()) <= _imageTime + ros::Duration(timeDelay)) {
            _poseTime = std::get<0>(transformQueue.front());
            s_pos = std::get<1>(transformQueue.front());
            s_q = std::get<2>(transformQueue.front());
            transformQueue.pop();
            newPos = true;
        }
        if (transformQueue.empty() || std::get<0>(transformQueue.front()) <= _imageTime + ros::Duration(timeDelay)) {
            break;
        }
        if (!newPos) {
            depthMapQueue.pop();
            continue;
        }

//        cout << ++receivedCount << '\t' << fabs((_poseTime - _imageTime).toSec()) << endl;
        newMsg = true;
#ifndef PROBABILISTIC
        // TODO: Modify this, this will cause Compile Error
        s_pc = depthMapQueue.front();
        return;
#else
        if (use_depth_filter) {
            lastT = T;
            lastImg = img.clone();
        }
        T.block<3, 3>(0, 0) = s_q.toRotationMatrix();
        T.block<3, 1>(0, 3) = s_pos;
        T(3, 0) = T(3, 1) = T(3, 2) = 0;
        T(3, 3) = 1;
        T = T * T_D_B * T_B_C;
        origin = Eigen::Vector3d(T(0, 3), T(1, 3), T(2, 3)) / T(3, 3);


        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(depthMapQueue.front(), depthMapQueue.front()->encoding);
        constexpr double kDepthScalingFactor = 1000.0;
        if (depthMapQueue.front()->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
            (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, kDepthScalingFactor);
        }
        cv_ptr->image.copyTo(img);

        double depth;
        cloud.clear();
        double uu, vv;

        t1 = t2 = 0;
        if (!use_depth_filter) {
            for (int v = 0; v < img.rows; v++)
                for (int u = 0; u < img.cols; u++) {
                    depth = img.at<uint16_t>(v, u) / kDepthScalingFactor;
                    pcl::PointXYZ point;
                    point.x = (u - center_x) * depth / focal_x;
                    point.y = (v - center_y) * depth / focal_y;
                    point.z = depth;
                    cloud.push_back(point);
                }
        } else {
            Eigen::Vector4d tmpp;
            Eigen::Vector3d tmp;
            for (int v = depth_filter_margin; v < img.rows - depth_filter_margin; v++)
                for (int u = depth_filter_margin; u < img.cols - depth_filter_margin; u++) {
                    depth = img.at<uint16_t>(v, u) / kDepthScalingFactor;
                    pcl::PointXYZ point;
                    point.x = (u - center_x) * depth / focal_x;
                    point.y = (v - center_y) * depth / focal_y;
                    point.z = depth;
                    if (!firstImg) {
//                        if (depth > depth_filter_maxdist || depth < depth_filter_mindist) continue;
                        tmpp = lastT.inverse() * T * Eigen::Vector4d(point.x, point.y, point.z, 1);
                        tmp = Eigen::Vector3d(tmpp(0), tmpp(1), tmpp(2)) / tmpp(3);
                        uu = tmp.x() * focal_x / tmp.z() + center_x;
                        vv = tmp.y() * focal_y / tmp.z() + center_y;
                        if (uu >= 0 && uu < img.cols - 1 && vv >= 0 && vv < img.rows - 1) {
                            if (fabs(getInterpolation(lastImg, uu, vv) / kDepthScalingFactor - tmp.z())
                                < depth_filter_tolerance) {
                                cloud.push_back(point);
//                                ++t1;
                            }
//                            ++t2;
                        } //else cloud.push_back(point);
                    } //else cloud.push_back(point);
                }
            firstImg = false;
        }
//        cout << t1 << "\t" << t2 << "\t" << (double) t1 / t2 << '\t' << (double) t2 / (img.rows * img.cols) << endl;
//        cout << "pointcloud_size:\t" << cloud.points.size() << "\t"
//             << (sum_kkp += cloud.points.size()) / (double) (++cnt_kkp) << endl;

        cout << "pointcloud_size:\t" << cloud.points.size() << endl;
        if ((int) cloud.points.size() == 0)
            return;
//        sensor_msgs::PointCloud m;
//        m.header.frame_id = "world";
//        m.points.clear();
//        for (int i = 0; i < cloud.points.size(); i++) {
//            Eigen::Vector4d tmp = T * Eigen::Vector4d(cloud[i].x, cloud[i].y, cloud[i].z, 1);
//            geometry_msgs::Point32 p;
//            p.x = tmp(0) / tmp(3);
//            p.y = tmp(1) / tmp(3);
//            p.z = tmp(2) / tmp(3);
//            m.points.push_back(p);
//        }
//        recpc_pub.publish(m);
        // TODO: when using vector, this is not needed
#ifdef HASH_TABLE
        uset.clear();
        uset_s.clear();
#endif
//        cout << cloud.points.size() << endl;
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

        if (param_num_thread == 0) {
            int numThread = 1;
            int part = cloud.points.size() / numThread;
            // cout << part << endl;
            raycastProcess(0, part, tt);
        } else {
            int numThread = param_num_thread;//std::thread::hardware_concurrency();
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
        }
        // sensor_msgs::PointCloud m;
        // m.header.frame_id = "world";
        // m.points.clear();
        // for (int i = 0; i < cloud.points.size(); i++) {
        //     Eigen::Vector4d tmp = T * Eigen::Vector4d(cloud[i].x, cloud[i].y, cloud[i].z, 1);
        //     geometry_msgs::Point32 p;
        //     p.x = tmp(0) / tmp(3);
        //     p.y = tmp(1) / tmp(3);
        //     p.z = tmp(2) / tmp(3);
        //     m.points.push_back(p);
        // }
        // recpc_pub.publish(m);
        raycastingTimer.Stop();
        depthMapQueue.pop();
#endif
    }

}

ros::Publisher meshPub;
visualization_msgs::Marker meshROS;
static std::string mesh_resource;

void publishPose(Eigen::Vector3d pos, Eigen::Quaterniond q) {
    // Mesh model
    meshROS.header.frame_id = "world";
    meshROS.ns = "mesh";
    meshROS.id = 0;
    meshROS.type = visualization_msgs::Marker::ARROW;
    meshROS.action = visualization_msgs::Marker::ADD;
    meshROS.pose.position.x = pos.x();
    meshROS.pose.position.y = pos.y();
    meshROS.pose.position.z = pos.z();
    meshROS.pose.orientation.w = q.w();
    meshROS.pose.orientation.x = q.x();
    meshROS.pose.orientation.y = q.y();
    meshROS.pose.orientation.z = q.z();
    meshROS.scale.x = 0.4;
    meshROS.scale.y = 0.1;
    meshROS.scale.z = 0.1;
    meshROS.color.a = 1;
    meshROS.color.r = 1;
    meshROS.color.g = 1;
    meshROS.color.b = 1;
    // meshROS.mesh_resource = mesh_resource;
    meshPub.publish(meshROS);
}

// #ifdef PROBABILISTIC

// void transformCallback(const geometry_msgs::TransformStamped::ConstPtr &msg) {
// //    cout << "TRANSFORM: " << msg->header.stamp << endl;
//     pos = Eigen::Vector3d(msg->transform.translation.x,
//                           msg->transform.translation.y,
//                           msg->transform.translation.z);
//     q = Eigen::Quaterniond(msg->transform.rotation.w,
//                            msg->transform.rotation.x,
//                            msg->transform.rotation.y,
//                            msg->transform.rotation.z);
//     transformQueue.push(std::make_tuple(msg->header.stamp, pos, q));
// }

// #else
// void transformCallback(const nav_msgs::Odometry::ConstPtr &msg) {
//     cout << "TRANSFORM: " << msg->header.stamp << endl;
//     pos = Eigen::Vector3d(msg->pose.pose.position.x,
//                           msg->pose.pose.position.y,
//                           msg->pose.pose.position.z);
//     q = Eigen::Quaterniond(msg->pose.pose.orientation.w,
//                            msg->pose.pose.orientation.x,
//                            msg->pose.pose.orientation.y,
//                            msg->pose.pose.orientation.z);
//     transformQueue.push(std::make_tuple(msg->header.stamp, pos, q));

// }
void transformCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
//    cout << ++posCount << '\t' << "TRANSFORM: " << msg->header.stamp << endl;
    pos = Eigen::Vector3d(msg->pose.position.x,
                          msg->pose.position.y,
                          msg->pose.position.z);
    q = Eigen::Quaterniond(msg->pose.orientation.w,
                           msg->pose.orientation.x,
                           msg->pose.orientation.y,
                           msg->pose.orientation.z);

    transformQueue.push(std::make_tuple(msg->header.stamp, pos, q));
    publishPose(pos, q);

}

// #endif
int esdf_cnt = 0;

void updateESDFEvent(const ros::TimerEvent & /*event*/) {
    if (!newMsg) return;
    newMsg = false;
    c_pos = s_pos;

#ifndef PROBABILISTIC
    timing::Timer handlePCTimer("handlePointCloud");
    pcl::fromROSMsg(*s_pc, cloud);

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
    cout << "Running " << esdf_cnt << " updates." << endl;
    ros::Time t1 = ros::Time::now();
    if (esdf_map->checkUpdate()) {

        timing::Timer updateESDFTimer("updateESDF");
        if (globalUpdate)
            esdf_map->setOriginalRange();
        else
            esdf_map->setUpdateRange(c_pos - radius, c_pos + radius, true);
        esdf_map->updateOccupancy();
        esdf_map->updateESDF();
#ifdef SIGNED_NEEDED
        inv_esdf_map->updateOccupancy();
        inv_esdf_map->updateESDF();
#endif
        updateESDFTimer.Stop();
        timing::Timing::Print(std::cout);
//        if (!esdf_map->checkWithGT()){
//            exit(0);
//        }
    }
    ros::Time t2 = ros::Time::now();

    std::string text = "FIESTA\nCurrent update Time\n"
                       + timing::Timing::SecondsToTimeString((t2 - t1).toSec() * 1000)
                       + " ms\n" + "Average update Time\n" +
                       timing::Timing::SecondsToTimeString(timing::Timing::GetMeanSeconds("updateESDF") * 1000)
                       + " ms";

    if (visulize_every_n_updates != 0 && esdf_cnt % visulize_every_n_updates == 0) {
//        std::thread(visualization, esdf_map, text).detach();
        visualization(esdf_map, globalVis, text);
    } else {
//        std::thread(visualization, nullptr, text).detach();
        visualization(nullptr, globalVis, text);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ESDF_fromPointcloud");
    ros::NodeHandle node("~");

    node.param<double>("resolution", resolution, 0.1);
    node.param<int>("visulize_every_n_updates", visulize_every_n_updates, 1);
    node.param<double>("min_ray_length_m", min_ray_length_m, 0.1);
    node.param<double>("max_ray_length_m", max_ray_length_m, 5.0);
    node.param<double>("slice_vis", slice_vis, 0);
    node.param<double>("vis_lower_bound", vis_lower_bound, 0);
    node.param<double>("vis_upper_bound", vis_upper_bound, 2);
    node.param<double>("cx", center_x, 0);
    node.param<double>("cy", center_y, 0);
    node.param<double>("fx", focal_x, 1);
    node.param<double>("fy", focal_y, 1);

    node.param<double>("max_dist", max_dist, 2.0);
    node.param<int>("num_thread", param_num_thread, 0);
    // TODO:  modify this
    node.param("mesh_resource", mesh_resource, std::string("meshes/hummingbird.mesh"));


    if (!globalMap) {
        globalVis = globalUpdate = false;
    }


#ifdef HASH_TABLE
    int reservedSize;
    lCornor << -100.f, -100.f, -100.f;
    rCornor << 100.f, 100.f, 100.f;
    node.param<int>("reservedSize", reservedSize, 1000000);
    esdf_map = new ESDF_Map(Eigen::Vector3d(0, 0, 0), resolution, reservedSize);
#ifdef SIGNED_NEEDED
    inv_esdf_map = new ESDF_Map(Eigen::Vector3d(0, 0, 0), resolution, reservedSize);
#endif

#else
    double lx, ly, lz;
    double rx, ry, rz;
    node.param<double>("lx", lx, -20.f);
    node.param<double>("ly", ly, -20.f);
    node.param<double>("lz", lz, 0.f);
    node.param<double>("rx", rx, 20.f);
    node.param<double>("ry", ry, 20.f);
    node.param<double>("rz", rz, 5.f);
    node.param<double>("radius_x", radius_x, 3.f);
    node.param<double>("radius_y", radius_y, 3.f);
    node.param<double>("radius_z", radius_z, 1.5f);
    radius = Eigen::Vector3d(radius_x, radius_y, radius_z);

    node.param<bool>("globalMap", globalMap, false);
    node.param<bool>("globalUpdate", globalUpdate, false);
    node.param<bool>("globalVis", globalVis, false);

    node.param<bool>("use_depth_filter", use_depth_filter, true);
    node.param<double>("depth_filter_tolerance", depth_filter_tolerance, 0.1f);
    node.param<double>("depth_filter_maxdist", depth_filter_maxdist, 10.f);
    node.param<double>("depth_filter_mindist", depth_filter_mindist, 0.1f);
    node.param<int>("depth_filter_margin", depth_filter_margin, 0);

    lCornor << lx, ly, lz;
    rCornor << rx, ry, rz;
    map_size = rCornor - lCornor;
    esdf_map = new ESDF_Map(lCornor, resolution, map_size);
#ifdef SIGNED_NEEDED
    inv_esdf_map = new ESDF_Map(lCornor, resolution, map_size);
#endif
#endif

#ifdef PROBABILISTIC
    node.param<double>("pHit", pHit, 0.70);
    node.param<double>("pMiss", pMiss, 0.35);
    node.param<double>("pMin", pMin, 0.12);
    node.param<double>("pMax", pMax, 0.97);
    node.param<double>("pOcc", pOcc, 0.80);
    esdf_map->setParameter(pHit, pMiss, pMin, pMax, pOcc);
#endif

    // These 2 lines are to be used in fast raycasting
    lCornor /= resolution;
    rCornor /= resolution;

#ifndef HASH_TABLE
    uset.resize(esdf_map->grid_total_size);
    uset_s.resize(esdf_map->grid_total_size);
    std::fill(uset.begin(), uset.end(), 0);
    std::fill(uset_s.begin(), uset_s.end(), 0);
#endif


    // For Jie Bao
    ros::Subscriber subpoints = node.subscribe("pointcloud", 10, pointcloudCallBack,
                                               ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub = node.subscribe("transform", 10, transformCallback);
    ros::Subscriber subdepth = node.subscribe("depth", 10, depthCallBack);

    T_B_C << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    T_D_B << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    // Cow_and_Lady
    // ros::Subscriber subpoints = node.subscribe("/camera/depth_registered/points", 1000, pointcloudCallBack);
    // ros::Subscriber sub = node.subscribe("/kinect/vrpn_client/estimated_transform", 1000, transformCallback);

    // T_B_C << 1, 0, 0, 0,
    //         0, 1, 0, 0,
    //         0, 0, 1, 0,
    //         0, 0, 0, 1;
    // T_D_B << 0.971048, -0.120915, 0.206023, 0.00114049,
    //         0.15701, 0.973037, -0.168959, 0.0450936,
    //         -0.180038, 0.196415, 0.96385, 0.0430765,
    //         0.0, 0.0, 0.0, 1.0;
//
    //EuRoC
//    ros::Subscriber subpoints = node.subscribe("/dense_stereo/pointcloud", 1000, pointcloudCallBack);
//    ros::Subscriber sub = node.subscribe("/vicon/firefly_sbx/firefly_sbx", 1000, transformCallback);
//    T_B_C << 0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
//            0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
//            -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
//            0.0, 0.0, 0.0, 1.0;
//    T_D_B << 0.33638, -0.01749, 0.94156, 0.06901,
//            -0.02078, -0.99972, -0.01114, -0.02781,
//            0.94150, -0.01582, -0.33665, -0.12395,
//            0.0, 0.0, 0.0, 1.0;
//    T_D_B = T_D_B.inverse();

    // lidar
//        ros::Subscriber subpoints = node.subscribe("/laser_cloud_surround", 1000, pointcloudCallBack);
//    ros::Subscriber sub = node.subscribe("/odom_world", 1000, transformCallback);
//    T_B_C << 1, 0, 0, 0,
//            0, 1, 0, 0,
//            0, 0, 1, 0,
//            0, 0, 0, 1;
//    T_D_B << 1, 0, 0, 0,
//            0, 1, 0, 0,
//            0, 0, 1, 0,
//            0, 0, 0, 1;
    occ_pub = node.advertise<visualization_msgs::Marker>("ESDF_Map/occ", 1, true);
    dist_pub = node.advertise<visualization_msgs::Marker>("ESDF_Map/dist", 1, true);
    slice_pub = node.advertise<visualization_msgs::Marker>("ESDF_Map/slice", 1, true);
    occ_pointcloud_pub = node.advertise<sensor_msgs::PointCloud>("ESDF_Map/occ_pc", 1, true);
    text_pub = node.advertise<visualization_msgs::Marker>("ESDF_Map/text", 1, true);
    meshPub = node.advertise<visualization_msgs::Marker>("ESDF_Map/robot", 100, true);
    recpc_pub = node.advertise<sensor_msgs::PointCloud>("ESDF_Map/rectified_pointcloud", 1, true);

    double update_esdf_every_n_sec;
    node.param<double>("update_esdf_every_n_sec", update_esdf_every_n_sec, 0.1);
    ros::Timer update_mesh_timer_ =
            node.createTimer(ros::Duration(update_esdf_every_n_sec),
                             &updateESDFEvent);
//    double update_mesh_every_n_sec;
//    node.param<double>( "update_mesh_every_n_sec", update_mesh_every_n_sec, 0.5);
//    ros::Timer update_mesh_timer_ =
//            node.createTimer(ros::Duration(120.f),
//                             &);
//    ros::MultiThreadedSpinner spinner(10);
//    spinner.spin();
    ros::spin();
    delete esdf_map;
#ifdef SIGNED_NEEDED
    delete inv_esdf_map;
#endif

    return 0;
}
