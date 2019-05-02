#include <ros/ros.h>
#include "ESDF_Map.h"
#include <vector>
#include <iostream>
#include <Eigen/Eigen>

using std::cout;
using std::endl;
using std::vector;
ros::Publisher occ_pub;
ros::Publisher dist_pub;


void visulization(ESDF_Map esdf_map) {
    visualization_msgs::Marker occ_marker;
    esdf_map.getOccupancyMarker(occ_marker, 0, Eigen::Vector4d(0, 0, 1.0, 0.8));
    occ_pub.publish(occ_marker);
    // visualize distance field
    vector<visualization_msgs::Marker> dis_markers;
    esdf_map.getESDFMarker(dis_markers, 0, Eigen::Vector3d(1.0, 0, 0));
    for (int i = 0; i < int(dis_markers.size()); ++i) {
        dist_pub.publish(dis_markers[i]);
        ros::Duration(0.1).sleep();
    }

    ros::Duration(1.0).sleep();
}

int main(int argc, char **argv) {

    /*
    ros::init(argc, argv, "EDFS_Map_Test");
    ros::NodeHandle node;

    occ_pub = node.advertise<visualization_msgs::Marker>("EDFS_Map/occ", 1, true);
    dist_pub = node.advertise<visualization_msgs::Marker>("EDFS_Map/dist", 1, true);
    ros::Duration(1.0).sleep();

    ros::Time t1, t2;

    // create a map
    Eigen::Vector3d origin, map_size;
    origin << -5.0, -5.0, 0.0;
    map_size << 10.0, 10.0, 5.0;
    double resolution = 0.2;
    ESDF_Map esdf_map = ESDF_Map(origin, resolution);

    // set occupancy for some positions
    Eigen::Vector3d pos;

    std::vector<std::pair<int, int>> vp;
    for (int x = -4; x <= 4; x += 2)
        for (int y = -4; y <= 4; y += 2)
            vp.push_back(std::make_pair(x, y));

    // insert
    double tt = 0;
    std::random_shuffle(vp.begin(), vp.end());
    for (auto iter = vp.begin(); iter != vp.end(); iter++) {
        // a cuboid
        for (double z = 0.0; z < 5.0; z += 0.1) {
            pos << iter->first, iter->second, z;
            esdf_map.setOccupancy(pos, 1);
        } // z


        t1 = ros::Time::now();
        esdf_map.updateESDF();
        t2 = ros::Time::now();
        tt += (t2 - t1).toSec();
        if (!esdf_map.check()) {
            cout << "Fail!!!" << endl;
            exit(-1);
        }
        if (std::distance(vp.begin(), iter) % 25 == 24)
            visulization(esdf_map);
    } //iter

    cout << "Total Time Used is " << tt << "s." << endl;
//    visulization(esdf_map);

    // delete
    tt = 0;
    std::random_shuffle(vp.begin(), vp.end());
    for (auto iter = vp.begin(); iter != vp.end(); iter++) {
        // a cuboid
        for (double z = 0.0; z < 5.0; z += 0.1) {
            pos << iter->first, iter->second, z;
            esdf_map.setOccupancy(pos, 0);
        } // z


        t1 = ros::Time::now();
        esdf_map.updateESDF();
        t2 = ros::Time::now();
        tt += (t2 - t1).toSec();
        if (!esdf_map.check()) {
            cout << "Fail!!!" << endl;
            exit(-1);
        }
        if (std::distance(vp.begin(), iter) % 5 == 4)
            visulization(esdf_map);
    } //iter
    cout << "Total Time Used is " << tt << "s." << endl;


    return 0;
     */
}