//
// Created by tommy on 12/17/18.
//

#include <mutex>
#include "ESDF_Map.h"
#include <math.h>
#include <time.h>


using std::cout;
using std::endl;

bool ESDF_Map::exist(int idx) {
#ifdef PROBABILISTIC
    return occupancyBuffer[idx] > logit(min_occupancy_log_);
#else
    return occupancyBuffer[idx] == 1;
#endif
}

bool ESDF_Map::posInMap(Eigen::Vector3d pos) {

#ifdef HASH_TABLE
    return true;
#else
    if (pos(0) < min_range(0) || pos(1) < min_range(1) || pos(2) < min_range(2)) {
//        cout << pos(0) << ' ' << pos(1) << ' ' << pos(2) << endl;
//        cout << "less than min range!" << endl;
        return false;
    }

    if (pos(0) > max_range(0) || pos(1) > max_range(1) || pos(2) > max_range(2)) {
//        cout << pos(0) << ' ' << pos(1) << ' ' << pos(2) << endl;
//        cout << "larger than max range!" << endl;
        return false;
    }
    return true;
#endif
}

bool ESDF_Map::voxInRange(Eigen::Vector3i vox, bool currectVec) {
    if (currectVec)
        return (vox(0) >= min_vec(0) && vox(0) <= max_vec(0)
             && vox(1) >= min_vec(1) && vox(1) <= max_vec(1)
             && vox(2) >= min_vec(2) && vox(2) <= max_vec(2));
    else return (vox(0) >= last_min_vec(0) && vox(0) <= last_max_vec(0)
              && vox(1) >= last_min_vec(1) && vox(1) <= last_max_vec(1)
              && vox(2) >= last_min_vec(2) && vox(2) <= last_max_vec(2));
}

void ESDF_Map::pos2vox(Eigen::Vector3d pos, Eigen::Vector3i &vox) {
    for (int i = 0; i < 3; ++i)
        vox(i) = floor((pos(i) - origin(i)) / resolution);
}

void ESDF_Map::vox2pos(Eigen::Vector3i vox, Eigen::Vector3d &pos) {
    for (int i = 0; i < 3; ++i)
        pos(i) = (vox(i) + 0.5) * resolution + origin(i);
}

int ESDF_Map::vox2idx(Eigen::Vector3i vox) {
#ifdef HASH_TABLE
    if (vox(0) == undefined) return special4undefined;
    return findAndInsert(Eigen::Vector3i(vox(0), vox(1), vox(2)));
#else
    if (vox(0) == undefined) return special4undefined;
    return vox(0) * grid_size_yz + vox(1) * grid_size(2) + vox(2);
#endif
}

int ESDF_Map::vox2idx(Eigen::Vector3i vox, int subsampling_factor) {
#ifdef HASH_TABLE
    // TODO: ignore subsampling_factor
    if (vox(0) == undefined) return special4undefined;
    return findAndInsert(Eigen::Vector3i(vox(0), vox(1), vox(2)));
#else
    if (vox(0) == undefined) return special4undefined;
    return vox(0) * grid_size_yz / subsampling_factor / subsampling_factor / subsampling_factor
           + vox(1) * grid_size(2) / subsampling_factor / subsampling_factor
           + vox(2) / subsampling_factor;
#endif
}

Eigen::Vector3i ESDF_Map::idx2vox(int idx) {
#ifdef HASH_TABLE
    return voxBuffer[idx];
#else
    return Eigen::Vector3i(idx / grid_size_yz,
                           idx % (grid_size_yz) / grid_size(2),
                           idx % grid_size(2));
#endif

}

double ESDF_Map::dist(Eigen::Vector3i a, Eigen::Vector3i b) {
    return (b - a).cast<double>().norm() * resolution;
//        return (b - a).squaredNorm();
// TODO: may use square root & * resolution at last together to speed up
}


#ifdef HASH_TABLE

ESDF_Map::ESDF_Map(Eigen::Vector3d origin, double resolution, int reserveSize_)
        : origin(origin), resolution(resolution) {
    inf = 10000;
    undefined = -10000;
    special4undefined = 0;
#ifdef BLOCK
    blockBit = 3;
    block = (1 << blockBit);

    blockSize = block * block * block;
    if (blockSize > reserveSize_) reserveSize_ = blockSize;
#endif
    setOriginalRange();
    count = 1; // 0 is used for special use
    reserveSize = reserveSize_ + 1;
    occupancyBuffer.resize(reserveSize);
    distanceBuffer.resize(reserveSize);
//    distanceBufferNegative.resize(reserveSize);
    closestObstacle.resize(reserveSize);
    voxBuffer.resize(reserveSize);
    tmp1.resize(reserveSize);
    tmp2.resize(reserveSize);

    std::fill(distanceBuffer.begin(), distanceBuffer.end(), (double) undefined);
//    std::fill(distanceBufferNegative.begin(), distanceBufferNegative.end(), (double) undefined);
    std::fill(occupancyBuffer.begin(), occupancyBuffer.end(), 0);
    std::fill(closestObstacle.begin(), closestObstacle.end(), Eigen::Vector3i(undefined, undefined, undefined));
    std::fill(voxBuffer.begin(), voxBuffer.end(), Eigen::Vector3i(undefined, undefined, undefined));
    std::fill(tmp1.begin(), tmp1.end(), 0);
    std::fill(tmp2.begin(), tmp2.end(), 0);

    head.resize(reserveSize);
    prev.resize(reserveSize);
    next.resize(reserveSize);
    std::fill(head.begin(), head.end(), undefined);
    std::fill(prev.begin(), prev.end(), undefined);
    std::fill(next.begin(), next.end(), undefined);

}

#else

ESDF_Map::ESDF_Map(Eigen::Vector3d origin, double resolution, Eigen::Vector3d map_size)
        : origin(origin), resolution(resolution), map_size(map_size) {


    for (int i = 0; i < 3; ++i) grid_size(i) = ceil(map_size(i) / resolution);

    min_range = origin;
    max_range = origin + map_size;
    grid_size_yz = grid_size(1) * grid_size(2);
    inf = 10000;
    undefined = -10000;

    grid_total_size = grid_size(0) * grid_size_yz;
    special4undefined = grid_total_size;
    setOriginalRange();


    cout << grid_total_size << endl;
    occupancyBuffer.resize(grid_total_size);

    distanceBuffer.resize(grid_total_size);
//    distanceBufferNegative.resize(grid_total_size);

    closestObstacle.resize(grid_total_size);
    tmp1.resize(grid_total_size);
    tmp2.resize(grid_total_size);

    std::fill(distanceBuffer.begin(), distanceBuffer.end(), (double) undefined);
//    std::fill(distanceBufferNegative.begin(), distanceBufferNegative.end(), (double) undefined);

    std::fill(occupancyBuffer.begin(), occupancyBuffer.end(), 0);
    std::fill(closestObstacle.begin(), closestObstacle.end(), Eigen::Vector3i(undefined, undefined, undefined));
    std::fill(tmp1.begin(), tmp1.end(), 0);
    std::fill(tmp2.begin(), tmp2.end(), 0);

    head.resize(grid_total_size + 1);
    prev.resize(grid_total_size);
    next.resize(grid_total_size);
    std::fill(head.begin(), head.end(), undefined);
    std::fill(prev.begin(), prev.end(), undefined);
    std::fill(next.begin(), next.end(), undefined);

}

#endif

void ESDF_Map::deleteFromList(int link, int idx) {
    if (prev[idx] != undefined)
        next[prev[idx]] = next[idx];
    else
        head[link] = next[idx];
    if (next[idx] != undefined)
        prev[next[idx]] = prev[idx];
    prev[idx] = next[idx] = undefined;
}

void ESDF_Map::insertIntoList(int link, int idx) {
    if (head[link] == undefined)
        head[link] = idx;
    else {
        prev[head[link]] = idx;
        next[idx] = head[link];
        head[link] = idx;
    }
}

void ESDF_Map::updateESDF() {
//    clock_t startTime,endTime;
//    startTime = clock();
//    updateOccupancy();
    cout << "Insert " << insertQueue.size() << "\tDelete " << deleteQueue.size() << endl;
    while (!insertQueue.empty()) {
        QueueElement xx = insertQueue.front();
        insertQueue.pop();
        int idx = vox2idx(xx.point);
        if (exist(idx)) {
            // exist after a whole brunch of updates
            // delete previous link & create a new linked-list
            deleteFromList(vox2idx(closestObstacle[idx]), idx);
            closestObstacle[idx] = xx.point;
            distanceBuffer[idx] = 0.0;
            insertIntoList(idx, idx);
            updateQueue.push(xx);
        }
    }
    while (!deleteQueue.empty()) {
        QueueElement xx = deleteQueue.front();

        deleteQueue.pop();
        int idx = vox2idx(xx.point);
        if (!exist(idx)) {
            // doesn't exist after a whole brunch of updates

            int next_obs_idx;
            for (int obs_idx = head[idx]; obs_idx != undefined; obs_idx = next_obs_idx) {

                closestObstacle[obs_idx] = Eigen::Vector3i(undefined, undefined, undefined);
                Eigen::Vector3i obs_vox = idx2vox(obs_idx);

                double distance = inf;
                // find neighborhood whose closest obstacles exist
                for (int i = 0; i < dirNum; i++) {
                    Eigen::Vector3i newPos = obs_vox + dir[i];
                    int newPosId = vox2idx(newPos);
                    if (voxInRange(newPos) && closestObstacle[newPosId](0) != undefined
                        && exist(vox2idx(closestObstacle[newPosId]))) {
                        // if in range and closest obstacles exist
                        double tmp = dist(obs_vox, closestObstacle[newPosId]);
                        if (tmp < distance) {
                            distance = tmp;
                            closestObstacle[obs_idx] = closestObstacle[newPosId];
                        }
                        break;
                    } // if
                } // for neighborhood

                // destroy the linked-list
                prev[obs_idx] = undefined;
                next_obs_idx = next[obs_idx];
                next[obs_idx] = undefined;

                distanceBuffer[obs_idx] = distance;
                if (distance < inf) {
                    updateQueue.push(QueueElement{obs_vox, distance});
                }
                int newObsIdx = vox2idx(closestObstacle[obs_idx]);
                insertIntoList(newObsIdx, obs_idx);
            } // for obs_idx
            head[idx] = undefined;
        } // if
    } // deleteQueue
    while (true) {
        int times = 0, changeNum = 0;
        while (!updateQueue.empty()) {
            QueueElement xx = updateQueue.front();
//            QueueElement xx = updateQueue.top();

            updateQueue.pop();
            int idx = vox2idx(xx.point);
            if (xx.distance == distanceBuffer[idx]) {
                times++;
                bool change = false;
                for (int i = 0; i < dirNum; i++) {
                    Eigen::Vector3i newPos = xx.point + dir[i];
                    if (voxInRange(newPos)) {
                        int newPosId = vox2idx(newPos);
                        if (closestObstacle[newPosId](0) == undefined) continue;
                        double tmp = dist(xx.point, closestObstacle[newPosId]);

                        if (distanceBuffer[idx] > tmp) {
                            distanceBuffer[idx] = tmp;
                            change = true;
                            deleteFromList(vox2idx(closestObstacle[idx]), idx);

                            int newObsIdx = vox2idx(closestObstacle[newPosId]);
                            insertIntoList(newObsIdx, idx);
                            closestObstacle[idx] = closestObstacle[newPosId];
                        }
                    }
                }


                if (change) {
                    changeNum++;
                    updateQueue.push(QueueElement{xx.point, distanceBuffer[idx]});
                    continue;
                }
//                double dist_tmp = dist(xx.point, observationPoint);
                for (int i = 0; i < dirNum; i++) {
                    Eigen::Vector3i newPos = xx.point + dir[i];
                    if (voxInRange(newPos)) {
                        int newPosId = vox2idx(newPos);

//                        if (observationPoint(0) != 10000 && distanceBuffer[newPosId] < 0 && dist(newPos, observationPoint) < dist_tmp) {
//                            distanceBuffer[newPosId] = inf;
//                            insertIntoList(special4undefined, newPosId);
//                        }

//                        if (observationPoint(0) != 10000 && distanceBuffer[newPosId] < 0 &&
//                            dist(newPos, observationPoint) < dist_tmp) {
//                            int sum = 0;
//                            for (int j = 0; j < dirNum; j++) {
//                                Eigen::Vector3i newPos2 = newPos + dir[j];
//                                if (voxInRange(newPos2)) {
//                                    sum += (distanceBuffer[vox2idx(newPos2)] >= 0);
//                                }
//                            }
//                            if (sum >= 2) {
//                                distanceBuffer[newPosId] = inf;
//                                insertIntoList(special4undefined, newPosId);
//                            }
////                            distanceBuffer[newPosId] += 0.4;
////                            if (distanceBuffer[newPosId] >= 0){
////                                distanceBuffer[newPosId] = inf;
////                                insertIntoList(grid_total_size, newPosId);
////                            }
//                        }

                        double tmp = dist(newPos, closestObstacle[idx]);
                        if (distanceBuffer[newPosId] > tmp) {
                            distanceBuffer[newPosId] = tmp;
                            deleteFromList(vox2idx(closestObstacle[newPosId]), newPosId);

                            int newObsIdx = vox2idx(closestObstacle[idx]);
                            insertIntoList(newObsIdx, newPosId);
                            closestObstacle[newPosId] = closestObstacle[idx];
                            updateQueue.push(QueueElement{newPos, tmp});
                        }
                    }
                }
            }
        }
        totalTime += times;
        cout << "Expanding " << times << " nodes, with changeNum = " << changeNum << ", accumulator = " << totalTime
             << endl;
        break;
        if (head[special4undefined] == undefined) break;
        std::queue<int> q_tmp;

        for (int idx = head[special4undefined]; idx != undefined; idx = next[idx]) {
            for (int i = 0; i < dirNum; i++) {
                Eigen::Vector3i newPos = idx2vox(idx) + dir[i];
                if (voxInRange(newPos)) {
                    int newPosId = vox2idx(newPos);
                    if (distanceBuffer[newPosId] < 0) {
                        distanceBuffer[newPosId] = inf;
                        q_tmp.push(newPosId);
                    }
                    updateQueue.push(QueueElement{newPos, distanceBuffer[newPosId]});
                }
            }
        }
//        if (q_tmp.size() == 0) break;
        cout << "Q_tmp size: " << q_tmp.size() << endl;
        while (!q_tmp.empty()) {
            int x = q_tmp.front();
            q_tmp.pop();
            insertIntoList(special4undefined, x);
        }
    }
//    endTime = clock();
//    cout << "Totle Time : " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
}


int ESDF_Map::setOccupancy(Eigen::Vector3d pos, int occ) {
    if (occ != 1 && occ != 0) {
        cout << "occ value error!" << endl;
        return undefined;
    }

    if (!posInMap(pos)) {
//        cout << "Not in map" << endl;
        return undefined;
    }

    Eigen::Vector3i vox;
    pos2vox(pos, vox);
    return setOccupancy(vox, occ);
}

#ifndef HASH_TABLE
std::mutex mtx;
#endif

int ESDF_Map::setOccupancy(Eigen::Vector3i vox, int occ) {
    int idx = vox2idx(vox);//, idx2 = vox2idx(vox, 2);

    if (!voxInRange(vox)) return idx;

#ifdef PROBABILISTIC
    tmp2[idx]++;
    tmp1[idx] += occ;
    if (tmp2[idx] == 1) {
#ifdef HASH_TABLE
        updateQueue2.push(QueueElement{vox, 0.0});
#else
        // multi-thread
        // mtx.lock();
        updateQueue2.push(QueueElement{vox, 0.0});
        // mtx.unlock();
#endif
    }

    return idx;
#else
    if (occupancyBuffer[idx] != occ && occupancyBuffer[idx] != (occ | 2)) {
//        cout << occupancyBuffer[idx] << "\t" << occ << endl;
        if (occ == 1) insertQueue.push(QueueElement{vox, 0.0});
        else deleteQueue.push(QueueElement{vox, inf});
    }
    occupancyBuffer[idx] = occ;
    if (distanceBuffer[idx] < 0) {
        distanceBuffer[idx] = inf;
        insertIntoList(special4undefined, idx);
    }
#endif
}


bool ESDF_Map::checkUpdate() {
#ifdef PROBABILISTIC
    return updateQueue2.size() > 0;
#else
    return true;
#endif
}

bool ESDF_Map::updateOccupancy(bool globalMap) {
#ifdef PROBABILISTIC


    cout << "Occupancy Update" << ' ' << updateQueue2.size() << '\t';
    double influence = 0.2;
    while (!updateQueue2.empty()) {
        QueueElement xx = updateQueue2.front();
        updateQueue2.pop();
        int idx = vox2idx(xx.point);
        int occupy = exist(idx);
        double log_odds_update = (tmp1[idx] >= tmp2[idx] - tmp1[idx] ? prob_hit_log_ : prob_miss_log_);

        tmp1[idx] = tmp2[idx] = 0;
        if (distanceBuffer[idx] < 0) {
            distanceBuffer[idx] = inf;
            insertIntoList(special4undefined, idx);
        }
        if ((log_odds_update >= 0 &&
             occupancyBuffer[idx] >= clamp_max_log_) ||
            (log_odds_update <= 0 &&
             occupancyBuffer[idx] <= clamp_min_log_)) {
            continue;
        }
        if (!globalMap && !voxInRange(xx.point, false)){
            occupancyBuffer[idx] = 0;
        }
        occupancyBuffer[idx] = std::min(
                std::max(occupancyBuffer[idx] + log_odds_update, clamp_min_log_),
                clamp_max_log_);
        if (exist(idx) && !occupy) {
            insertQueue.push(QueueElement{xx.point, 0.0});
        } else if (!exist(idx) && occupy) {
            deleteQueue.push(QueueElement{xx.point, inf});
        }
    }
#endif
    return insertQueue.size() > 0 || deleteQueue.size() > 0;
}

int ESDF_Map::getOccupancy(Eigen::Vector3d pos) {
    if (!posInMap(pos)) return undefined;

    Eigen::Vector3i vox;
    pos2vox(pos, vox);

    return exist(vox2idx(vox));
}

int ESDF_Map::getOccupancy(Eigen::Vector3i vox) {
    // TODO: no boundary check
    return exist(vox2idx(vox));
}

double ESDF_Map::getDistance(Eigen::Vector3d pos) {
    if (!posInMap(pos)) return undefined;

    Eigen::Vector3i vox;
    pos2vox(pos, vox);

    return getDistance(vox);
}

double ESDF_Map::getDistance(Eigen::Vector3i vox) {
    return distanceBuffer[vox2idx(vox)] < 0 ? inf : distanceBuffer[vox2idx(vox)];
}

// region VISUALIZATION
void ESDF_Map::getPointCloud(sensor_msgs::PointCloud &m, int vis_lower_bound, int vis_upper_bound) {
    m.header.frame_id = "world";
    m.points.clear();
#ifdef HASH_TABLE
    for (int i = 1; i < count; i++) {
        if (!exist(vox2idx(voxBuffer[i])) || voxBuffer[i].z() < vis_lower_bound || voxBuffer[i].z() > vis_upper_bound
         || voxBuffer[i].x()<min_vec(0) || voxBuffer[i].x()>max_vec(0)
        || voxBuffer[i].y()<min_vec(1) || voxBuffer[i].y()>max_vec(1))
            continue;

        Eigen::Vector3d pos;
        vox2pos(Eigen::Vector3i(voxBuffer[i]), pos);

        geometry_msgs::Point32 p;
        p.x = pos(0);
        p.y = pos(1);
        p.z = pos(2);
        m.points.push_back(p);
//        cnt++;
    }
#else
    for (int x = min_vec(0); x <= max_vec(0); ++x)
        for (int y = min_vec(1); y <= max_vec(1); ++y)
            for (int z = min_vec(2); z <= max_vec(2); ++z) {
                if (!exist(vox2idx(Eigen::Vector3i(x, y, z))) || z < vis_lower_bound || z > vis_upper_bound) continue;

                Eigen::Vector3d pos;
                vox2pos(Eigen::Vector3i(x, y, z), pos);

                geometry_msgs::Point32 p;
                p.x = pos(0);
                p.y = pos(1);
                p.z = pos(2);
                m.points.push_back(p);
            }
//    cout << m.points.size() << endl;
#endif
}


inline std_msgs::ColorRGBA rainbowColorMap(double h) {
    std_msgs::ColorRGBA color;
    color.a = 1;
    // blend over HSV-values (more colors)

    double s = 1.0;
    double v = 1.0;

    h -= floor(h);
    h *= 6;
    int i;
    double m, n, f;

    i = floor(h);
    f = h - i;
    if (!(i & 1)) f = 1 - f;  // if i is even
    m = v * (1 - s);
    n = v * (1 - s * f);

    switch (i) {
        case 6:
        case 0:
            color.r = v;
            color.g = n;
            color.b = m;
            break;
        case 1:
            color.r = n;
            color.g = v;
            color.b = m;
            break;
        case 2:
            color.r = m;
            color.g = v;
            color.b = n;
            break;
        case 3:
            color.r = m;
            color.g = n;
            color.b = v;
            break;
        case 4:
            color.r = n;
            color.g = m;
            color.b = v;
            break;
        case 5:
            color.r = v;
            color.g = m;
            color.b = n;
            break;
        default:
            color.r = 1;
            color.g = 0.5;
            color.b = 0.5;
            break;
    }

    return color;
}


void ESDF_Map::getSliceMarker(visualization_msgs::Marker &m, int slice, int id,
                              Eigen::Vector4d color, double max_dist) {
    m.header.frame_id = "world";
    m.id = id;
    m.type = visualization_msgs::Marker::POINTS;
    m.action = visualization_msgs::Marker::MODIFY;
    m.scale.x = resolution;
    m.scale.y = resolution;
    m.scale.z = resolution;
    m.pose.orientation.w = 1;
    m.pose.orientation.x = 0;
    m.pose.orientation.y = 0;
    m.pose.orientation.z = 0;

    m.points.clear();
    m.colors.clear();
    // iterate the map
    std_msgs::ColorRGBA c;
#ifdef HASH_TABLE
    // TODO: low performance, need to be modified, which is also easy
    for (int i = 1; i < count; i++) {
        int idx = vox2idx(voxBuffer[i]);
        if (voxBuffer[i].z() != slice || distanceBuffer[idx] < 0  || distanceBuffer[idx] >= inf
        || voxBuffer[i].x()<min_vec(0) || voxBuffer[i].x()>max_vec(0)
        || voxBuffer[i].y()<min_vec(1) || voxBuffer[i].y()>max_vec(1))
            continue;

        Eigen::Vector3d pos;
        vox2pos(Eigen::Vector3i(voxBuffer[i]), pos);

        geometry_msgs::Point p;
        p.x = pos(0);
        p.y = pos(1);
        p.z = pos(2);
        c = rainbowColorMap(distanceBuffer[idx] <= max_dist ? distanceBuffer[idx] / max_dist : 1);
        m.points.push_back(p);
        m.colors.push_back(c);
    }
#else
    for (int x = min_vec(0); x <= max_vec(0); ++x)
        for (int y = min_vec(1); y <= max_vec(1); ++y) {
            int z = slice;
            Eigen::Vector3i vox = Eigen::Vector3i(x, y, z);
            if (distanceBuffer[vox2idx(vox)] < 0 || distanceBuffer[vox2idx(vox)] >= inf) continue;

            Eigen::Vector3d pos;
            vox2pos(vox, pos);

            geometry_msgs::Point p;
            p.x = pos(0);
            p.y = pos(1);
            p.z = pos(2);

            c = rainbowColorMap(distanceBuffer[vox2idx(vox)] <= max_dist ? distanceBuffer[vox2idx(vox)] / max_dist : 1);
            m.points.push_back(p);
            m.colors.push_back(c);
        }
#endif
}

#ifdef DEPRECATED
void ESDF_Map::getOccupancyMarker(visualization_msgs::Marker &m, int id, Eigen::Vector4d color) {
    m.header.frame_id = "world";
    m.id = id;
    m.type = visualization_msgs::Marker::CUBE_LIST;
    m.action = visualization_msgs::Marker::MODIFY;
    m.scale.x = resolution * 0.9;
    m.scale.y = resolution * 0.9;
    m.scale.z = resolution * 0.9;
    m.pose.orientation.w = 1;
    m.pose.orientation.x = 0;
    m.pose.orientation.y = 0;
    m.pose.orientation.z = 0;

    m.color.a = color(3);
    m.color.r = color(0);
    m.color.g = color(1);
    m.color.b = color(2);
//    m.points.clear();

    // iterate the map

#ifdef HASH_TABLE
    for (int i = 1; i < count; i++) {
        if (!exist(vox2idx(voxBuffer[i]))) continue;

        Eigen::Vector3d pos;
        vox2pos(Eigen::Vector3i(voxBuffer[i]), pos);

        geometry_msgs::Point p;
        p.x = pos(0);
        p.y = pos(1);
        p.z = pos(2);
        m.points.push_back(p);
    }
#else
    for (int x = 0; x < grid_size(0); ++x)
        for (int y = 0; y < grid_size(1); ++y)
            for (int z = 0; z < grid_size(2); ++z) {
                if (!exist(vox2idx(Eigen::Vector3i(x, y, z)))) continue;

                Eigen::Vector3d pos;
                vox2pos(Eigen::Vector3i(x, y, z), pos);

                geometry_msgs::Point p;
                p.x = pos(0);
                p.y = pos(1);
                p.z = pos(2);
                m.points.push_back(p);
            }
#endif
}

visualization_msgs::Marker m[51];
void ESDF_Map::getESDFMarker(std::vector<visualization_msgs::Marker> &markers, int id, Eigen::Vector3d color) {
//    cout << 10000 << endl;
    double max_dist = *std::max_element(distanceBuffer.begin(), distanceBuffer.end());
//    cout << std::distance(distanceBuffer.begin(), distanceBuffer.end());
//    for (auto x = distanceBuffer.begin(); x != distanceBuffer.end(); x++) {
//        cout << *x << endl;
//        exit(-1);
//    }
//    cout << "max_dist:\t" << max_dist << endl;
    max_dist = 5;
    // get marker in several distance level
//    int totalLevel = max_dist / resolution;
//    cout << totalLevel << endl;
//    if (totalLevel > 50) totalLevel = 50;
    int totalLevel = 50;

    double min_a = 0.00, max_a = 0.50;
    double da = (max_a - min_a) / (totalLevel - 1);
    double delta_d = max_dist / totalLevel;
    cout << delta_d << endl;
    for (int i = 0; i < 51; ++i) {
        m[i].header.frame_id = "world";
        m[i].id = i + totalLevel * id;
        m[i].type = visualization_msgs::Marker::CUBE_LIST;
        m[i].action = visualization_msgs::Marker::ADD;
        m[i].scale.x = resolution * 0.9;
        m[i].scale.y = resolution * 0.9;
        m[i].scale.z = resolution * 0.9;
        m[i].color.r = color(0);
        m[i].color.g = color(1);
        m[i].color.b = color(2);
        m[i].color.a = min_a + da * i;
        m[i].points.clear();
    }
    m[50].color.r = 0;
    m[50].color.g = 0;
    m[50].color.b = 1;

#ifdef HASH_TABLE
    // iterate the map
    int tot0 = 0, tot1 = 0;
    for (int ii = 1; ii < count; ii++) {
        int level = int(distanceBuffer[vox2idx(voxBuffer[ii])] / delta_d);
        if (level >= totalLevel || level < 0) {
            tot0 += level < 0;
            tot1 += level >= totalLevel;
            if (level < 0) {
                continue;
                int check = 5;
                for (int i = 0; i < dirNum; i++) {
                    Eigen::Vector3i newPos = voxBuffer[ii] + dir[i];
                    if (voxInRange(newPos)) {
                        int newPosId = vox2idx(newPos);
                        if (distanceBuffer[newPosId] < 0) {
                            check--;
                            if (check == 0) break;
                        }
                    }
                }
                if (check) level = totalLevel;
                else continue;
            } else level = totalLevel;
        }
        Eigen::Vector3d pos;
        vox2pos(voxBuffer[ii], pos);

        geometry_msgs::Point p;
        p.x = pos(0);
        p.y = pos(1);
        p.z = pos(2);
        m[level].points.push_back(p);
    }
    count--;
    cout << "count: " << count << endl;
    cout << ">=0 " << tot0 << ' ' << (double) (count - tot0) / count * 100 << "%" << endl;
    cout << "inf " << tot1 << ' ' << (double) tot1 / count * 100 << "%" << endl;
    cout << (double) tot1 / (count - tot0) * 100 << "%" << endl;
    for (int i = 0; i < 5; ++i) {
        cout << i << ' ' << m[i].points.size() << ' '
             << (double) m[i].points.size() / count * 100 << "%" << endl;
    }
    int i = 20;
    cout << i << ' ' << m[i].points.size() << ' '
         << (double) m[i].points.size() / count * 100 << "%" << endl;
    count++;
#else
    // iterate the map
    int tot0 = 0, tot1 = 0;
    for (int x = 0; x < grid_size(0); ++x)
        for (int y = 0; y < grid_size(1); ++y)
            for (int z = 0; z < grid_size(2); ++z) {
                int level = int(distanceBuffer[vox2idx(Eigen::Vector3i(x, y, z))] / delta_d);
                if (level >= totalLevel || level < 0) {
                    tot0 += level < 0;
                    tot1 += level >= totalLevel;
                    if (level < 0) {
//                        continue;
                        int check = 5;
                        for (int i = 0; i < dirNum; i++) {
                            Eigen::Vector3i newPos = Eigen::Vector3i(x, y, z) + dir[i];
                            if (voxInRange(newPos)) {
                                int newPosId = vox2idx(newPos);
                                if (distanceBuffer[newPosId] < 0) {
                                    check--;
                                    if (check == 0) break;
                                }
                            }
                        }
                        if (check) level = totalLevel;
                        else continue;
                    } else level = totalLevel;
                }
                Eigen::Vector3d pos;
                vox2pos(Eigen::Vector3i(x, y, z), pos);

                geometry_msgs::Point p;
                p.x = pos(0);
                p.y = pos(1);
                p.z = pos(2);
                m[level].points.push_back(p);
            }
    cout << ">=0 " << grid_total_size - tot0 << ' ' << (double) (grid_total_size - tot0) / grid_total_size * 100 << "%"
         << endl;
    cout << "25 " << tot1 << ' ' << (double) tot1 / grid_total_size * 100 << "%" << endl;
    cout << (double) tot1 / (grid_total_size - tot0) * 100 << "%" << endl;
    for (int i = 0; i < 5; ++i) {
        cout << i << ' ' << m[i].points.size() << ' '
             << (double) m[i].points.size() / grid_total_size * 100 << "%" << endl;
    }
    int i = 50;
    cout << i << ' ' << m[i].points.size() << ' '
         << (double) m[i].points.size() / grid_total_size * 100 << "%" << endl;
#endif
    markers.clear();

    for (int i = 0; i < 51; ++i)
        markers.push_back(m[i]);
}
#endif
// endregion
