//
// Created by tommy on 12/17/18.
//

#include "ESDFMap.h"
#include <math.h>
#include <time.h>

using std::cout;
using std::endl;

const double fiesta::ESDFMap::Logit(const double &x) const {
  return log(x / (1 - x));
}

bool fiesta::ESDFMap::Exist(const int &idx) {
#ifdef PROBABILISTIC
  return occupancy_buffer_[idx] > min_occupancy_log_;
#else
  return occupancy_buffer_[idx] == 1;
#endif
}

void fiesta::ESDFMap::DeleteFromList(int link, int idx) {
  if (prev_[idx] != undefined_)
    next_[prev_[idx]] = next_[idx];
  else
    head_[link] = next_[idx];
  if (next_[idx] != undefined_)
    prev_[next_[idx]] = prev_[idx];
  prev_[idx] = next_[idx] = undefined_;
}

void fiesta::ESDFMap::InsertIntoList(int link, int idx) {
  if (head_[link] == undefined_)
    head_[link] = idx;
  else {
    prev_[head_[link]] = idx;
    next_[idx] = head_[link];
    head_[link] = idx;
  }
}

// region CONVERSION

bool fiesta::ESDFMap::PosInMap(Eigen::Vector3d pos) {
#ifdef HASH_TABLE
  return true;
#else
  if (pos(0) < min_range_(0) || pos(1) < min_range_(1) || pos(2) < min_range_(2)) {
//    cout << "less than min range!\t" << pos(0) << ' ' << pos(1) << ' ' << pos(2) << endl;
    return false;
  }

  if (pos(0) > max_range_(0) || pos(1) > max_range_(1) || pos(2) > max_range_(2)) {
//    cout << "larger than min range!\t" << pos(0) << ' ' << pos(1) << ' ' << pos(2) << endl;
    return false;
  }
  return true;
#endif
}

bool fiesta::ESDFMap::VoxInRange(Eigen::Vector3i vox, bool current_vec) {
  if (current_vec)
    return (vox(0) >= min_vec_(0) && vox(0) <= max_vec_(0)
        && vox(1) >= min_vec_(1) && vox(1) <= max_vec_(1)
        && vox(2) >= min_vec_(2) && vox(2) <= max_vec_(2));
  else
    return (vox(0) >= last_min_vec_(0) && vox(0) <= last_max_vec_(0)
        && vox(1) >= last_min_vec_(1) && vox(1) <= last_max_vec_(1)
        && vox(2) >= last_min_vec_(2) && vox(2) <= last_max_vec_(2));
}

void fiesta::ESDFMap::Pos2Vox(Eigen::Vector3d pos, Eigen::Vector3i &vox) {
  for (int i = 0; i < 3; ++i)
    vox(i) = floor((pos(i) - origin_(i)) / resolution_);
}

void fiesta::ESDFMap::Vox2Pos(Eigen::Vector3i vox, Eigen::Vector3d &pos) {
  for (int i = 0; i < 3; ++i)
    pos(i) = (vox(i) + 0.5) * resolution_ + origin_(i);
}

int fiesta::ESDFMap::Vox2Idx(Eigen::Vector3i vox) {
#ifdef HASH_TABLE
  if (vox(0) == undefined_) return reserved_idx_4_undefined_;
  return FindAndInsert(Eigen::Vector3i(vox(0), vox(1), vox(2)));
#else
  if (vox(0) == undefined_)
    return reserved_idx_4_undefined_;
  return vox(0) * grid_size_yz_ + vox(1) * grid_size_(2) + vox(2);
#endif
}

int fiesta::ESDFMap::Vox2Idx(Eigen::Vector3i vox, int sub_sampling_factor) {
#ifdef HASH_TABLE
  // TODO: ignore sub_sampling_factor
  if (vox(0) == undefined_) return reserved_idx_4_undefined_;
  return FindAndInsert(Eigen::Vector3i(vox(0), vox(1), vox(2)));
#else
  if (vox(0) == undefined_)
    return reserved_idx_4_undefined_;
  return vox(0) * grid_size_yz_ / sub_sampling_factor / sub_sampling_factor / sub_sampling_factor
      + vox(1) * grid_size_(2) / sub_sampling_factor / sub_sampling_factor
      + vox(2) / sub_sampling_factor;
#endif
}

Eigen::Vector3i fiesta::ESDFMap::Idx2Vox(int idx) {
#ifdef HASH_TABLE
  return vox_buffer_[idx];
#else
  return Eigen::Vector3i(idx / grid_size_yz_,
                         idx % (grid_size_yz_) / grid_size_(2),
                         idx % grid_size_(2));
#endif

}

// endregion

double fiesta::ESDFMap::Dist(Eigen::Vector3i a, Eigen::Vector3i b) {
  return (b - a).cast<double>().norm() * resolution_;
//        return (b - a).squaredNorm();
// TODO: may use square root & * resolution_ at last together to speed up
}

#ifdef HASH_TABLE

fiesta::ESDFMap::ESDFMap(Eigen::Vector3d origin, double resolution_, int reserve_size)
    : origin_(origin), resolution_(resolution_) {
  resolution_inv_ = 1 / resolution_;
  infinity_ = 10000;
  undefined_ = -10000;
  reserved_idx_4_undefined_ = 0;
#ifdef BLOCK
  block_bit_ = 3;
  block_ = (1 << block_bit_);

  block_size_ = block_ * block_ * block_;
  if (block_size_ > reserve_size) reserve_size = block_size_;
#endif
  SetOriginalRange();
  count = 1; // 0 is used for special use
  reserve_size_ = reserve_size + 1;
  occupancy_buffer_.resize(reserve_size_);
  distance_buffer_.resize(reserve_size_);
  closest_obstacle_.resize(reserve_size_);
  vox_buffer_.resize(reserve_size_);
  num_hit_.resize(reserve_size_);
  num_miss_.resize(reserve_size_);

  std::fill(distance_buffer_.begin(), distance_buffer_.end(), (double) undefined_);
  std::fill(occupancy_buffer_.begin(), occupancy_buffer_.end(), 0);
  std::fill(closest_obstacle_.begin(), closest_obstacle_.end(), Eigen::Vector3i(undefined_, undefined_, undefined_));
  std::fill(vox_buffer_.begin(), vox_buffer_.end(), Eigen::Vector3i(undefined_, undefined_, undefined_));
  std::fill(num_hit_.begin(), num_hit_.end(), 0);
  std::fill(num_miss_.begin(), num_miss_.end(), 0);

  head_.resize(reserve_size_);
  prev_.resize(reserve_size_);
  next_.resize(reserve_size_);
  std::fill(head_.begin(), head_.end(), undefined_);
  std::fill(prev_.begin(), prev_.end(), undefined_);
  std::fill(next_.begin(), next_.end(), undefined_);

}

#else

fiesta::ESDFMap::ESDFMap(Eigen::Vector3d origin, double resolution_, Eigen::Vector3d map_size)
    : origin_(origin), resolution_(resolution_), map_size_(map_size) {
  resolution_inv_ = 1 / resolution_;

  for (int i = 0; i < 3; ++i)
    grid_size_(i) = ceil(map_size(i) / resolution_);

  min_range_ = origin;
  max_range_ = origin + map_size;
  grid_size_yz_ = grid_size_(1) * grid_size_(2);
  infinity_ = 10000;
  undefined_ = -10000;

  grid_total_size_ = grid_size_(0) * grid_size_yz_;
  reserved_idx_4_undefined_ = grid_total_size_;
  SetOriginalRange();

  cout << grid_total_size_ << endl;
  occupancy_buffer_.resize(grid_total_size_);

  distance_buffer_.resize(grid_total_size_);
//    distanceBufferNegative.resize(grid_total_size_);

  closest_obstacle_.resize(grid_total_size_);
  num_hit_.resize(grid_total_size_);
  num_miss_.resize(grid_total_size_);

  std::fill(distance_buffer_.begin(), distance_buffer_.end(), (double) undefined_);
//    std::fill(distanceBufferNegative.begin(), distanceBufferNegative.end(), (double) undefined_);

  std::fill(occupancy_buffer_.begin(), occupancy_buffer_.end(), 0);
  std::fill(closest_obstacle_.begin(), closest_obstacle_.end(), Eigen::Vector3i(undefined_, undefined_, undefined_));
  std::fill(num_hit_.begin(), num_hit_.end(), 0);
  std::fill(num_miss_.begin(), num_miss_.end(), 0);

  head_.resize(grid_total_size_ + 1);
  prev_.resize(grid_total_size_);
  next_.resize(grid_total_size_);
  std::fill(head_.begin(), head_.end(), undefined_);
  std::fill(prev_.begin(), prev_.end(), undefined_);
  std::fill(next_.begin(), next_.end(), undefined_);

}

#endif

#ifdef PROBABILISTIC
void fiesta::ESDFMap::SetParameters(double p_hit, double p_miss, double p_min, double p_max, double p_occ) {
  prob_hit_log_ = Logit(p_hit);
  prob_miss_log_ = Logit(p_miss);
  clamp_min_log_ = Logit(p_min);
  clamp_max_log_ = Logit(p_max);
  min_occupancy_log_ = Logit(p_occ);
}
#endif

bool fiesta::ESDFMap::CheckUpdate() {
#ifdef PROBABILISTIC
  return !occupancy_queue_.empty();
#else
  return true;
#endif
}

bool fiesta::ESDFMap::UpdateOccupancy(bool global_map) {
#ifdef PROBABILISTIC
  cout << "Occupancy Update" << ' ' << occupancy_queue_.size() << '\t';
  while (!occupancy_queue_.empty()) {
    QueueElement xx = occupancy_queue_.front();
    occupancy_queue_.pop();
    int idx = Vox2Idx(xx.point_);
    int occupy = Exist(idx);
    double log_odds_update = (num_hit_[idx] >= num_miss_[idx] - num_hit_[idx] ? prob_hit_log_ : prob_miss_log_);

    num_hit_[idx] = num_miss_[idx] = 0;
    if (distance_buffer_[idx] < 0) {
      distance_buffer_[idx] = infinity_;
      InsertIntoList(reserved_idx_4_undefined_, idx);
    }
    if ((log_odds_update >= 0 &&
        occupancy_buffer_[idx] >= clamp_max_log_) ||
        (log_odds_update <= 0 &&
            occupancy_buffer_[idx] <= clamp_min_log_)) {
      continue;
    }
    if (!global_map && !VoxInRange(xx.point_, false)) {
      occupancy_buffer_[idx] = 0;
      distance_buffer_[idx] = infinity_;
    }
    occupancy_buffer_[idx] = std::min(
        std::max(occupancy_buffer_[idx] + log_odds_update, clamp_min_log_),
        clamp_max_log_);
    if (Exist(idx) && !occupy) {
      insert_queue_.push(QueueElement{xx.point_, 0.0});
    } else if (!Exist(idx) && occupy) {
      delete_queue_.push(QueueElement{xx.point_, (double) infinity_});
    }
  }
#endif
  return !insert_queue_.empty() || !delete_queue_.empty();
}

void fiesta::ESDFMap::UpdateESDF() {
//    clock_t startTime,endTime;
//    startTime = clock();
//    UpdateOccupancy();
  cout << "Insert " << insert_queue_.size() << "\tDelete " << delete_queue_.size() << endl;
  while (!insert_queue_.empty()) {
    QueueElement xx = insert_queue_.front();
    insert_queue_.pop();
    int idx = Vox2Idx(xx.point_);
    if (Exist(idx)) {
      // Exist after a whole brunch of updates
      // delete previous link & create a new linked-list
      DeleteFromList(Vox2Idx(closest_obstacle_[idx]), idx);
      closest_obstacle_[idx] = xx.point_;
      distance_buffer_[idx] = 0.0;
      InsertIntoList(idx, idx);
      update_queue_.push(xx);
    }
  }
  while (!delete_queue_.empty()) {
    QueueElement xx = delete_queue_.front();

    delete_queue_.pop();
    int idx = Vox2Idx(xx.point_);
    if (!Exist(idx)) {
      // doesn't Exist after a whole brunch of updates

      int next_obs_idx;
      for (int obs_idx = head_[idx]; obs_idx != undefined_; obs_idx = next_obs_idx) {

        closest_obstacle_[obs_idx] = Eigen::Vector3i(undefined_, undefined_, undefined_);
        Eigen::Vector3i obs_vox = Idx2Vox(obs_idx);

        double distance = infinity_;
        // find neighborhood whose closest obstacles Exist
        for (const auto &dir : dirs_) {
          Eigen::Vector3i new_pos = obs_vox + dir;
          int new_pos_idx = Vox2Idx(new_pos);
          if (VoxInRange(new_pos) && closest_obstacle_[new_pos_idx](0) != undefined_
              && Exist(Vox2Idx(closest_obstacle_[new_pos_idx]))) {
            // if in range and closest obstacles Exist
            double tmp = Dist(obs_vox, closest_obstacle_[new_pos_idx]);
            if (tmp < distance) {
              distance = tmp;
              closest_obstacle_[obs_idx] = closest_obstacle_[new_pos_idx];
            }
            break;
          } // if
        } // for neighborhood

        // destroy the linked-list
        prev_[obs_idx] = undefined_;
        next_obs_idx = next_[obs_idx];
        next_[obs_idx] = undefined_;

        distance_buffer_[obs_idx] = distance;
        if (distance < infinity_) {
          update_queue_.push(QueueElement{obs_vox, distance});
        }
        int new_obs_idx = Vox2Idx(closest_obstacle_[obs_idx]);
        InsertIntoList(new_obs_idx, obs_idx);
      } // for obs_idx
      head_[idx] = undefined_;
    } // if
  } // delete_queue_
  int times = 0, change_num = 0;
  while (!update_queue_.empty()) {
    QueueElement xx = update_queue_.front();
//            QueueElement xx = update_queue_.top();

    update_queue_.pop();
    int idx = Vox2Idx(xx.point_);
    if (xx.distance_ != distance_buffer_[idx])
      continue;
    times++;
    bool change = false;
    for (int i = 0; i < num_dirs_; i++) {
      Eigen::Vector3i new_pos = xx.point_ + dirs_[i];
      if (VoxInRange(new_pos)) {
        int new_pos_idx = Vox2Idx(new_pos);
        if (closest_obstacle_[new_pos_idx](0) == undefined_)
          continue;
        double tmp = Dist(xx.point_, closest_obstacle_[new_pos_idx]);

        if (distance_buffer_[idx] > tmp) {
          distance_buffer_[idx] = tmp;
          change = true;
          DeleteFromList(Vox2Idx(closest_obstacle_[idx]), idx);

          int new_obs_idx = Vox2Idx(closest_obstacle_[new_pos_idx]);
          InsertIntoList(new_obs_idx, idx);
          closest_obstacle_[idx] = closest_obstacle_[new_pos_idx];
        }
      }
    }

    if (change) {
      change_num++;
      update_queue_.push(QueueElement{xx.point_, distance_buffer_[idx]});
      continue;
    }

    int new_obs_idx = Vox2Idx(closest_obstacle_[idx]);
    for (const auto &dir : dirs_) {
      Eigen::Vector3i new_pos = xx.point_ + dir;
      if (VoxInRange(new_pos)) {
        int new_pos_id = Vox2Idx(new_pos);

        double tmp = Dist(new_pos, closest_obstacle_[idx]);
        if (distance_buffer_[new_pos_id] > tmp) {
          distance_buffer_[new_pos_id] = tmp;
          DeleteFromList(Vox2Idx(closest_obstacle_[new_pos_id]), new_pos_id);

          InsertIntoList(new_obs_idx, new_pos_id);
          closest_obstacle_[new_pos_id] = closest_obstacle_[idx];
          update_queue_.push(QueueElement{new_pos, tmp});
        }
      }
    }
  }
  total_time_ += times;
  cout << "Expanding " << times << " nodes, with change_num = " << change_num << ", accumulator = " << total_time_
       << endl;
//    endTime = clock();
//    cout << "Totle Time : " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
}


int fiesta::ESDFMap::SetOccupancy(Eigen::Vector3d pos, int occ) {
  if (occ != 1 && occ != 0) {
    cout << "occ value error!" << endl;
    return undefined_;
  }

  if (!PosInMap(pos)) {
//        cout << "Not in map" << endl;
    return undefined_;
  }

  Eigen::Vector3i vox;
  Pos2Vox(pos, vox);
  return SetOccupancy(vox, occ);
}

int fiesta::ESDFMap::SetOccupancy(Eigen::Vector3i vox, int occ) {
  int idx = Vox2Idx(vox);//, idx2 = Vox2Idx(vox, 2);

  if (!VoxInRange(vox))
    return idx;

#ifdef PROBABILISTIC
  num_miss_[idx]++;
  num_hit_[idx] += occ;
  if (num_miss_[idx] == 1) {
#ifdef HASH_TABLE
    occupancy_queue_.push(QueueElement{vox, 0.0});
#else
    // multi-thread
    // mtx.lock();
    occupancy_queue_.push(QueueElement{vox, 0.0});
    // mtx.unlock();
#endif
  }

  return idx;
#else
  if (occupancy_buffer_[idx] != occ && occupancy_buffer_[idx] != (occ | 2)) {
//        cout << occupancy_buffer_[idx] << "\t" << occ << endl;
      if (occ == 1) insert_queue_.push(QueueElement{vox, 0.0});
      else delete_queue_.push(QueueElement{vox, (double)infinity_});
  }
  occupancy_buffer_[idx] = occ;
  if (distance_buffer_[idx] < 0) {
      distance_buffer_[idx] = infinity_;
      InsertIntoList(reserved_idx_4_undefined_, idx);
  }
#endif
}

int fiesta::ESDFMap::GetOccupancy(Eigen::Vector3d pos) {
  if (!PosInMap(pos))
    return undefined_;

  Eigen::Vector3i vox;
  Pos2Vox(pos, vox);

  return Exist(Vox2Idx(vox));
}

int fiesta::ESDFMap::GetOccupancy(Eigen::Vector3i pos_id) {
  // TODO: no boundary check
  return Exist(Vox2Idx(std::move(pos_id)));
}

double fiesta::ESDFMap::GetDistance(Eigen::Vector3d pos) {
  if (!PosInMap(pos))
    return undefined_;

  Eigen::Vector3i vox;
  Pos2Vox(pos, vox);

  return GetDistance(vox);
}

double fiesta::ESDFMap::GetDistance(Eigen::Vector3i vox) {
  return distance_buffer_[Vox2Idx(vox)] < 0 ? infinity_ : distance_buffer_[Vox2Idx(vox)];
}

double fiesta::ESDFMap::GetDistWithGradTrilinear(Eigen::Vector3d pos,
                                                 Eigen::Vector3d &grad) {
  if (!PosInMap(pos))
    return -1;

  /* this is too young too simple */
  // gradient(0) = (GetDistance(id(0) + 1, id(1), id(2)) -
  //                GetDistance(id(0) - 1, id(1), id(2))) *
  //               0.5 * resolution_inv_;
  // gradient(1) = (GetDistance(id(0), id(1) + 1, id(2)) -
  //                GetDistance(id(0), id(1) - 1, id(2))) *
  //               0.5 * resolution_inv_;
  // gradient(2) = (GetDistance(id(0), id(1), id(2) + 1) -
  //                GetDistance(id(0), id(1), id(2) - 1)) *
  //               0.5 * resolution_inv_;

  /* use trilinear interpolation */
  Eigen::Vector3d pos_m = pos - 0.5 * resolution_ * Eigen::Vector3d::Ones();

  Eigen::Vector3i idx;
  Pos2Vox(pos_m, idx);
//    cout << voxInMap(idx) << endl;

  Eigen::Vector3d idx_pos, diff;
  Vox2Pos(idx, idx_pos);

  diff = (pos - idx_pos) * resolution_inv_;

  double values[2][2][2];
  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
      for (int z = 0; z < 2; z++) {
        Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
        values[x][y][z] = GetDistance(current_idx);
      }
    }
  }

  double v00 = (1 - diff[0]) * values[0][0][0] + diff[0] * values[1][0][0];
  double v01 = (1 - diff[0]) * values[0][0][1] + diff[0] * values[1][0][1];
  double v10 = (1 - diff[0]) * values[0][1][0] + diff[0] * values[1][1][0];
  double v11 = (1 - diff[0]) * values[0][1][1] + diff[0] * values[1][1][1];

  double v0 = (1 - diff[1]) * v00 + diff[1] * v10;
  double v1 = (1 - diff[1]) * v01 + diff[1] * v11;

  double dist = (1 - diff[2]) * v0 + diff[2] * v1;

  grad[2] = (v1 - v0) * resolution_inv_;
  grad[1] =
      ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * resolution_inv_;
  grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
  grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
  grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
  grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);

  grad[0] *= resolution_inv_;

  return dist;
}

// region VISUALIZATION

void fiesta::ESDFMap::GetPointCloud(sensor_msgs::PointCloud &m, int vis_lower_bound, int vis_upper_bound) {
  m.header.frame_id = "world";
  m.points.clear();
#ifdef HASH_TABLE
  for (int i = 1; i < count; i++) {
    if (!Exist(Vox2Idx(vox_buffer_[i])) || vox_buffer_[i].z() < vis_lower_bound || vox_buffer_[i].z() > vis_upper_bound
        || vox_buffer_[i].x() < min_vec_(0) || vox_buffer_[i].x() > max_vec_(0)
        || vox_buffer_[i].y() < min_vec_(1) || vox_buffer_[i].y() > max_vec_(1))
      continue;

    Eigen::Vector3d pos;
    Vox2Pos(Eigen::Vector3i(vox_buffer_[i]), pos);

    geometry_msgs::Point32 p;
    p.x = pos(0);
    p.y = pos(1);
    p.z = pos(2);
    m.points.push_back(p);
    //        cnt++;
  }
#else
  for (int x = min_vec_(0); x <= max_vec_(0); ++x)
    for (int y = min_vec_(1); y <= max_vec_(1); ++y)
      for (int z = min_vec_(2); z <= max_vec_(2); ++z) {
        if (!Exist(Vox2Idx(Eigen::Vector3i(x, y, z))) || z < vis_lower_bound || z > vis_upper_bound)
          continue;

        Eigen::Vector3d pos;
        Vox2Pos(Eigen::Vector3i(x, y, z), pos);

        geometry_msgs::Point32 p;
        p.x = pos(0);
        p.y = pos(1);
        p.z = pos(2);
        m.points.push_back(p);
      }
//    cout << m.points.size() << endl;
#endif
}

inline std_msgs::ColorRGBA RainbowColorMap(double h) {
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
  if (!(i & 1))
    f = 1 - f;  // if i is even
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i) {
    case 6:
    case 0:color.r = v;
      color.g = n;
      color.b = m;
      break;
    case 1:color.r = n;
      color.g = v;
      color.b = m;
      break;
    case 2:color.r = m;
      color.g = v;
      color.b = n;
      break;
    case 3:color.r = m;
      color.g = n;
      color.b = v;
      break;
    case 4:color.r = n;
      color.g = m;
      color.b = v;
      break;
    case 5:color.r = v;
      color.g = m;
      color.b = n;
      break;
    default:color.r = 1;
      color.g = 0.5;
      color.b = 0.5;
      break;
  }

  return color;
}

void fiesta::ESDFMap::GetSliceMarker(visualization_msgs::Marker &m, int slice, int id,
                                     Eigen::Vector4d color, double max_dist) {
  m.header.frame_id = "world";
  m.id = id;
  m.type = visualization_msgs::Marker::POINTS;
  m.action = visualization_msgs::Marker::MODIFY;
  m.scale.x = resolution_;
  m.scale.y = resolution_;
  m.scale.z = resolution_;
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
    int idx = Vox2Idx(vox_buffer_[i]);
    if (vox_buffer_[i].z() != slice || distance_buffer_[idx] < 0 || distance_buffer_[idx] >= infinity_
        || vox_buffer_[i].x() < min_vec_(0) || vox_buffer_[i].x() > max_vec_(0)
        || vox_buffer_[i].y() < min_vec_(1) || vox_buffer_[i].y() > max_vec_(1))
      continue;

    Eigen::Vector3d pos;
    Vox2Pos(Eigen::Vector3i(vox_buffer_[i]), pos);

    geometry_msgs::Point p;
    p.x = pos(0);
    p.y = pos(1);
    p.z = pos(2);
    c = RainbowColorMap(distance_buffer_[idx] <= max_dist ? distance_buffer_[idx] / max_dist : 1);
    m.points.push_back(p);
    m.colors.push_back(c);
  }
#else
  for (int x = min_vec_(0); x <= max_vec_(0); ++x)
    for (int y = min_vec_(1); y <= max_vec_(1); ++y) {
      int z = slice;
      Eigen::Vector3i vox = Eigen::Vector3i(x, y, z);
      if (distance_buffer_[Vox2Idx(vox)] < 0 || distance_buffer_[Vox2Idx(vox)] >= infinity_)
        continue;

      Eigen::Vector3d pos;
      Vox2Pos(vox, pos);

      geometry_msgs::Point p;
      p.x = pos(0);
      p.y = pos(1);
      p.z = pos(2);

      c = RainbowColorMap(
          distance_buffer_[Vox2Idx(vox)] <= max_dist ? distance_buffer_[Vox2Idx(vox)] / max_dist : 1);
      m.points.push_back(p);
      m.colors.push_back(c);
    }
#endif
}

// endregion

// region HASH_TABLE
#ifdef HASH_TABLE
void fiesta::ESDFMap::IncreaseCapacity(int &old_size, int new_size) {
  occupancy_buffer_.resize(new_size);
  distance_buffer_.resize(new_size);
  closest_obstacle_.resize(new_size);
  vox_buffer_.resize(new_size);
  num_hit_.resize(new_size);
  num_miss_.resize(new_size);

  std::fill(distance_buffer_.begin() + old_size, distance_buffer_.end(), (double) undefined_);
  std::fill(occupancy_buffer_.begin() + old_size, occupancy_buffer_.end(), 0);
  std::fill(closest_obstacle_.begin() + old_size, closest_obstacle_.end(),
            Eigen::Vector3i(undefined_, undefined_, undefined_));
  std::fill(vox_buffer_.begin() + old_size, vox_buffer_.end(), Eigen::Vector3i(undefined_, undefined_, undefined_));
  std::fill(num_hit_.begin() + old_size, num_hit_.end(), 0);
  std::fill(num_miss_.begin() + old_size, num_miss_.end(), 0);

  head_.resize(new_size);
  prev_.resize(new_size);
  next_.resize(new_size);
  std::fill(head_.begin() + old_size, head_.end(), undefined_);
  std::fill(prev_.begin() + old_size, prev_.end(), undefined_);
  std::fill(next_.begin() + old_size, next_.end(), undefined_);

  printf("\tnew_size:\t%d\n", new_size);
  old_size = new_size;
}

int fiesta::ESDFMap::FindAndInsert(Eigen::Vector3i hash_key) {
#ifdef BLOCK
  if (count + block_size_ > reserve_size_)
    IncreaseCapacity(reserve_size_, reserve_size_ * 2);
#ifdef BITWISE
  int idx_in_block = ((hash_key.x() & (block_ - 1)) << block_bit_ << block_bit_)
      + ((hash_key.y() & (block_ - 1)) << block_bit_)
      + (hash_key.z() & (block_ - 1));
  Eigen::Vector3i block_id = hash_key.unaryExpr([&](const int x) { return x >> block_bit_; });
#else
  int idx_in_block = (hash_key.x() % block_ + block_) % block_ * block_ * block_
                     + (hash_key.y() % block_ + block_) % block_ * block_
                     + (hash_key.z() % block_ + block_) % block_;
    Eigen::Vector3i block_id = hash_key.unaryExpr([&](const int x) { return x / block_ - (x % block_ < 0); });
#endif
  auto tmp = hash_table_.find(block_id);

  if (tmp == hash_table_.end()) {
    hash_table_.insert(std::make_pair(block_id, count));
#ifdef BITWISE
    block_id = block_id.unaryExpr([&](const int x) { return x << block_bit_; });
#else
    block_id = block_id * block_;
#endif
    for (int i = 0; i < block_; i++)
      for (int j = 0; j < block_; j++)
        for (int k = 0; k < block_; k++) {
          vox_buffer_[count++] = block_id + Eigen::Vector3i(i, j, k);
        }
    return count - block_size_ + idx_in_block;
  } else {
    return tmp->second + idx_in_block;
  }

#else

  if (count >= reserve_size_)
      IncreaseCapacity(reserve_size_, reserve_size_ * 2);
  auto tmp = hash_table_.find(hashKey);

  if (tmp == hash_table_.end()) {
      hash_table_.insert(std::make_pair(hashKey, count));
      vox_buffer_[count] = hashKey;
      return count++;
  } else {
      return tmp->second;
  }


  }
#endif
}

#endif

// endregion


//region LOCAL vs GLOBAL

void fiesta::ESDFMap::SetUpdateRange(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos, bool new_vec) {
#ifndef HASH_TABLE
  min_pos(0) = std::max(min_pos(0), min_range_(0));
    min_pos(1) = std::max(min_pos(1), min_range_(1));
    min_pos(2) = std::max(min_pos(2), min_range_(2));

    max_pos(0) = std::min(max_pos(0), max_range_(0));
    max_pos(1) = std::min(max_pos(1), max_range_(1));
    max_pos(2) = std::min(max_pos(2), max_range_(2));
#endif
  if (new_vec) {
    last_min_vec_ = min_vec_;
    last_max_vec_ = max_vec_;
  }
  Pos2Vox(min_pos, min_vec_);
  Pos2Vox(
      max_pos - Eigen::Vector3d(resolution_ / 2, resolution_ / 2, resolution_ / 2),
      max_vec_);
}

void fiesta::ESDFMap::SetOriginalRange() {
#ifdef HASH_TABLE
  min_vec_ << -infinity_, -infinity_, -infinity_;
  max_vec_ << +infinity_, +infinity_, +infinity_;
  last_min_vec_ = min_vec_;
  last_max_vec_ = max_vec_;
#else
  min_vec_ << 0, 0, 0;
    max_vec_ << grid_size_(0) - 1, grid_size_(1) - 1, grid_size_(2) - 1;
    last_min_vec_ = min_vec_;
    last_max_vec_ = max_vec_;
#endif
}

#ifndef PROBABILISTIC
void SetAway(){
    SetAway(min_vec_, max_vec_);
  }

  void SetAway(Eigen::Vector3i left, Eigen::Vector3i right) {
      for (int i = left(0); i <= right(0); i++)
          for (int j = left(1); j <= right(1); j++)
              for (int k = left(2); k <= right(2); k++)
                  occupancy_buffer_[Vox2Idx(Eigen::Vector3i(i, j, k))] |= 2;
  }
  void SetBack(){
    SetBack(min_vec_, max_vec_);
  }
  void SetBack(Eigen::Vector3i left, Eigen::Vector3i right) {
      for (int i = left(0); i <= right(0); i++)
          for (int j = left(1); j <= right(1); j++)
              for (int k = left(2); k <= right(2); k++)
                  if (occupancy_buffer_[Vox2Idx(Eigen::Vector3i(i, j, k))] >= 2)
                      SetOccupancy(Eigen::Vector3i(i, j, k), 0);
  }
#endif

//endregion

#ifdef DEBUG

// region DEBUG

// only for test, check whether consistent
bool fiesta::ESDFMap::CheckConsistency() {
#ifdef HASH_TABLE
  for (int ii = 1; ii < count; ii++) {
    int vox = Vox2Idx(vox_buffer_[ii]);
    if (prev_[vox] != undefined_ && next_[prev_[vox]] != vox
        || next_[vox] != undefined_ && prev_[next_[vox]] != vox) {
      std::cout << vox_buffer_[ii](0) << ' ' << vox_buffer_[ii](1) << ' ' << vox_buffer_[ii](2) << ' ' << vox
                << std::endl;
      std::cout << prev_[vox] << ' ' << next_[prev_[vox]] << ' ' << next_[vox] << ' ' << prev_[next_[vox]]
                << std::endl;
      std::cout << Vox2Idx(closest_obstacle_[next_[prev_[vox]]]) << ' ' << Vox2Idx(closest_obstacle_[vox])
                << std::endl;
      std::cout << Vox2Idx(closest_obstacle_[prev_[vox]]) << ' ' << Vox2Idx(closest_obstacle_[next_[vox]])
                << std::endl;

      return false;
    }
    if (prev_[vox] == undefined_ && distance_buffer_[vox] >= 0
        && head_[Vox2Idx(closest_obstacle_[vox])] != vox)
      return false;
  }

#else

  for (int x = 0; x < grid_size_(0); ++x)
      for (int y = 0; y < grid_size_(1); ++y)
        for (int z = 0; z < grid_size_(2); ++z) {
          int vox = Vox2Idx(Eigen::Vector3i(x, y, z));
          if (prev_[vox] != undefined_ && next_[prev_[vox]] != vox ||
              next_[vox] != undefined_ && prev_[next_[vox]] != vox) {
            std::cout << x << ' ' << y << ' ' << z << ' ' << vox << std::endl;
            std::cout << prev_[vox] << ' ' << next_[prev_[vox]] << ' ' << next_[vox] << ' ' << prev_[next_[vox]]
                      << std::endl;
            std::cout << Vox2Idx(closest_obstacle_[next_[prev_[vox]]]) << ' ' << Vox2Idx(closest_obstacle_[vox])
                      << std::endl;
            std::cout << Vox2Idx(closest_obstacle_[prev_[vox]]) << ' ' << Vox2Idx(closest_obstacle_[next_[vox]])
                      << std::endl;

            return false;
          }
          if (prev_[vox] == undefined_ && distance_buffer_[vox] >= 0
              && head_[Vox2Idx(closest_obstacle_[vox])] != vox)
            return false;
        }
#endif
  return true;
}

// only for test, check between Ground Truth calculated by k-d tree
bool fiesta::ESDFMap::CheckWithGroundTruth() {
#ifdef HASH_TABLE
  Eigen::Vector3i ma = Eigen::Vector3i(0, 0, 0), mi = Eigen::Vector3i(0, 0, 0);
  //        ESDFMap *esdf_map_ = new ESDFMap(Eigen::Vector3d(0, 0, 0), resolution, 10000000);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_->width = 0;
  cloud_->height = 1;
  for (int ii = 1; ii < count; ii++)
    if (Exist(ii))
      cloud_->width++;
  cloud_->points.resize(cloud_->width * cloud_->height);
  int tot_ = 0;
  for (int ii = 1; ii < count; ii++)
    if (Exist(ii)) {
      cloud_->points[tot_].x = vox_buffer_[ii](0);
      cloud_->points[tot_].y = vox_buffer_[ii](1);
      cloud_->points[tot_].z = vox_buffer_[ii](2);
      tot_++;
      if (vox_buffer_[ii](0) > ma(0)) ma(0) = vox_buffer_[ii](0);
      if (vox_buffer_[ii](1) > ma(1)) ma(1) = vox_buffer_[ii](1);
      if (vox_buffer_[ii](2) > ma(2)) ma(2) = vox_buffer_[ii](2);
      if (vox_buffer_[ii](0) < mi(0)) mi(0) = vox_buffer_[ii](0);
      if (vox_buffer_[ii](1) < mi(1)) mi(1) = vox_buffer_[ii](1);
      if (vox_buffer_[ii](2) < mi(2)) mi(2) = vox_buffer_[ii](2);

    }
  std::cout << ma(0) << ' ' << ma(1) << ' ' << ma(2) << ' ' << mi(0) << ' ' << mi(1) << ' ' << mi(2) << std::endl;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloud_);
  std::vector<int> pointIdxNKNSearch(1);
  std::vector<float> pointNKNSquaredDistance(1);
  int cnt1 = 0, cnt2 = 0, cnt3 = 0;
  double ems1 = 0, ems2 = 0, max1 = 0, max2 = 0;
  int a[32];
  std::fill(a, a + 32, 0);
  for (int ii = 1; ii < count; ii++) {
    if (distance_buffer_[ii] >= 0 && distance_buffer_[ii] < infinity_) {
      kdtree.nearestKSearch(pcl::PointXYZ(vox_buffer_[ii](0), vox_buffer_[ii](1), vox_buffer_[ii](2)), 1,
                            pointIdxNKNSearch, pointNKNSquaredDistance);
      double tmp = sqrt(pointNKNSquaredDistance[0]) * resolution_;
      //                if (fabs(distance_buffer_[ii] - tmp) > 1e-3) {
      //                    std::cout << vox_buffer_[ii](0) << '\t' << vox_buffer_[ii](1) << '\t' << vox_buffer_[ii](2) << '\n';
      //                    std::cout << cloud_->points[pointIdxNKNSearch[0]].x << '\t'
      //                              << cloud_->points[pointIdxNKNSearch[0]].y << '\t'
      //                              << cloud_->points[pointIdxNKNSearch[0]].z << '\n';
      //
      //                    std::cout << distance_buffer_[ii] << '\t' << tmp << '\n';
      //
      //                    return false;
      double error = distance_buffer_[ii] - tmp;
      if (error > 1e-3) {
        cnt1++;
        a[(int) (error / 0.1)]++;
      }
      if (error < -1e-3) {
        cnt3++;
      }
      ems1 += distance_buffer_[ii] - tmp;
      ems2 += (distance_buffer_[ii] - tmp) * (distance_buffer_[ii] - tmp);
      cnt2++;
      max1 = std::max(distance_buffer_[ii] - tmp, max1);
      //                }
    }
  }

  std::cout << count << std::endl;
  std::cout << ems1 << " / " << cnt1 << " = " << ems1 / cnt1 << std::endl;
  std::cout << ems1 << " / " << cnt2 << " = " << ems1 / cnt2 << std::endl;
  std::cout << ems2 << " / " << cnt1 << " = " << ems2 / cnt1 << std::endl;
  std::cout << ems2 << " / " << cnt2 << " = " << ems2 / cnt2 << std::endl;
  std::cout << "max = " << max1 << "\tcnt3 = " << cnt3 << std::endl;
  for (int i = 0; i < 32; i++) {
    std::cout << " [ " << i * 0.1 << ", " << i * 0.1 + 0.1 << " ]\t" << a[i] << std::endl;
  }

#else
  //        ESDFMap *esdf_map_ = new ESDFMap(Eigen::Vector3d(0, 0, 0), resolution, 10000000);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = 0;
    cloud->height = 1;
    for (int x = 0; x < grid_size_(0); ++x)
      for (int y = 0; y < grid_size_(1); ++y)
        for (int z = 0; z < grid_size_(2); ++z)
          if (Exist(Vox2Idx(Eigen::Vector3i(x, y, z))))
            cloud->width++;
    cloud->points.resize(cloud->width * cloud->height);
    int tot = 0;
    for (int x = 0; x < grid_size_(0); ++x)
      for (int y = 0; y < grid_size_(1); ++y)
        for (int z = 0; z < grid_size_(2); ++z)
          if (Exist(Vox2Idx(Eigen::Vector3i(x, y, z)))) {
            cloud->points[tot].x = x;
            cloud->points[tot].y = y;
            cloud->points[tot].z = z;
            tot++;
          }
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);
    int cnt1 = 0, cnt2 = 0, cnt3 = 0;
    double ems1 = 0, ems2 = 0, max1 = 0, max2 = 0;
    int a[32];
    std::fill(a, a + 32, 0);
    for (int x = 0; x < grid_size_(0); ++x)
      for (int y = 0; y < grid_size_(1); ++y)
        for (int z = 0; z < grid_size_(2); ++z) {
          int ii = Vox2Idx(Eigen::Vector3i(x, y, z));
          if (distance_buffer_[ii] >= 0 && distance_buffer_[ii] < infinity_) {
            kdtree.nearestKSearch(pcl::PointXYZ(x, y, z), 1,
                                  pointIdxNKNSearch, pointNKNSquaredDistance);
            double tmp = sqrt(pointNKNSquaredDistance[0]) * resolution_;
//                if (fabs(distance_buffer_[ii] - tmp) > 1e-3) {
//                    std::cout << vox_buffer_[ii](0) << '\t' << vox_buffer_[ii](1) << '\t' << vox_buffer_[ii](2) << '\n';
//                    std::cout << cloud_->points[pointIdxNKNSearch[0]].x << '\t'
//                              << cloud_->points[pointIdxNKNSearch[0]].y << '\t'
//                              << cloud_->points[pointIdxNKNSearch[0]].z << '\n';
//
//                    std::cout << distance_buffer_[ii] << '\t' << tmp << '\n';
//
//                    return false;
            double error = distance_buffer_[ii] - tmp;
            if (error > 1e-3) {
              cnt1++;
              a[(int) (error / 0.1)]++;
            }
            if (error < -1e-3) {
              cnt3++;
            }
            ems1 += distance_buffer_[ii] - tmp;
            ems2 += (distance_buffer_[ii] - tmp) * (distance_buffer_[ii] - tmp);
            cnt2++;
            max1 = std::max(distance_buffer_[ii] - tmp, max1);
//                }
          }
        }

    std::cout << grid_total_size_ << std::endl;
    std::cout << ems1 << " / " << cnt1 << " = " << ems1 / cnt1 << std::endl;
    std::cout << ems1 << " / " << cnt2 << " = " << ems1 / cnt2 << std::endl;
    std::cout << ems2 << " / " << cnt1 << " = " << ems2 / cnt1 << std::endl;
    std::cout << ems2 << " / " << cnt2 << " = " << ems2 / cnt2 << std::endl;
    std::cout << "max = " << max1 << "\tcnt3 = " << cnt3 << std::endl;
    for (int i = 0; i < 32; i++) {
      std::cout << " [ " << i * 0.1 << ", " << i * 0.1 + 0.1 << " ]\t" << a[i] << std::endl;
    }

#endif
  return true;
}

// endregion

#endif