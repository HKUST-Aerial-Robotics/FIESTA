//
// Created by tommy on 5/2/19.
//

#include "parameters.h"
void fiesta::Parameters::SetParameters(const ros::NodeHandle &node) {
  node.param<double>("resolution", resolution_, 0.1);
  node.param<int>("visualize_every_n_updates", visualize_every_n_updates_, 1);
  node.param<double>("min_ray_length", min_ray_length_, 0.5);
  node.param<double>("max_ray_length", max_ray_length_, 5.0);
  double slice_vis_level_tmp, vis_lower_bound_tmp, vis_upper_bound_tmp;

  node.param<double>("slice_vis_max_dist", slice_vis_max_dist_, 2.0);
  node.param<double>("slice_vis_level", slice_vis_level_tmp, 5);
  node.param<double>("vis_lower_bound", vis_lower_bound_tmp, -5);
  node.param<double>("vis_upper_bound", vis_upper_bound_tmp, 10);
  slice_vis_level_ = (int) (slice_vis_level_tmp / resolution_);
  vis_lower_bound_ = (int) (vis_lower_bound_tmp / resolution_);
  vis_upper_bound_ = (int) (vis_upper_bound_tmp / resolution_);

  node.param<double>("center_x", center_x_, 322.477357419);
  node.param<double>("center_y", center_y_, 237.076346481);
  node.param<double>("focal_x", focal_x_, 384.458089392);
  node.param<double>("focal_y", focal_y_, 383.982755697);
  node.param<int>("ray_cast_num_thread", ray_cast_num_thread_, 0);
  double radius_x, radius_y, radius_z;
  node.param<double>("radius_x", radius_x, 3.f);
  node.param<double>("radius_y", radius_y, 3.f);
  node.param<double>("radius_z", radius_z, 1.5f);
  radius_ = Eigen::Vector3d(radius_x, radius_y, radius_z);

  node.param<bool>("global_map", global_map_, true);
  node.param<bool>("global_update", global_update_, true);
  node.param<bool>("global_vis", global_vis_, true);
  if (!global_map_)
    global_vis_ = global_update_ = false;

  node.param<bool>("use_depth_filter", use_depth_filter_, true);
  node.param<double>("depth_filter_tolerance", depth_filter_tolerance_, 0.1f);
  node.param<double>("depth_filter_max_dist", depth_filter_max_dist_, 10.f);
  node.param<double>("depth_filter_min_dist", depth_filter_min_dist_, 0.1f);
  node.param<int>("depth_filter_margin", depth_filter_margin_, 0);

#ifdef HASH_TABLE
  l_cornor_ << -100.f, -100.f, -100.f;
    r_cornor_ << 100.f, 100.f, 100.f;
    node.param<int>("reserved_size", reserved_size_, 1000000);
#else
  double lx, ly, lz;
  double rx, ry, rz;
  node.param<double>("lx", lx, -20.f);
  node.param<double>("ly", ly, -20.f);
  node.param<double>("lz", lz, -5.f);
  node.param<double>("rx", rx, 20.f);
  node.param<double>("ry", ry, 20.f);
  node.param<double>("rz", rz, 5.f);

  l_cornor_ << lx, ly, lz;
  r_cornor_ << rx, ry, rz;
  map_size_ = r_cornor_ - l_cornor_;
#endif

  node.param<double>("update_esdf_every_n_sec", update_esdf_every_n_sec_, 0.1f);
//  T_B_C_ << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
//  T_D_B_ << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;

  // LADY_AND_COW
  T_B_C_ << 1, 0, 0, 0,
           0, 1, 0, 0,
           0, 0, 1, 0,
           0, 0, 0, 1;
  T_D_B_ << 0.971048, -0.120915, 0.206023, 0.00114049,
           0.15701, 0.973037, -0.168959, 0.0450936,
           -0.180038, 0.196415, 0.96385, 0.0430765,
           0.0, 0.0, 0.0, 1.0;

  // EuRoC
  //    T_B_C << 0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
  //            0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
  //            -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
  //            0.0, 0.0, 0.0, 1.0;
  //    T_D_B << 0.33638, -0.01749, 0.94156, 0.06901,
  //            -0.02078, -0.99972, -0.01114, -0.02781,
  //            0.94150, -0.01582, -0.33665, -0.12395,
  //            0.0, 0.0, 0.0, 1.0;
  //    T_D_B = T_D_B.inverse();

#ifdef PROBABILISTIC
  node.param<double>("p_hit", p_hit_, 0.70);
  node.param<double>("p_miss", p_miss_, 0.35);
  node.param<double>("p_min", p_min_, 0.12);
  node.param<double>("p_max", p_max_, 0.97);
  node.param<double>("p_occ", p_occ_, 0.80);
//    esdf_map_->SetParameters(p_hit_, p_miss_, p_min_, p_max_, p_occ_);
#endif
}