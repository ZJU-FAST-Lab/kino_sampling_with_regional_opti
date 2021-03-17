#include "occ_grid/pos_checker.h"
#include <chrono>

namespace tgk_planner
{
bool PosChecker::validateASample(const StatePVA &sample)
{
  /* test time usage */
  // std::mt19937_64 gen_;     
  // std::random_device rd;
  // gen_ = std::mt19937_64(rd());
  // std::uniform_real_distribution<double> pos_x_, pos_y_, pos_z_;
  // pos_x_ = std::uniform_real_distribution<double>(-20, 20);
  // pos_y_ = std::uniform_real_distribution<double>(-20, 20);
  // pos_z_ = std::uniform_real_distribution<double>(0, 2);
  // float obs_x, obs_y, obs_z;
  // auto t1 = std::chrono::high_resolution_clock::now();
  // Eigen::Vector3i idx;
  // for (int i=0; i<1000000; ++i)
  // {
  //   Eigen::Vector3d p(pos_x_(gen_), pos_y_(gen_), pos_z_(gen_));
  //   occ_map_->posToIndex(p, idx);
  // }
  // auto t2 = std::chrono::high_resolution_clock::now();
  // std::chrono::duration<double> diff1 = t2 - t1;
  // std::cout << "posToIndex: " << diff1.count() << " us\n";

  // ros::Time t1 = ros::Time::now();
  // int n(0);
  Vector3d vel(sample.segment(3, 3));
  double vel_mag = vel.norm();
  double brake_dis = vel_mag / 2;
  Vector3d delta_pos = vel / vel_mag * resolution_ * 2.0;
  int n_check = ceil(brake_dis / resolution_ / 2.0);

  Vector3d pos_fwd(sample.head(3)), pos_bwd(sample.head(3));
  for (int i = 1; i <= n_check; ++i)
  {
    // n += 2;
    pos_fwd += delta_pos;
    if (!validatePosSurround(pos_fwd))
    {
      return false;
    }
    pos_bwd -= delta_pos;
    if (!validatePosSurround(pos_bwd))
    {
      return false;
    }
  }
  // ros::Time t2 = ros::Time::now();
  // ROS_WARN_STREAM("validate a sample, " <<n << " pos, "<< (t2-t1).toSec() * 1e6 << " us");
  return true;
}

bool PosChecker::validatePt(const Vector3d &pos)
{
  return (occ_map_->getVoxelState(pos) == 0);
}

inline void PosChecker::getCheckPos(const Vector3d &pos, const Vector3d &vel,
                             const Vector3d &acc, vector<Vector3d> &grids,
                             double hor_radius, double ver_radius)
{
  int n = 0;
  Eigen::Vector3d cw_edge_pos, ccw_edge_pos;
  Eigen::Vector2d vel_hor, cw_radius_vec;
  cw_edge_pos[2] = pos[2];
  ccw_edge_pos[2] = pos[2];
  vel_hor[0] = vel[0];
  vel_hor[1] = vel[1];
  if (vel_hor[0] < 1e-4 && vel_hor[1] < 1e-4)
  {
    vel_hor[0] = 1;
    vel_hor[1] = 1;
  }
  //rotate 90 degree in X-Y plane
  cw_radius_vec[0] = vel_hor[1];
  cw_radius_vec[1] = - vel_hor[0];
  cw_radius_vec = cw_radius_vec.normalized() * hor_radius;
  cw_edge_pos.head(2) = pos.head(2) + cw_radius_vec;
  ccw_edge_pos.head(2) = pos.head(2) - cw_radius_vec;
  //add horizontal vox;
  // getlineGrids(cw_edge_pos, ccw_edge_pos, grids);
  // grids.emplace_back(cw_edge_pos[0], cw_edge_pos[1], cw_edge_pos[2]);
  // grids.emplace_back(ccw_edge_pos[0], ccw_edge_pos[1], ccw_edge_pos[2]);
  grids[n][0] = cw_edge_pos[0];
  grids[n][1] = cw_edge_pos[1];
  grids[n][2] = cw_edge_pos[2];
  n++;
  grids[n][0] = ccw_edge_pos[0];
  grids[n][1] = ccw_edge_pos[1];
  grids[n][2] = ccw_edge_pos[2];


  Eigen::Vector3d vertical_up(pos), vertical_down(pos);
  vertical_up(2) += ver_radius;
  vertical_down(2) -= ver_radius;
  //add veltical vox;
  // getlineGrids(vertical_up, vertical_down, grids);
  // grids.emplace_back(vertical_up[0], vertical_up[1], vertical_up[2]);
  // grids.emplace_back(vertical_down[0], vertical_down[1], vertical_down[2]);
  n++;
  grids[n][0] = vertical_up[0];
  grids[n][1] = vertical_up[1];
  grids[n][2] = vertical_up[2];
  n++;
  grids[n][0] = vertical_down[0];
  grids[n][1] = vertical_down[1];
  grids[n][2] = vertical_down[2];

  // grids.emplace_back(pos[0], pos[1], pos[2]);
  n++;
  grids[n][0] = pos[0];
  grids[n][1] = pos[1];
  grids[n][2] = pos[2];
}

void PosChecker::getlineGrids(const Vector3d &s_p, const Vector3d &e_p, vector<Vector3d> &grids)
{
  RayCaster raycaster;
  Eigen::Vector3d ray_pt;
  Eigen::Vector3d start = s_p / resolution_, end = e_p / resolution_;
  bool need_ray = raycaster.setInput(start, end);
  if (need_ray)
  {
    while (raycaster.step(ray_pt))
    {
      Eigen::Vector3d tmp = (ray_pt)*resolution_;
      tmp[0] += resolution_ / 2.0;
      tmp[1] += resolution_ / 2.0;
      tmp[2] += resolution_ / 2.0;
      grids.emplace_back(tmp[0], tmp[1], tmp[2]);
    }
  }

  //check end
  Eigen::Vector3d end_idx;
  end_idx[0] = std::floor(end.x());
  end_idx[1] = std::floor(end.y());
  end_idx[2] = std::floor(end.z());

  ray_pt[0] = (double)end_idx[0];
  ray_pt[1] = (double)end_idx[1];
  ray_pt[2] = (double)end_idx[2];
  Eigen::Vector3d tmp = (ray_pt)*resolution_;
  tmp[0] += resolution_ / 2.0;
  tmp[1] += resolution_ / 2.0;
  tmp[2] += resolution_ / 2.0;
  grids.emplace_back(tmp[0], tmp[1], tmp[2]);
}

bool PosChecker::checkPolySeg(const Piece &seg)
{
  double tau = seg.getDuration();
  for (double t = 0.0; t <= tau; t += dt_)
  {
    Vector3d pos, vel, acc;
    pos = seg.getPos(t);
    vel = seg.getVel(t);
    acc = seg.getAcc(t);
    if (!checkState(pos, vel, acc))
      return false;
      
    // check curvature, ugly 
    if (t > 0.3 && t < tau - 0.3)
    {
      double tmp = vel.norm() * vel.norm() * vel.norm();
      double k = (vel.cross(acc)).norm() / tmp;
      if (k >= 8)
        return false;
    }
  }
  return true;
};

bool PosChecker::checkPolySeg(const Piece &seg, double t_s, double t_e)
{
  double tau = seg.getDuration();
  if (t_s < -FLT_EPSILON || t_e > tau + FLT_EPSILON)
  {
    ROS_WARN_STREAM("Check time violates duration, tau: " << tau << ", t_s: " << t_s << ", t_e: " << t_e);
    return false;
  }
  for (double t = t_s; t <= t_e; t += dt_)
  {
    Vector3d pos, vel, acc;
    pos = seg.getPos(t);
    vel = seg.getVel(t);
    acc = seg.getAcc(t);
    if (!checkState(pos, vel, acc))
      return false;
    // if (t > 0.3 && t < t_e - 0.3)
    // {
    //   if (!curvatureValid(vel, acc))
    //     return false;
    // }
  }
  return true;
};

bool PosChecker::checkPolySeg(const Piece &seg, vector<pair<Vector3d, Vector3d>> &traversal_lines)
{
  double tau = seg.getDuration();
  pair<Vector3d, Vector3d> line;
  bool is_valid(true);
  bool result(true);
  bool zigzag(false);
  Vector3d last_pos = seg.getPos(0.0);

  for (double t = 0.0; t <= tau; t += dt_)
  {
    Eigen::Vector3d pos, vel, acc;
    pos = seg.getPos(t);
    vel = seg.getVel(t);
    acc = seg.getAcc(t);
    if (!zigzag)
    {
      if (t > 0.3 && t < tau - 0.3)
      {
        double tmp = vel.norm() * vel.norm() * vel.norm();
        double k = (vel.cross(acc)).norm() / tmp;
        if (k >= 6)
        {
          line.first = pos;
          line.second = Eigen::Vector3d(0.0, 0.0, -1.0);
          traversal_lines.push_back(line);
          zigzag = true;
        }
      }
    }

    if (is_valid && !checkState(pos, vel, acc))
    {
      result = false;
      is_valid = false;
      line.first = last_pos;
    }
    else if (!is_valid && checkState(pos, vel, acc))
    {
      is_valid = true;
      line.second = pos;
      traversal_lines.push_back(line);
    }
    last_pos = pos;
  }
  return result;
};

bool PosChecker::checkPolySeg(const Piece &seg, pair<Vector3d, Vector3d> &collide_pts, pair<double, double> &t_s_e, bool &need_region_opt)
{
  need_region_opt = false;
  double tau = seg.getDuration();
  pair<Vector3d, Vector3d> line;
  bool is_valid(true);
  bool result(true);
  bool zigzag(false);
  bool first_collision(true);
  Vector3d last_pos = seg.getPos(0.0);

  for (double t = 0.0; t <= tau; t += dt_)
  {
    Eigen::Vector3d pos, vel, acc;
    pos = seg.getPos(t);
    vel = seg.getVel(t);
    acc = seg.getAcc(t);
    if (!zigzag)
    {
      if (t > 0.3 && t < tau - 0.3)
      {
        double tmp = vel.norm() * vel.norm() * vel.norm();
        double k = (vel.cross(acc)).norm() / tmp;
        if (k >= 6)
        {
          zigzag = true;
          result = false;
        }
      }
    }

    if (is_valid && !checkState(pos, vel, acc))
    {
      if (!occ_map_->isInMap(pos))
      {
        need_region_opt = false;
        return false;
      }
      if (!first_collision)
      {
        need_region_opt = false;
        return result;
      }
      result = false;
      is_valid = false;
      collide_pts.first = last_pos;
      t_s_e.first = t;
    }
    else if (!is_valid && checkState(pos, vel, acc))
    {
      is_valid = true;
      collide_pts.second = pos;
      need_region_opt = true;
      first_collision = false;
      t_s_e.second = t;
    }
    last_pos = pos;
  }
  return result;
};

bool PosChecker::checkPolyTraj(const Trajectory &traj, double t_s, double t_e, Vector3d &collide_pos, double &remain_safe_time)
{
  double tau = traj.getTotalDuration();
  if (t_s < -FLT_EPSILON || t_e > tau + FLT_EPSILON)
  {
    ROS_WARN_STREAM("Check time violates duration, tau: " << tau << ", t_s: " << t_s << ", t_e: " << t_e);
    return false;
  }
  for (double t = t_s; t <= t_e; t += dt_)
  {
    Vector3d pos, vel, acc;
    pos = traj.getPos(t);
    vel = traj.getVel(t);
    if (!checkState(pos, vel, acc))
    {
      remain_safe_time = t - t_s;
      collide_pos = pos;
      return false;
    }
  }
  remain_safe_time = t_e - t_s;
  return true;
};

bool PosChecker::checkPolyTraj(const Trajectory &traj)
{
  double tau = traj.getTotalDuration();
  for (double t = 0.0; t <= tau; t += dt_)
  {
    Vector3d pos, vel, acc;
    pos = traj.getPos(t);
    vel = traj.getVel(t);
    if (!checkState(pos, vel, acc))
    {
      return false;
    }
  }
  return true;
};

bool PosChecker::checkPolyTraj(const Trajectory &traj, vector<pair<Vector3d, Vector3d>> &traversal_lines)
{
  double tau = traj.getTotalDuration();
  pair<Vector3d, Vector3d> line;
  bool is_valid(true);
  bool result(true);
  Vector3d last_pos = traj.getPos(0.0);

  for (double t = 0.0; t <= tau; t += dt_)
  {
    Eigen::Vector3d pos, vel, acc;
    pos = traj.getPos(t);
    vel = traj.getVel(t);
    acc = traj.getAcc(t);
    if (is_valid && !checkState(pos, vel, acc))
    {
      result = false;
      is_valid = false;
      line.first = last_pos;
    }
    else if (!is_valid && checkState(pos, vel, acc))
    {
      is_valid = true;
      line.second = pos;
      traversal_lines.push_back(line);
    }
    last_pos = pos;
  }
  return result;
};

bool PosChecker::checkPolyTraj(const Trajectory &traj, vector<pair<Vector3d, Vector3d>> &traversal_lines, vector<pair<double, double>> &ts_s_e)
{
  double tau = traj.getTotalDuration();
  pair<Vector3d, Vector3d> line;
  pair<double, double> t_s_e;
  bool is_valid(true);
  bool result(true);
  Vector3d last_pos = traj.getPos(0.0);

  for (double t = 0.0; t <= tau; t += dt_)
  {
    Eigen::Vector3d pos, vel, acc;
    pos = traj.getPos(t);
    vel = traj.getVel(t);
    acc = traj.getAcc(t);
    if (is_valid && !checkState(pos, vel, acc))
    {
      result = false;
      is_valid = false;
      line.first = last_pos;
      t_s_e.first = t;
    }
    else if (!is_valid && checkState(pos, vel, acc))
    {
      is_valid = true;
      line.second = pos;
      t_s_e.second = t;
      traversal_lines.push_back(line);
      ts_s_e.push_back(t_s_e);
    }
    last_pos = pos;
  }
  return result;
};

// stop after first collision
bool PosChecker::getPolyTrajCollision(const Trajectory &traj, std::vector<int> &segs, std::vector<Eigen::Vector3d> &collisions, 
                                      std::vector<double> &t_s, std::vector<double> &t_e)
{
  bool result(true);
  int n_seg = traj.getPieceNum();
  Vector3d pos, vel, acc;
  for (int i = 0; i < n_seg; ++i)
  {
    double tau = traj[i].getDuration();
    for (double t = 0.0; t < tau; t += dt_)
    {
      pos = traj[i].getPos(t);
      vel = traj[i].getVel(t);
      if (!checkState(pos, vel, acc))
      {
        result = false;
        segs.push_back(i);
        collisions.push_back(pos);
        t_s.push_back(std::max(t - 0.2, 0.0));
        t_e.push_back(std::min(t + 0.2, tau));
        break;
      }
    }
  }
  return result;
}

bool PosChecker::getPolyTrajAttractPts(const Trajectory &front_traj, const Trajectory &traj, std::vector<pair<int, int>> &seg_num_obs_size, 
                                      std::vector<Eigen::Vector3d> &att_pts, 
                                      std::vector<double> &t_s, std::vector<double> &t_e)
{
  bool result(true);
  int n_seg = traj.getPieceNum();
  Vector3d pos, vel, acc, attract_pt, front_traj_pt;
  pair<int, int> snos;
  for (int i = 0; i < n_seg; ++i)
  {
    snos.first = i;
    snos.second = 0;
    double tau = traj[i].getDuration();
    double collide_t_last = 0.0;
    bool first_obs(true);
    for (double t = 0.0; t < tau; t += dt_)
    {
      pos = traj[i].getPos(t);
      vel = traj[i].getVel(t);
      if (!checkState(pos, vel, acc))
      {
        result = false;
        if (first_obs || t - collide_t_last > 0.2)
        {
          first_obs = false;
          collide_t_last = t;
          front_traj_pt = front_traj[i].getPos(t);
          attract_pt = front_traj_pt + (front_traj_pt - pos);
          att_pts.emplace_back(attract_pt[0], attract_pt[1], attract_pt[2]);
          t_s.emplace_back(std::max(t - 0.2, 0.0));
          t_e.emplace_back(std::min(t + 0.2, tau));
          snos.second++;
        }
      }
    }
    if (snos.second > 0)
    {
      seg_num_obs_size.push_back(snos);
    }
  }
  return result;
}

bool PosChecker::getRegionalAttractPts(const Trajectory &front_traj, const Trajectory &traj, std::vector<pair<int, int>> &seg_num_obs_size, 
                                      std::vector<Eigen::Vector3d> &att_pts, 
                                      std::vector<double> &t_s, std::vector<double> &t_e)
{
  seg_num_obs_size.clear();
  att_pts.clear();
  t_s.clear();
  t_e.clear();
  bool result(true);
  int n_seg = traj.getPieceNum();
  Vector3d pos, vel, acc, attract_pt, front_traj_pt;
  pair<int, int> snos;
  for (int i = 0; i < n_seg; ++i)
  {
    snos.first = i;
    snos.second = 0;
    double tau = traj[i].getDuration();
    double collide_t_last = 0.0;
    bool first_obs(true);
    for (double t = 0.0; t < tau; t += dt_)
    {
      pos = traj[i].getPos(t);
      vel = traj[i].getVel(t);
      if (!checkState(pos, vel, acc))
      {
        result = false;
        if (first_obs || t - collide_t_last > 0.2)
        {
          first_obs = false;
          collide_t_last = t;
          front_traj_pt = front_traj[i].getPos(t);
          attract_pt = front_traj_pt + (front_traj_pt - pos);
          att_pts.emplace_back(attract_pt[0], attract_pt[1], attract_pt[2]);
          t_s.emplace_back(std::max(t - 0.2, 0.0));
          t_e.emplace_back(std::min(t + 0.2, tau));
          snos.second++;
        }
      }
    }
    if (snos.second > 0)
    {
      seg_num_obs_size.push_back(snos);
    }
  }
  return result;
}

void PosChecker::getRegionalAttractPts(const Trajectory &input_traj, 
                                      const vector<Eigen::Vector3d> &free_path, 
                                      const pair<double, double> &t_s_e,
                                      std::vector<pair<int, int>> &seg_num_obs_size, 
                                      std::vector<Eigen::Vector3d> &att_pts, 
                                      std::vector<double> &t_s, std::vector<double> &t_e)
{
  double t_in_half_collide = (t_s_e.first + t_s_e.second) / 2.0;
  Vector3d pos_in_half_collide = input_traj.getPos(t_in_half_collide);
  Vector3d vel_in_half_collide = input_traj.getVel(t_in_half_collide);
  //search for attract_pt
  size_t n_free_path = free_path.size() / 2;
  Vector3d free_pt = free_path[n_free_path];
  double min_attract_dis = 0.6;
  Vector3d att_vec = free_pt - pos_in_half_collide;
  double att_vec_len = att_vec.norm();
  Vector3d att_pt = free_pt + att_vec / att_vec_len * max(att_vec_len, min_attract_dis);
  int seg_num = input_traj.getPieceNum();
  double t_in_ts_seg(t_s_e.first);
  double t_in_te_seg(t_s_e.second);
  int ts_seg = input_traj.locatePieceIdx(t_in_ts_seg);
  int te_seg = input_traj.locatePieceIdx(t_in_te_seg);

  if (ts_seg == te_seg)
  {
    seg_num_obs_size.push_back(std::make_pair(ts_seg, 1));
    att_pts.push_back(att_pt);
    t_s.push_back(max(0.0, t_in_ts_seg - 0.2));
    t_e.push_back(min(input_traj[te_seg].getDuration(), t_in_te_seg + 0.2));
    return;
  }

  for (int seg = ts_seg; seg <= te_seg; ++seg)
  {
    seg_num_obs_size.push_back(std::make_pair(seg, 1));
    att_pts.push_back(att_pt);
    if (seg == ts_seg)
    {
      t_s.push_back(max(0.0, t_in_ts_seg - 0.2));
      t_e.push_back(input_traj[seg].getDuration());
    }
    else if (seg == te_seg)
    {
      t_s.push_back(0.0);
      t_e.push_back(min(input_traj[seg].getDuration(), t_in_te_seg + 0.2));
    }
    else
    {
      t_s.push_back(0.0);
      t_e.push_back(input_traj[seg].getDuration());
    }
  }
}





} // namespace tgk_planner
