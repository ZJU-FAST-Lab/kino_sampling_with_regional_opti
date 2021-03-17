#ifndef _POS_CHECKER_
#define _POS_CHECKER_

#include "occ_grid/occ_map.h"
#include "occ_grid/raycast.h"
#include "poly_traj_utils/traj_utils.hpp"
#include <ros/ros.h>
#include <Eigen/Eigen>

#define GLOBAL_TEST

using Eigen::Vector3d;
using Eigen::Vector3i;

namespace tgk_planner
{

class PosChecker
{
private:
  OccMap::Ptr occ_map_;
  double hrz_safe_radius_, vtc_safe_radius_;
  double copter_diag_len_;
  double resolution_;
  double dt_;
  bool inflate_;
  std::vector<Vector3d> line_grids_;

  void getlineGrids(const Vector3d &s_p, const Vector3d &e_p, vector<Vector3d> &grids);

  bool checkState(const Vector3d &pos, const Vector3d &vel, const Vector3d &acc);
  bool checkState(const Vector3d &pos, const Vector3d &vel, const Vector3d &acc, Vector3d &collision_pt);

  inline bool curvatureValid(const Vector3d &vel, const Vector3d &acc)
  {
    double tmp = vel.norm() * vel.norm() * vel.norm();
    double k = (vel.cross(acc)).norm() / tmp;
    if (k >= 8)
      return false;
    else
      return true;
  };

public:
  PosChecker(){};

  ~PosChecker(){};

  void init(const ros::NodeHandle &nh)
  {
    nh.param("pos_checker/hrz_safe_radius", hrz_safe_radius_, 0.0);
    nh.param("pos_checker/vtc_safe_radius", vtc_safe_radius_, 0.0);
    nh.param("pos_checker/copter_diag_len", copter_diag_len_, 0.0);
    nh.param("pos_checker/dt", dt_, 0.0); //dt_ should be less than resolution/Vmax
    nh.param("pos_checker/inflate", inflate_, false);
    ROS_WARN_STREAM("[pos_checker] param: hrz_safe_radius: " << hrz_safe_radius_);
    ROS_WARN_STREAM("[pos_checker] param: vtc_safe_radius: " << vtc_safe_radius_);
    ROS_WARN_STREAM("[pos_checker] param: copter_diag_len: " << copter_diag_len_);
    ROS_WARN_STREAM("[pos_checker] param: dt: " << dt_);
    ROS_WARN_STREAM("[pos_checker] param: inflate: " << inflate_);
    line_grids_.resize(5);
  };

  void setMap(const OccMap::Ptr &occ_map)
  {
    occ_map_ = occ_map;
    resolution_ = occ_map_->getResolution();
  };

  int getVoxelState(const Vector3d& pos)
  {
    return occ_map_->getVoxelState(pos);
  };

  ros::Time getLocalTime()
  {
    return occ_map_->getLocalTime();
  };

  double getResolution()
  {
    return resolution_;
  };

  bool validatePosSurround(const Vector3d &pos);
  bool validatePosSurround(const Vector3i &id);
  bool validateASample(const StatePVA &sample);
  bool validatePt(const Vector3d &pos);

  void getCheckPos(const Vector3d &pos, const Vector3d &vel,
                   const Vector3d &acc, vector<Vector3d> &grids,
                   double hor_radius, double ver_radius);

  bool checkPolySeg(const Piece& seg);

  bool checkPolySeg(const Piece& seg, double t_s, double t_e);

  bool checkPolySeg(const Piece &seg, vector<pair<Vector3d, Vector3d>> &traversal_lines);
  bool checkPolySeg(const Piece &seg, pair<Vector3d, Vector3d> &collide_pts, pair<double, double> &t_s_e, bool &need_region_opt);

  bool checkPolyTraj(const Trajectory &traj, double t_s, double t_e, Vector3d &collide_pos, double &remain_safe_time);
  bool checkPolyTraj(const Trajectory &traj, vector<pair<Vector3d, Vector3d>> &traversal_lines);
  bool checkPolyTraj(const Trajectory &traj);
  bool checkPolyTraj(const Trajectory &traj, vector<pair<Vector3d, Vector3d>> &traversal_lines, vector<pair<double, double>> &ts_s_e);

  bool checkLine(const Vector3d &s_p, const Vector3d &e_p, Eigen::Vector3d &collide_pt, bool inflate);

  bool getPolyTrajCollision(const Trajectory &traj, std::vector<int> &segs, std::vector<Eigen::Vector3d> &collisions, 
                            std::vector<double> &t_s, std::vector<double> &t_e);
  bool getPolyTrajAttractPts(const Trajectory &front_traj, const Trajectory &traj, std::vector<pair<int, int>> &seg_num_obs_size, 
                            std::vector<Eigen::Vector3d> &att_pts, 
                            std::vector<double> &t_s, std::vector<double> &t_e);
  bool getRegionalAttractPts(const Trajectory &front_traj, const Trajectory &traj, 
                            std::vector<pair<int, int>> &seg_num_obs_size, 
                            std::vector<Eigen::Vector3d> &att_pts, 
                            std::vector<double> &t_s, std::vector<double> &t_e);
  void getRegionalAttractPts(const Trajectory &input_traj, const vector<Eigen::Vector3d> &free_path, const pair<double, double> &t_s_e,
                            std::vector<pair<int, int>> &seg_num_obs_size, 
                            std::vector<Eigen::Vector3d> &att_pts, 
                            std::vector<double> &t_s, std::vector<double> &t_e);
  // squared distance
  float nearestObs(const Vector3d &pos, Vector3d &obs)
  {
    float obs_x, obs_y, obs_z;
    float range = occ_map_->nearestObs(pos[0], pos[1], pos[2], obs_x, obs_y, obs_z);
    return range;
  };

  void posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id);
  Eigen::Vector3i posToIndex(const Eigen::Vector3d &pos);
  void indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos);
  Eigen::Vector3i getOccMapSize();

  typedef shared_ptr<PosChecker> Ptr;


public:
  void getSurroundDistance(Eigen::Vector3d pts[2][2][2], double dists[2][2][2]) {
    for (int x = 0; x < 2; x++) {
      for (int y = 0; y < 2; y++) {
        for (int z = 0; z < 2; z++) {
          dists[x][y][z] = occ_map_->getDistance(pts[x][y][z]);
        }
      }
    }
  }

  /* for bench mark */
  void interpolateTrilinear(double values[2][2][2],
                            const Eigen::Vector3d& diff,
                            double& value,
                            Eigen::Vector3d& grad) 
  {
    // trilinear interpolation
    double v00 = (1 - diff(0)) * values[0][0][0] + diff(0) * values[1][0][0];
    double v01 = (1 - diff(0)) * values[0][0][1] + diff(0) * values[1][0][1];
    double v10 = (1 - diff(0)) * values[0][1][0] + diff(0) * values[1][1][0];
    double v11 = (1 - diff(0)) * values[0][1][1] + diff(0) * values[1][1][1];
    double v0 = (1 - diff(1)) * v00 + diff(1) * v10;
    double v1 = (1 - diff(1)) * v01 + diff(1) * v11;

    value = (1 - diff(2)) * v0 + diff(2) * v1;

    grad[2] = (v1 - v0) / resolution_;
    grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) / resolution_;
    grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
    grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
    grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
    grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);
    grad[0] /= resolution_;
  }

  void evaluateEDTWithGrad(const Eigen::Vector3d& pos,
                          double time, double& dist,
                          Eigen::Vector3d& grad) 
  {
    Eigen::Vector3d diff;
    Eigen::Vector3d sur_pts[2][2][2];
    occ_map_->getSurroundPts(pos, sur_pts, diff);

    double dists[2][2][2];
    getSurroundDistance(sur_pts, dists);

    interpolateTrilinear(dists, diff, dist, grad);
  }
  /* for bench mark */
};

inline void PosChecker::posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id)
{
  occ_map_->posToIndex(pos, id);
}

inline Eigen::Vector3i PosChecker::posToIndex(const Eigen::Vector3d &pos)
{
  return occ_map_->posToIndex(pos);
}

inline void PosChecker::indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos)
{
  occ_map_->indexToPos(id, pos);
}

inline Eigen::Vector3i PosChecker::getOccMapSize()
{
  return occ_map_->getMapSize();
}

#ifndef GLOBAL_TEST
inline bool PosChecker::validatePosSurround(const Vector3d &pos)
{
  // int n(0);
  // ros::Time t1 = ros::Time::now();

  int x_size = ceil(copter_diag_len_ / 2.0 / resolution_);
  int y_size = ceil(copter_diag_len_ / 2.0 / resolution_);
  int z_size = ceil(vtc_safe_radius_ / resolution_);
  Vector3d grid(pos);
  for (int i = -x_size; i <= x_size; i += x_size)
    for (int j = -y_size; j <= y_size; j += y_size)
      for (int k = -z_size; k <= z_size; k += z_size)
      {
        // n++;
        grid = pos + Vector3d(i, j, k) * resolution_;
        if (occ_map_->getVoxelState(grid) != 0)
        {
          return false;
        }
      }
  // ros::Time t2 = ros::Time::now();
  // ROS_WARN_STREAM("validate "<< n << "grids used: " << (t2-t1).toSec() * 1e6 << " us");
  return true;
}

//if state pos valid, return true
inline bool PosChecker::checkState(const Vector3d &pos, const Vector3d &vel, const Vector3d &acc)
{
  if (inflate_)
  {
    getCheckPos(pos, vel, acc, line_grids_, hrz_safe_radius_, vtc_safe_radius_);
    for (const Vector3d &grid : line_grids_)
    {
      if (occ_map_->getVoxelState(grid) != 0)
      {
        //cout << "collision: " << grid.transpose() << endl;
        return false;
      }
    }
    return true;
  }
  else
  {
    if (occ_map_->getVoxelState(pos) != 0)
    {
      // cout << "collision: "<< pos.transpose() << endl;
      return false;
    }
    return true;
  }
};

inline bool PosChecker::checkState(const Vector3d &pos, const Vector3d &vel, const Vector3d &acc, Vector3d &collision_pt)
{
  if (inflate_)
  {
    getCheckPos(pos, vel, acc, line_grids_, hrz_safe_radius_, vtc_safe_radius_);
    for (const Vector3d &grid : line_grids_)
    {
      if (occ_map_->getVoxelState(grid) != 0)
      {
        //cout << "collision: " << grid.transpose() << endl;
        collision_pt = grid;
        return false;
      }
    }
    return true;
  }
  else
  {
    if (occ_map_->getVoxelState(pos) != 0)
    {
      // cout << "collision: "<< pos.transpose() << endl;
      collision_pt = pos;
      return false;
    }
    return true;
  }
};

inline bool PosChecker::checkLine(const Vector3d &s_p, const Vector3d &e_p, Eigen::Vector3d &collide_pt, bool inflate)
{
  RayCaster raycaster;
  Eigen::Vector3d ray_pt;
  Eigen::Vector3d start = s_p / resolution_, end = e_p / resolution_;
  bool need_ray = raycaster.setInput(start, end);
  Vector3d vel = e_p - s_p, acc;
  if (need_ray)
  {
    Eigen::Vector2d dir_hrz = (e_p - s_p).head(2).normalized() * hrz_safe_radius_;
    Eigen::Matrix2d r_m;
    r_m << 0, 1,
          -1, 0;
    dir_hrz = r_m * dir_hrz;

    while (raycaster.step(ray_pt))
    {
      Eigen::Vector3d tmp = (ray_pt)*resolution_;
      tmp[0] += resolution_ / 2.0;
      tmp[1] += resolution_ / 2.0;
      tmp[2] += resolution_ / 2.0;
      if (occ_map_->getVoxelState(tmp) != 0)
      {
        collide_pt = tmp;
        return false;
      }
      if (inflate)
      {
        Eigen::Vector3d shift_hrz(tmp);
        shift_hrz.head(2) += dir_hrz / 2.0;
        if (occ_map_->getVoxelState(shift_hrz) != 0)
        {
          collide_pt = tmp;
          return false;
        }
        shift_hrz.head(2) -= dir_hrz;
        if (occ_map_->getVoxelState(shift_hrz) != 0)
        {
          collide_pt = tmp;
          return false;
        }
        Eigen::Vector3d shift_vtc(tmp);
        shift_vtc[2] += vtc_safe_radius_ / 2.0;
        if (occ_map_->getVoxelState(shift_vtc) != 0)
        {
          collide_pt = tmp;
          return false;
        }
        shift_vtc[2] -= vtc_safe_radius_;
        if (occ_map_->getVoxelState(shift_vtc) != 0)
        {
          collide_pt = tmp;
          return false;
        }
      }
    }
    return true;
  }
  else
  {
    return validatePosSurround(s_p);
  }
}
#endif

#ifdef GLOBAL_TEST
inline bool PosChecker::validatePosSurround(const Vector3d &pos) 
{
  if (occ_map_->getDistance(pos) < copter_diag_len_) return false; 
  return true;
}

inline bool PosChecker::validatePosSurround(const Vector3i &id) 
{
  if (occ_map_->getDistance(id) < copter_diag_len_) return false; 
  return true;
}

inline bool PosChecker::checkState(const Vector3d &pos, const Vector3d &vel, const Vector3d &acc)
{
  if (occ_map_->getDistance(pos) < copter_diag_len_) return false;
  return true;
}

inline bool PosChecker::checkState(const Vector3d &pos, const Vector3d &vel, const Vector3d &acc, Vector3d &collision_pt)
{
  if (occ_map_->getDistance(pos) < copter_diag_len_) {collision_pt = pos; return false;}
  return true;
}

//actually it won't be inlined
// inline bool PosChecker::checkLine(const Vector3d &s_p, const Vector3d &e_p, Eigen::Vector3d &collide_pt, bool inflate)
// {
//   RayCaster raycaster;
//   Eigen::Vector3d ray_pt;
//   Eigen::Vector3d start = s_p / resolution_, end = e_p / resolution_;
//   bool need_ray = raycaster.setInput(start, end);
//   Vector3d vel = e_p - s_p, acc;
//   if (need_ray)
//   {
//     while (raycaster.step(ray_pt))
//     {
//       Eigen::Vector3d tmp = (ray_pt)*resolution_;
//       tmp[0] += resolution_ / 2.0;
//       tmp[1] += resolution_ / 2.0;
//       tmp[2] += resolution_ / 2.0;
//       if (occ_map_->getDistance(tmp) < copter_diag_len_)
//       {
//         collide_pt = tmp;
//         return false;
//       }
//     }
//     return true;
//   }
//   else
//   {
//     return validatePosSurround(s_p);
//   }
// }
inline bool PosChecker::checkLine(const Vector3d &s_p, const Vector3d &e_p, Eigen::Vector3d &collide_pt, bool inflate)
{
  Vector3d dir = e_p - s_p;
  double len = dir.norm();
  Vector3d dir_norm = dir / len;
  Vector3d step_vec = dir_norm * resolution_ * 1.5;
  int check_n = floor(len / resolution_ / 1.5);
  Vector3d check_pt(s_p);
  for (int i = 1; i < check_n; ++i)
  {
    check_pt += step_vec;
    if (!validatePosSurround(check_pt))
    {
      collide_pt = check_pt;
      return false;
    }
  }
  return true;
}
#endif

} // namespace tgk_planner

#endif