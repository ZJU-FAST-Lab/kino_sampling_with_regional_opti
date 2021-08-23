#ifndef _POS_CHECKER_
#define _POS_CHECKER_

#include "occ_grid/occ_map.h"
#include "occ_grid/raycast.h"
#include "poly_traj_utils/traj_utils.hpp"
#include <ros/ros.h>
#include <Eigen/Eigen>

using Eigen::Vector3d;
using Eigen::Vector3i;

namespace kino_planner
{

class PosChecker
{
private:
  OccMap::Ptr occ_map_;
  double resolution_;
  double dt_;

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
    nh.param("pos_checker/dt", dt_, 0.0); //dt_ should be less than resolution/Vmax
    ROS_WARN_STREAM("[pos_checker] param: dt: " << dt_);
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

inline bool PosChecker::validatePosSurround(const Vector3d &pos) 
{
  if (occ_map_->isInflateOccupied(pos)) return false;
  return true;
}

inline bool PosChecker::validatePosSurround(const Vector3i &id) 
{
  if (occ_map_->isInflateOccupied(id)) return false;
  return true;
}

inline bool PosChecker::checkState(const Vector3d &pos, const Vector3d &vel, const Vector3d &acc)
{
  if (occ_map_->isInflateOccupied(pos)) return false;
  return true;
}

inline bool PosChecker::checkState(const Vector3d &pos, const Vector3d &vel, const Vector3d &acc, Vector3d &collision_pt)
{
  if (occ_map_->isInflateOccupied(pos)) {collision_pt = pos; return false;}
  return true;
}

} // namespace kino_planner

#endif