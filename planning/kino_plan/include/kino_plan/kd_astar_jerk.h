#ifndef _KD_ASTAR_JERK_H
#define _KD_ASTAR_JERK_H

#include "kino_plan/bvp_solver.h"
#include "kino_plan/kinodynamic_astar.h"
#include "occ_grid/pos_checker.h"
#include <Eigen/Eigen>
#include <boost/functional/hash.hpp>
#include <iostream>
#include <map>
#include <queue>
#include <ros/console.h>
#include <ros/ros.h>
#include <string>
#include <unordered_map>
#include <visualization_msgs/Marker.h>

using std::make_pair;

namespace tgk_planner {

class PathNodeJ {
public:
  /* -------------------- */
  Eigen::Vector3i index;
  Eigen::Matrix<double, 9, 1> state;
  double g_score, f_score;
  Eigen::Vector3d input;
  double duration;
  double time; // dyn
  int time_idx;
  PathNodeJ *parent;
  char node_state;

  /* -------------------- */
  PathNodeJ() {
    parent = NULL;
    node_state = NOT_EXPAND;
  }
  ~PathNodeJ(){};
};
typedef PathNodeJ *PathNodeJPtr;

class NodeComparatorJ {
public:
  bool operator()(PathNodeJPtr node1, PathNodeJPtr node2) {
    return node1->f_score > node2->f_score;
  }
};

class NodeHashTableJ {
private:
  /* data */
  std::unordered_map<Eigen::Vector3i, PathNodeJPtr,
                     matrix_hash<Eigen::Vector3i>>
      data_3d_;
  std::unordered_map<Eigen::Vector4i, PathNodeJPtr,
                     matrix_hash<Eigen::Vector4i>>
      data_4d_;

public:
  NodeHashTableJ(/* args */) {}
  ~NodeHashTableJ() {}
  void insert(Eigen::Vector3i idx, PathNodeJPtr node) {
    data_3d_.insert(make_pair(idx, node));
  }
  void insert(Eigen::Vector3i idx, int time_idx, PathNodeJPtr node) {
    data_4d_.insert(
        make_pair(Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx), node));
  }

  PathNodeJPtr find(Eigen::Vector3i idx) {
    auto iter = data_3d_.find(idx);
    return iter == data_3d_.end() ? NULL : iter->second;
  }
  PathNodeJPtr find(Eigen::Vector3i idx, int time_idx) {
    auto iter =
        data_4d_.find(Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx));
    return iter == data_4d_.end() ? NULL : iter->second;
  }

  void clear() {
    data_3d_.clear();
    data_4d_.clear();
  }
};

class KinodynamicAstarJ {
private:
  /* ---------- main data structure ---------- */
  vector<PathNodeJPtr> path_node_pool_;
  int use_node_num_, iter_num_;
  NodeHashTableJ expanded_nodes_;
  std::priority_queue<PathNodeJPtr, std::vector<PathNodeJPtr>, NodeComparatorJ>
      open_set_;
  std::vector<PathNodeJPtr> path_nodes_;

  /* ---------- record data ---------- */
  Eigen::Vector3d start_vel_, end_vel_, start_acc_;
  Eigen::Matrix<double, 9, 9> phi_; // state transit matrix
  PosChecker::Ptr pos_checker_ptr_;
  bool is_shot_succ_ = false;
  Eigen::MatrixXd coef_shot_;
  double t_shot_;
  bool has_path_ = false;

  /* ---------- parameter ---------- */
  /* search */
  double max_tau_ = 0.25;
  double init_max_tau_ = 0.8;
  double max_vel_ = 3.0;
  double max_acc_ = 3.0;
  double max_jerk_ = 3.0;
  double w_time_ = 10.0;
  double horizon_;
  double lambda_heu_;
  double margin_;
  int allocate_num_;
  int check_num_;
  double tie_breaker_ = 1.0 + 1.0 / 10000;

  double input_res_, input_res_init_, time_res_, time_res_init_;
  /* map */
  double resolution_, inv_resolution_, time_resolution_, inv_time_resolution_;
  double time_origin_;
  double rho_;

  ros::NodeHandle nh_;
  ros::Publisher point_pub_;
  // std::vector<Eigen::Vector3d> visited_pts;
  visualization_msgs::Marker visited_points;

  BVPSolver::IntegratorBVP bvp_;

  /* helper */
  int timeToIndex(double time);
  void retrievePath(PathNodeJPtr end_node);

  /* shot trajectory */

  bool computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2,
                       double time_to_goal);
  double estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2,
                           double &optimal_time);

  /* state propagation */
  void stateTransit(Eigen::Matrix<double, 9, 1> &state0,
                    Eigen::Matrix<double, 9, 1> &state1, Eigen::Vector3d um,
                    double tau);

public:
  KinodynamicAstarJ(){};
  ~KinodynamicAstarJ();

  enum { REACH_HORIZON = 1, REACH_END = 2, NO_PATH = 3 };

  /* main API */
  void setParam(const ros::NodeHandle &nh);
  void init();
  void reset();
  int search(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
             Eigen::Vector3d start_acc, Eigen::Vector3d end_pt,
             Eigen::Vector3d end_vel, Eigen::Vector3d end_acc, bool init,
             bool dynamic = false, double time_start = -1.0);

  void setPosChecker(const PosChecker::Ptr &checker);
  std::vector<Eigen::Vector3d> getKinoTraj(double delta_t);
  void getVisTraj(double delta_t, vector<Eigen::Matrix<double, 9, 1>> &states);
  void getTrajAttributes(double &traj_duration, double &ctrl_cost,
                         double &traj_length, int &seg_nums);
  // Eigen::MatrixXd getSamples(double &ts, int &K);
  std::vector<PathNodeJPtr> getVisitedNodes();
  void getTraj(Trajectory &traj);

  typedef shared_ptr<KinodynamicAstarJ> Ptr;
};

} // namespace fast_planner

#endif