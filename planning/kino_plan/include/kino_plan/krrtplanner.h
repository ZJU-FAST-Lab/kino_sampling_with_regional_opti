#ifndef _KRRTPLANNER_H_
#define _KRRTPLANNER_H_

#include "node_utils.h"
#include "kdtree.h"
#include "visualization_utils/visualization_utils.h"
#include "occ_grid/pos_checker.h"
#include "poly_traj_utils/traj_utils.hpp"
#include "bvp_solver.h"
#include "bias_sampler.h"
#include "poly_opt/traj_optimizer.h"
#include "r3_plan/a_star_search.h"

#include <vector>
#include <stack>

using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector2i;
using Eigen::Vector3d;
using Eigen::Vector3i;
using Eigen::VectorXd;
using std::list;
using std::pair;
using std::stack;
using std::vector;

namespace tgk_planner
{

class KRRTPlanner
{
public:
  KRRTPlanner();
  KRRTPlanner(const ros::NodeHandle &nh);
  ~KRRTPlanner();

  // api
  void reset();
  void init(const ros::NodeHandle &nh);
  void setPosChecker(const PosChecker::Ptr &checker);
  void setVisualizer(const VisualRviz::Ptr &vis);
  void setRegionalOptimizer(const TrajOptimizer::Ptr &optimizer_)
  {
    optimizer_ptr_ = optimizer_;
  };
  void setSearcher(const std::shared_ptr<AstarPathFinder> &searcher)
  {
    searcher_ = searcher;
  }
  int plan(Vector3d start_pos, Vector3d start_vel, Vector3d start_acc,
           Vector3d end_pos, Vector3d end_vel, Vector3d end_acc,
           double search_time, const Vector3d &normal, const Vector3d &dire, bool need_consistancy, bool use_regional_opt);
  void getTraj(Trajectory &traj)
  {
    traj = traj_;
  };
  void getFirstTraj(Trajectory &traj)
  {
    traj = first_traj_;
  };
  double getFirstTrajTimeUsage()
  {
    return first_traj_use_time_;
  };
  double getFinalTrajTimeUsage()
  {
    return final_traj_use_time_;
  };
  void getConvergenceInfo(vector<Trajectory>& traj_list, vector<double>& solution_cost_list, vector<double>& solution_time_list)
  {
    traj_list = traj_list_;
    solution_time_list = solution_time_list_;
    solution_cost_list = solution_cost_list_;
  };

  // evaluation
  void evaluateTraj(const Trajectory& traj, double &traj_duration, double &traj_length, int &seg_nums, double &acc_integral, double &jerk_integral);

  // bias_sampler
  BiasSampler sampler_;

  enum
  {
    FAILURE = 0, 
    SUCCESS = 1, 
    SUCCESS_CLOSE_GOAL = 2
  };
  typedef shared_ptr<KRRTPlanner> KRRTPlannerPtr;

private:
  int rrtStar(const StatePVA &x_init, const StatePVA &x_final, int n, double search_time, double radius, const bool rewire);
  double dist(const StatePVA &x0, const StatePVA &x1);
  void fillTraj(const RRTNodePtr &goal_leaf, Trajectory& traj);
  void chooseBypass(RRTNodePtr &goal_leaf, const RRTNodePtr &tree_start_node);

  // vis
  bool debug_vis_;
  VisualRviz::Ptr vis_ptr_;
  void sampleWholeTree(const RRTNodePtr &root, vector<StatePVA> *vis_x, vector<Vector3d>& knots);

  RRTNodePtrVector start_tree_; //pre allocated in Constructor
  std::vector<StatePVA> orphans_;
  Trajectory traj_;
  Trajectory first_traj_; //initialized when first path found
  RRTNodePtr start_node_, goal_node_, close_goal_node_;
  int valid_start_tree_node_nums_;
  double final_traj_use_time_, first_traj_use_time_;
  bool test_convergency_;
  vector<Trajectory> traj_list_;
  vector<double> solution_cost_list_;
  vector<double> solution_time_list_;

  // radius for for/backward search
  double getForwardRadius(double tau, double cost);
  double getBackwardRadius(double tau, double cost);
  struct kdres *getForwardNeighbour(const StatePVA &x1, struct kdtree *kd_tree, double tau, double radius_p);
  struct kdres *getBackwardNeighbour(const StatePVA &x1, struct kdtree *kd_tree, double tau, double radius_p);

  // nodehandle params
  double radius_cost_between_two_states_;
  double rho_;
  double v_mag_sample_;
  double vel_limit_, acc_limit_, jerk_limit_;
  bool allow_orphan_, allow_close_goal_, stop_after_first_traj_found_, rewire_, use_regional_opt_;
  double search_time_;
  int tree_node_nums_, orphan_nums_;

  // environment
  PosChecker::Ptr pos_checker_ptr_;
  bool checkSegmentConstraints(const Piece &seg);
  bool getTraversalLines(const Piece &seg, vector<pair<Vector3d, Vector3d>> &traversal_lines);

  // bvp_solver
  BVPSolver::IntegratorBVP bvp_;

  // // bias_sampler
  // BiasSampler sampler_;

  // regional optimizer
  TrajOptimizer::Ptr optimizer_ptr_;
  std::shared_ptr<AstarPathFinder> searcher_;

  // for replan
  Vector3d replan_normal_, replan_dire_;
  bool need_consistancy_;
};

} // namespace tgk_planner

#endif //_KRRTPLANNER_H_
