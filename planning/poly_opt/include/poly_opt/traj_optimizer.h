#ifndef _TRAJ_OPTIMIZER_H_
#define _TRAJ_OPTIMIZER_H_

#include "occ_grid/pos_checker.h"
#include "visualization_utils/visualization_utils.h"
#include "r3_plan/a_star_search.h"
#include <Eigen/Eigen>
#include <ros/ros.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace tgk_planner 
{
class TrajOptimizer
{
public:
  TrajOptimizer(const ros::NodeHandle &node);
  TrajOptimizer();
  void setPosChecker(const PosChecker::Ptr& checker)
  {
    pos_checker_ptr_ = checker;
  };
  int getIterationTimes()
  {
    return ite_times_;
  };
  void getTraj(Trajectory &traj)
  {
    traj = optimized_traj_;
  };
  bool initialize(const Trajectory &traj, int type)
  {
    int seg_num = traj.getPieceNum();
    if (seg_num <= 1)
      return false;
    
    //set front-end traj
    front_end_traj_ = traj;
    
    if (type == SMOOTH_HOMO_OBS)
      return initialize_smooth_close_obs();
    else if (type == SMOOTH_HOMO)
      return initialize_smooth_close();
    else if (type == SMOOTH)
      return initialize_smooth(traj);
  }
  void setVisualizer(const VisualRviz::Ptr &vis)
  {
    vis_ptr_ = vis;
  }
  void setSearcher(const std::shared_ptr<AstarPathFinder> &searcher)
  {
    searcher_ = searcher;
  }
  bool solve_S_H(); // change smooth and homotopy weights
  bool solve_S_H_O(); // add and change obs penalty
  bool solve_S(); // fixed wpts
  bool solveRegionalOpt(std::vector<pair<int, int>> &seg_num_obs_size, 
                        std::vector<Eigen::Vector3d> &attract_pts, 
                        std::vector<double> &t_s, std::vector<double> &t_e);

  typedef shared_ptr<TrajOptimizer> Ptr;

private:
  PosChecker::Ptr pos_checker_ptr_;
  VisualRviz::Ptr vis_ptr_;
  std::shared_ptr<AstarPathFinder> searcher_;

  /** coefficient of polynomials*/
  Eigen::MatrixXd coeff_;  
  Eigen::MatrixXd coeff0_;
  Trajectory optimized_traj_, front_end_traj_;
  
  /** way points info, from start point to end point **/
  std::set<int> fixed_pos_;
  Eigen::MatrixXd path_;
  Eigen::MatrixXd vel_way_points_; 
  Eigen::MatrixXd acc_way_points_; 
  std::vector<double> time_;
  int m_; //segments number
  int n_; //fixed_pos number (exclude start and goal)

  
  /** important matrix and  variables*/
  Eigen::MatrixXd Ct_;
  Eigen::MatrixXd Z_, Zp_;
  Eigen::MatrixXd A_inv_multiply_Ct_;
  
  Eigen::MatrixXd A_;
  Eigen::MatrixXd A_inv_;
  Eigen::MatrixXd Q_smooth_, Q_close_, Q_obs_;

  vector<Eigen::MatrixXd> vQo_i_; //obs Q of the i^th segment
  vector<Eigen::MatrixXd> vcoeffo_i_;
 
  Eigen::MatrixXd D_;
  Eigen::MatrixXd Q_all_;
  Eigen::MatrixXd coeff_obs_;
  Eigen::MatrixXd Z_all_;

  double vel_limit_, acc_limit_, jerk_limit_;
  int ite_times_, max_iter_times_;
  bool initialize_smooth_close_obs();
  bool initialize_smooth_close();
  bool initialize_smooth(const Trajectory &traj);
  bool checkTrajectoryConstraints(const Trajectory &traj);
  void setWayPointsAndTime(std::vector<Eigen::Vector3d>& way_points,
                            std::vector<Eigen::Vector3d>& vel,
                            std::vector<Eigen::Vector3d>& acc,
                            std::vector<double>& time);
  void tryQPCloseForm(double percent_of_close);
  void tryQPCloseForm();
  void tryQP(const MatrixXd &Q_all, const MatrixXd &Z_all);

  void calMatrixA();
  void calMatrixCandMatrixZ(int type);
  void calMatrixC();
  void calMatrixQ_smooth(int type);
  void calMatrixQ_close();
  void calMatrixQ_obs(const vector<int> &segs, const vector<double> &t_s, const vector<double> &t_e);
  void calMatrixQAndCoeffPerObs(const Vector3d &attract_pts, const double &t_s, const double &t_e, 
                               Eigen::Matrix<double, 6, 6> &Q_oi, Eigen::Matrix<double, 6, 3> &coeff_oi);
  void calMatrixQobsAndCoeff(const vector<pair<int, int>> &seg_num_obs_size, const vector<Vector3d> &attract_pts, 
                               const vector<double> &t_s, const vector<double> &t_e);

  bool getVerticesFromTraj(vector<Vector3d> &vertices, 
                          vector<Vector3d> &vels, 
                          vector<Vector3d> &accs, 
                          const Trajectory &trajectory, std::vector<double> durs);
  bool getCollideSegIdx(const Trajectory& traj, std::set<int> &indices);
  void splitDurations(std::vector<double>& durs, const std::set<int> &indices);
  bool reTiming(Trajectory& traj);

  enum QSmoothType
  {
    MINIMUM_ACC, 
    MINIMUM_JERK, 
    MINIMUM_SNAP
  };
  enum CType
  {
    MIDDLE_P_V_CONSISTANT,
    MIDDLE_P_V_A_CONSISTANT
  };

public:
  enum OptimizeType
  {
    SMOOTH_HOMO_OBS,
    SMOOTH_HOMO,
    SMOOTH,
  };
};

}

#endif