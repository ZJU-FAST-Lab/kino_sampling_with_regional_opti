#include "kino_plan/bvp_solver.h"
#include "visualization_utils/visualization_utils.h"
#include "quadrotor_msgs/PositionCommand.h"
#include <ros/ros.h>
#include <Eigen/Eigen>

class BVPTest
{
public:
  BVPTest(const ros::NodeHandle &nh): nh_(nh)
  {
    nh.param("test/start_px", start_state_[0], 0.0);
    nh.param("test/start_py", start_state_[1], 0.0);
    nh.param("test/start_pz", start_state_[2], 0.0);
    nh.param("test/start_vx", start_state_[3], 0.0);
    nh.param("test/start_vy", start_state_[4], 0.0);
    nh.param("test/start_vz", start_state_[5], 0.0);
    nh.param("test/start_ax", start_state_[6], 0.0);
    nh.param("test/start_ay", start_state_[7], 0.0);
    nh.param("test/start_az", start_state_[8], 0.0);
    nh.param("test/rho", rho_, 0.0);

    vis_ptr_.reset(new VisualRviz(nh));
    goal_sub_ = nh_.subscribe("/goal", 1, &BVPTest::goalCallback, this);

    bvp_.init(TRIPLE_INTEGRATOR);
    bvp_.setRho(rho_);
  };

private:
  ros::NodeHandle nh_;
  BVPSolver::IntegratorBVP bvp_;
  VisualRviz::Ptr vis_ptr_;
  StatePVA start_state_, end_state_;
  double rho_;

  ros::Subscriber goal_sub_;
  void goalCallback(const quadrotor_msgs::PositionCommand::ConstPtr &goal_msg)
  {
    end_state_ << goal_msg->position.x,
        goal_msg->position.y,
        goal_msg->position.z,
        goal_msg->velocity.x + 1.0,
        goal_msg->velocity.y + 2.0,
        goal_msg->velocity.z,
        goal_msg->acceleration.x,
        goal_msg->acceleration.y,
        goal_msg->acceleration.z;
    solve(start_state_, end_state_);
  };

  /* 
  ratio: new duration / original duration
  */
  CoefficientMat reTiming(double ratio, const CoefficientMat &coeff)
  {
    CoefficientMat res_coeff;
    int order = coeff.cols();
    double t = 1.0;
    for (int curr_order = order - 1; curr_order >= 0; --curr_order)
    {
      res_coeff.col(curr_order) = coeff.col(curr_order) / t;
      t *= ratio;
    }
    return res_coeff;
  };
  
  void solve(const StatePVA &s, const StatePVA &e)
  {
    ROS_INFO_STREAM("[TEST]: solve s: " << s.transpose() << ", e: " << e.transpose());
    if (bvp_.solve(s, e))
    {
      CoefficientMat coeff;
      bvp_.getCoeff(coeff);
      std::cout << "ori_coeff: \n" << coeff << std::endl;
      double best_tau = bvp_.getTauStar();
      double best_cost = bvp_.getCostStar();
      ROS_INFO("[TEST]: Best cost: %lf, best tau: %lf", best_cost, best_tau);
      Piece poly_seg(best_tau, coeff);
      /* for traj vis */
      std::vector<StatePVA> vis_x;
      vis_x.clear();
      poly_seg.sampleOneSeg(&vis_x);
      vis_ptr_->visualizeStates(vis_x, BestTraj, ros::Time::now());

      double vel_max = poly_seg.getMaxVelRate();
      double acc_max = poly_seg.getMaxAccRate();
      double jerk_max = poly_seg.getMaxJerkRate();
      ROS_WARN("[TEST]: vel_max: %lf, acc_max: %lf, jerk_max: %lf", vel_max, acc_max, jerk_max);

      double tau_low = best_tau / 3.0;
      double tau_high = best_tau * 3.0;
      std::vector<std::vector<StatePVA>> vis_xs;
      std::vector<std::string> texts;
      std::vector<Eigen::Vector3d> texts_pos;
      for (double t = tau_low; t <= tau_high; t += 0.1)
      {
        ros::Time t1 = ros::Time::now();
        // bvp_.calCoeffFromTau(t, coeff);
        CoefficientMat new_coeff = reTiming(t / best_tau, coeff);
        ros::Time t2 = ros::Time::now();
        Piece curr_poly_seg(t, new_coeff);
        double vel_max = curr_poly_seg.getMaxVelRate();
        ros::Time t3 = ros::Time::now();
        double acc_max = curr_poly_seg.getMaxAccRate();
        ros::Time t4 = ros::Time::now();
        double jerk_max = curr_poly_seg.getMaxJerkRate();
        ROS_WARN_STREAM("[TEST]: re-calculate coeff uses: " << (t2 - t1).toSec() * 1e6 << " us, " << (t3 - t2).toSec() * 1e6 << " us, " << (t4 - t3).toSec() * 1e6 << " us");
        std::string s = "t: " + std::to_string(t) + ", v: " + std::to_string(vel_max) + ", a: " + std::to_string(acc_max) + ", j: " + std::to_string(jerk_max);
        texts.push_back(s);
        vis_x.clear();
        curr_poly_seg.sampleOneSeg(&vis_x);
        vis_xs.push_back(vis_x);
        ROS_ERROR("[TEST]: tau: %lf, vel_max: %lf, acc_max: %lf, jerk: %lf", t, vel_max, acc_max, jerk_max);
        Eigen::Vector3d p;
        p = curr_poly_seg.getPos(t / 2.0);
        texts_pos.push_back(p);
      }
      vis_ptr_->visualizeMultiTraj(vis_xs, ros::Time::now());
      vis_ptr_->visualizeText(texts, texts_pos, ros::Time::now());
    }
    else
    {
      ROS_INFO("sth wrong with bvp_solver");
    }
  };
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_bap_node");
  ros::NodeHandle nh("~");
  BVPTest bvp_tester(nh);
  ros::spin();
}