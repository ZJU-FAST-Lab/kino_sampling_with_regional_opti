#ifndef _TEST_PLANNERS_H_
#define _TEST_PLANNERS_H_

#include "self_msgs_and_srvs/GlbObsRcv.h"
#include "occ_grid/occ_map.h"
#include "occ_grid/pos_checker.h"
#include "kino_plan/krrtplanner.h"
#include "kino_plan/kfmt.h"
#include "kino_plan/topo_prm.h"
#include "visualization_utils/visualization_utils.h"
#include "poly_opt/traj_optimizer.h"
#include "quadrotor_msgs/PolynomialTrajectory.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "r3_plan/r3_planner.h"
#include "r3_plan/a_star_search.h"
#include "kino_plan/kinodynamic_astar.h"
#include "kino_plan/kd_astar_jerk.h"
#include "bspline_opt/bspline_optimizer.h"
#include <bspline/non_uniform_bspline.h>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Empty.h>

namespace tgk_planner
{
class PlannerTester
{
public:
  PlannerTester(){o_file_.open("/home/kai/ros_ws/learn_to_sample_ws/src/data/bench_forest.csv", ios::out | ios::app);};
  ~PlannerTester(){o_file_.close();};
  void init(const ros::NodeHandle& nh);
  bool searchForTraj(Vector3d start_pos, Vector3d start_vel, Vector3d start_acc,  
                     Vector3d end_pos, Vector3d end_vel, Vector3d end_acc, 
                     double search_time);
  bool testConvergence(Vector3d start_pos, Vector3d start_vel, Vector3d start_acc,  
                     Vector3d end_pos, Vector3d end_vel, Vector3d end_acc, 
                     double search_time);
  bool benchmarkOptimization(Vector3d start_pos, Vector3d start_vel, Vector3d start_acc,  
                     Vector3d end_pos, Vector3d end_vel, Vector3d end_acc, 
                     double search_time);

private:
  bool optimize(const Trajectory &traj);
  bool optimize_old(const Trajectory &traj);
  bool optimize_liu(const Trajectory &traj);
  bool bSplineOpt(const Trajectory& front_traj, fast_planner::NonUniformBspline& pos);

  // map, checker, planner 
  OccMap::Ptr env_ptr_;
  PosChecker::Ptr pos_checker_ptr_;
  unique_ptr<TopologyPRM> topo_prm_;
  KFMTPlanner::KFMTPlannerPtr fmt_planner_ptr_;
  KRRTPlanner::KRRTPlannerPtr krrt_planner_ptr_;
  TrajOptimizer::Ptr optimizer_ptr_;
  VisualRviz::Ptr vis_ptr_;
  shared_ptr<R3Planner> r3_planer_ptr_;
  shared_ptr<AstarPathFinder> astar_searcher_;
  KinodynamicAstar::Ptr kastar_traj_finder_;
  KinodynamicAstarJ::Ptr kastar_jerk_finder_;
  fast_planner::BsplineOptimizer::Ptr bspline_optimizer_;
  
  // ros 
  ros::NodeHandle nh_;
  ros::Timer execution_timer_;
  ros::Subscriber goal_sub_;
  ros::Publisher traj_pub_;
  ros::ServiceClient rcv_glb_obs_client_;
  void goalCallback(const quadrotor_msgs::PositionCommand::ConstPtr& goal_msg);
  void executionCallback(const ros::TimerEvent& event);

  // params 
  Eigen::Vector3d start_pos_, start_vel_, start_acc_, end_pos_, end_vel_, end_acc_;
  bool close_goal_traj_;
  bool new_goal_, started_, use_optimization_;
  Eigen::Vector3d last_goal_pos_;
  Trajectory front_end_traj_, back_end_traj_, traj_;
  bool map_initialized;
  double replan_time_;
  bool runned;
  double max_vel_, max_acc_, ctrl_pt_dist_;

  std::ofstream o_file_;
  double start_x_, start_y_, start_z_;
  bool use_bench_, use_r3_;
  bool test_rrt_w_, test_rrt_wo_, test_a_double_, test_a_triple_, test_fmt_w_, test_fmt_wo_;
  int repeat_times_;
  int rrt_w_succ_num_, rrt_wo_suss_num_;
  int opt_succ_num_, opt_old_succ_num_, opt_liu_succ_num_, opt_bspline_succ_num_;
  int a_double_succ_num_, a_tripla_succ_num_, fmt_w_succ_num_, fmt_wo_succ_num_;
};

void PlannerTester::init(const ros::NodeHandle &nh)
{
  env_ptr_.reset(new OccMap);
  env_ptr_->init(nh);

  vis_ptr_.reset(new VisualRviz(nh));

  pos_checker_ptr_.reset(new PosChecker);
  pos_checker_ptr_->init(nh);
  pos_checker_ptr_->setMap(env_ptr_);

  astar_searcher_.reset(new AstarPathFinder());
  astar_searcher_->initGridMap(pos_checker_ptr_, pos_checker_ptr_->getOccMapSize());

  topo_prm_.reset(new TopologyPRM);
  topo_prm_->setPoschecker(pos_checker_ptr_);
  topo_prm_->init(nh);

  kastar_traj_finder_.reset(new KinodynamicAstar);
  kastar_traj_finder_->setParam(nh);
  kastar_traj_finder_->setPosChecker(pos_checker_ptr_);
  kastar_traj_finder_->init();

  kastar_jerk_finder_.reset(new KinodynamicAstarJ);
  kastar_jerk_finder_->setParam(nh);
  kastar_jerk_finder_->setPosChecker(pos_checker_ptr_);
  kastar_jerk_finder_->init();
  
  optimizer_ptr_.reset(new TrajOptimizer(nh));
  optimizer_ptr_->setPosChecker(pos_checker_ptr_);
  optimizer_ptr_->setVisualizer(vis_ptr_);
  optimizer_ptr_->setSearcher(astar_searcher_);

  bspline_optimizer_.reset(new fast_planner::BsplineOptimizer);
  bspline_optimizer_->setParam(nh);
  bspline_optimizer_->setEnvironment(pos_checker_ptr_);

  r3_planer_ptr_.reset(new R3Planner(nh, pos_checker_ptr_));

  krrt_planner_ptr_.reset(new KRRTPlanner(nh));
  krrt_planner_ptr_->init(nh);
  krrt_planner_ptr_->setPosChecker(pos_checker_ptr_);
  krrt_planner_ptr_->setVisualizer(vis_ptr_);
  krrt_planner_ptr_->setRegionalOptimizer(optimizer_ptr_);
  krrt_planner_ptr_->setSearcher(astar_searcher_);

  fmt_planner_ptr_.reset(new KFMTPlanner(nh));
  fmt_planner_ptr_->init(nh);
  fmt_planner_ptr_->setPosChecker(pos_checker_ptr_);
  fmt_planner_ptr_->setVisualizer(vis_ptr_);
  fmt_planner_ptr_->setRegionalOptimizer(optimizer_ptr_);
  fmt_planner_ptr_->setSearcher(astar_searcher_);

  execution_timer_ = nh_.createTimer(ros::Duration(0.01), &PlannerTester::executionCallback, this); // 100Hz
  goal_sub_ = nh_.subscribe("/goal", 1, &PlannerTester::goalCallback, this);
  traj_pub_ = nh_.advertise<quadrotor_msgs::PolynomialTrajectory>("/front_traj", 10);
  rcv_glb_obs_client_ = nh_.serviceClient<self_msgs_and_srvs::GlbObsRcv>("/pub_glb_obs");

  nh.param("test_plan/start_x", start_x_, 0.0);
  nh.param("test_plan/start_y", start_y_, 0.0);
  nh.param("test_plan/start_z", start_z_, 0.0);
  nh.param("test_plan/use_optimization", use_optimization_, false);
  nh.param("test_plan/use_r3", use_r3_, false);
  nh.param("test_plan/replan_time", replan_time_, 0.03);
  nh.param("test_plan/use_bench", use_bench_, false);
  nh.param("test_plan/test_rrt_w", test_rrt_w_, false);
  nh.param("test_plan/test_rrt_wo", test_rrt_wo_, false);
  nh.param("test_plan/test_a_double", test_a_double_, false);
  nh.param("test_plan/test_a_triple", test_a_triple_, false);
  nh.param("test_plan/test_fmt_w", test_fmt_w_, false);
  nh.param("test_plan/test_fmt_wo", test_fmt_wo_, false);
  nh.param("test_plan/repeat_times", repeat_times_, 0);
  //bspline
  nh.param("test_plan/ctrl_pt_dist", ctrl_pt_dist_, 0.0);
  nh.param("test_plan/max_vel", max_vel_, 0.0);
  nh.param("test_plan/max_acc", max_acc_, 0.0);

  ROS_WARN_STREAM("[test_plan] param: use_optimization: " << use_optimization_);
  ROS_WARN_STREAM("[test_plan] param: replan_time: " << replan_time_);
  ROS_WARN_STREAM("[test_plan] param: use_bench: " << use_bench_);
  ROS_WARN_STREAM("[test_plan] param: repeat_times: " << repeat_times_);
  
  new_goal_ = false;
  started_ = false;
  last_goal_pos_ << start_x_, start_y_, start_z_;
  map_initialized = false;
  runned = false;

  rrt_w_succ_num_ = 0;
  rrt_wo_suss_num_ = 0;
  opt_succ_num_ = 0;
  opt_old_succ_num_ = 0;
  a_double_succ_num_ = 0;
  a_tripla_succ_num_ = 0;
  fmt_w_succ_num_ = 0;
  fmt_wo_succ_num_ = 0;
}

void PlannerTester::executionCallback(const ros::TimerEvent &event)
{
  static int fsm_num = 0;
  fsm_num++;
  if (fsm_num == 100)
  {
    if (!env_ptr_->mapValid())
    {
      ROS_INFO("no map.");
      self_msgs_and_srvs::GlbObsRcv srv;
      if (!rcv_glb_obs_client_.call(srv))
        ROS_WARN("Failed to call service /pub_glb_obs");
    }
    else if (!runned && use_bench_)
    {
      map_initialized = true;

      /*********  bench convergence rrt w/ & rrt w/o ************/
      // Vector3d start_pos(12.0, -12.0, 0.0), start_vel(0.0, 0.0, 0.0), start_acc(0.0, 0.0, 0.0);
      // Vector3d end_pos(-12.0, 12.0, 0.0), end_vel(0.0, 0.0, 0.0), end_acc(0.0, 0.0, 0.0);
      // int n(0);
      // rrt_w_succ_num_ = 0;
      // rrt_wo_suss_num_ = 0;
      // opt_succ_num_ = 0;
      // opt_old_succ_num_ = 0;
      // a_double_succ_num_ = 0;
      // a_tripla_succ_num_ = 0;
      // fmt_w_succ_num_ = 0;
      // fmt_wo_succ_num_ = 0;
      // while (n < repeat_times_)
      // {
      //   ROS_ERROR_STREAM("start bench No." << n);
      //   if (testConvergence(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, replan_time_))
      //     n++;
      // }  
      /*********  bench convergence rrt w/ & rrt w/o ************/


      /*********  bench back-end w/ & w/o & liu & bspline ************/
      // Vector3d start_pos(5.0, -30.0, 0.0), start_vel(0.0, 0.0, 0.0), start_acc(0.0, 0.0, 0.0);
      // Vector3d end_pos(-5.0, 30.0, 0.0), end_vel(0.0, 0.0, 0.0), end_acc(0.0, 0.0, 0.0);
      // int n(0);
      // rrt_w_succ_num_ = 0;
      // rrt_wo_suss_num_ = 0;
      // opt_succ_num_ = 0;
      // opt_old_succ_num_ = 0;
      // opt_liu_succ_num_ = 0;
      // opt_bspline_succ_num_ = 0;
      // while (n < repeat_times_)
      // {
      //   ROS_ERROR_STREAM("start benchmarking back-end No." << n);
      //   if (benchmarkOptimization(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, replan_time_))
      //     n++;
      // }  
      /*********  bench back-end w/ & w/o & liu & bspline ************/


      /*********  bench front-end ************/
      std::random_device rd;
      std::mt19937_64 gen = std::mt19937_64(rd());            
      std::uniform_real_distribution<double> y_rand = std::uniform_real_distribution<double>(-1.0, 1.0);
      Vector3d start_pos(5.0, -30.0, 0.0), start_vel(0.0, 0.0, 0.0), start_acc(0.0, 0.0, 0.0);
      Vector3d end_pos(-5.0, 30.0, 0.0), end_vel(0.0, 0.0, 0.0), end_acc(0.0, 0.0, 0.0);
      int n(0);
      rrt_w_succ_num_ = 0;
      rrt_wo_suss_num_ = 0;
      opt_succ_num_ = 0;
      opt_old_succ_num_ = 0;
      a_double_succ_num_ = 0;
      a_tripla_succ_num_ = 0;
      fmt_w_succ_num_ = 0;
      fmt_wo_succ_num_ = 0;
      while (n < repeat_times_)
      {
        ROS_ERROR_STREAM("start bench No." << n);
        if (searchForTraj(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, replan_time_))
          n++;
        double y_bias = y_rand(gen);
        start_pos.x() = -5 + y_bias;
        end_pos.x() = 5 + y_bias;
      }  
      /*********  bench front-end ************/

      o_file_ << endl;
      o_file_ << rrt_w_succ_num_ << "," 
            << opt_succ_num_ << "," 
            << opt_old_succ_num_ << "," 
            << opt_liu_succ_num_ << "," 
            << opt_bspline_succ_num_ << "," 
            << rrt_wo_suss_num_ << "," 
            << a_double_succ_num_ << ","
            << a_tripla_succ_num_ << "," 
            << fmt_w_succ_num_ << "," 
            << fmt_wo_succ_num_ << "," << ","; 
      o_file_ << endl;
      runned = true;

    }
    
    fsm_num = 0;
  }
}


void PlannerTester::goalCallback(const quadrotor_msgs::PositionCommand::ConstPtr &goal_msg)
{
  end_pos_ << goal_msg->position.x,
              goal_msg->position.y,
              goal_msg->position.z;
  end_vel_ << goal_msg->velocity.x,
              goal_msg->velocity.y,
              goal_msg->velocity.z;
  end_acc_ << goal_msg->acceleration.x,
              goal_msg->acceleration.y,
              goal_msg->acceleration.z;
  started_ = true;
  new_goal_ = true;
  searchForTraj(last_goal_pos_, Eigen::Vector3d(0.0,0.0,0.0), Eigen::Vector3d(0.0,0.0,0.0), end_pos_, end_vel_, end_acc_, replan_time_);
  last_goal_pos_ = end_pos_;

}

bool PlannerTester::benchmarkOptimization(Vector3d start_pos, Vector3d start_vel, Vector3d start_acc,
                        Vector3d end_pos, Vector3d end_vel, Vector3d end_acc,
                        double search_time)
{
  bool tried_once(false);
  int result(false);
  double traj_len(0.0), traj_duration(0.0), traj_acc_itg(0.0), traj_jerk_itg(0.0);
  int traj_seg_nums(0);

  vis_ptr_->visualizeStartAndGoal(start_pos, end_pos, pos_checker_ptr_->getLocalTime());

  /* r3planner   */
  if (use_r3_)
  {
  double len_cost(0.0);
  vector<Vector3d> route;
  vector<vector<Vector3d>> routes;
  ros::Time t1 = ros::Time::now();
  double radius(16); //radius square
  if (r3_planer_ptr_->planOnce(start_pos, end_pos, route, len_cost, radius))
  {
    tried_once = true;
    double r3_use_time = (ros::Time::now() - t1).toSec() * 1e3;
    ROS_ERROR_STREAM("r3 plan solved in: " << r3_use_time << " ms, route len: " << len_cost);
    // vector<vector<Vector3d>> select_paths;
    // size_t v_n = r3_planer_ptr_->getGraph(select_paths);
    // vis_ptr_->visualizePRM(select_paths, Color::Teal(), env_ptr_->getLocalTime());
    // getchar();
    routes.push_back(route);
    // vis_ptr_->visualizePRM(routes, Color::Red(), env_ptr_->getLocalTime());

    // vector<double> radii;
    // for (const auto & p: route)
    // {
    //   Vector3d obs;
    //   double radius = sqrt(pos_checker_ptr_->nearestObs(p, obs));
    //   radii.push_back(radius);
    // }
    // vis_ptr_->visualizeBalls(route, radii, env_ptr_->getLocalTime());
    krrt_planner_ptr_->sampler_.topoSetup(routes);
    fmt_planner_ptr_->sampler_.topoSetup(routes);
    // o_file_ << r3_use_time << "," << ","; // r3 use time (ms)
  }
  else
  {
    ROS_WARN("r3 plan fail");
    return false;
    // o_file_ << "," << ","; // r3 use time (ms)
  }
  }
  

  bool rrt_w_res(false);
  krrt_planner_ptr_->reset();
  ros::Time t_krrt_with_s = ros::Time::now();
  rrt_w_res = krrt_planner_ptr_->plan(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, search_time, Vector3d(0,0,0), Vector3d(0,0,0), 0, 1);
  double time_krrt_w = (ros::Time::now() - t_krrt_with_s).toSec();
  if (rrt_w_res == KRRTPlanner::SUCCESS)
  {
    tried_once = true;
    rrt_w_succ_num_++;
    Trajectory krrt_final_traj, krrt_first_traj;
    krrt_planner_ptr_->getFirstTraj(krrt_first_traj);
    krrt_planner_ptr_->getTraj(krrt_final_traj);
    std::vector<StatePVA> vis_x;
    vis_x.clear();
    krrt_first_traj.sampleWholeTrajectory(&vis_x);
    vis_ptr_->visualizeStates(vis_x, FirstTraj, pos_checker_ptr_->getLocalTime());
    // vis_x.clear();
    // krrt_final_traj.sampleWholeTrajectory(&vis_x);
    // vis_ptr_->visualizeStates(vis_x, FinalTraj, pos_checker_ptr_->getLocalTime());
    
    double first_traj_len(0.0), first_traj_duration(0.0), first_traj_acc_itg(0.0), first_traj_jerk_itg(0.0);
    double final_traj_len(0.0), final_traj_duration(0.0), final_traj_acc_itg(0.0), final_traj_jerk_itg(0.0);
    int first_traj_seg_nums(0), final_traj_seg_nums(0);
    krrt_planner_ptr_->evaluateTraj(krrt_first_traj, first_traj_duration, first_traj_len, first_traj_seg_nums, first_traj_acc_itg, first_traj_jerk_itg);
    krrt_planner_ptr_->evaluateTraj(krrt_final_traj, final_traj_duration, final_traj_len, final_traj_seg_nums, final_traj_acc_itg, final_traj_jerk_itg);
    double first_traj_use_time = krrt_planner_ptr_->getFirstTrajTimeUsage();
    double final_traj_use_time = krrt_planner_ptr_->getFinalTrajTimeUsage();
    cout << "==================== KRRT w/ =========================================" << endl;
    ROS_INFO_STREAM("[KRRT]: [front-end first path]: " << endl 
        << "    -   seg nums: " << first_traj_seg_nums << endl 
        << "    -   time: " << first_traj_use_time * 1e3 << " ms" << ", time + func_call: " << time_krrt_w * 1e3 << " ms" << endl 
        << "    -   acc integral: " << first_traj_acc_itg << endl
        << "    -   jerk integral: " << first_traj_jerk_itg << endl
        << "    -   traj duration: " << first_traj_duration << endl 
        << "    -   path length: " << first_traj_len << " m");
        
    ROS_INFO_STREAM("[KRRT]: [front-end final path]: " << endl 
        << "    -   seg nums: " << final_traj_seg_nums << endl 
        << "    -   time: " << final_traj_use_time * 1e3 << " ms"<< endl 
        << "    -   acc integral: " << final_traj_acc_itg << endl
        << "    -   jerk integral: " << final_traj_jerk_itg << endl
        << "    -   traj duration: " << final_traj_duration << endl 
        << "    -   path length: " << final_traj_len << " m");
    cout << "=======================================================================" << endl;
    o_file_ << first_traj_use_time * 1e3 << "," 
            << first_traj_seg_nums << "," 
            << first_traj_acc_itg << "," 
            << first_traj_jerk_itg << "," 
            << first_traj_duration << ","
            << first_traj_len << "," << ","; 

    o_file_ << final_traj_use_time * 1e3 << "," 
            << final_traj_seg_nums << "," 
            << final_traj_acc_itg << "," 
            << final_traj_jerk_itg << "," 
            << final_traj_duration << ","
            << final_traj_len << "," << ","; 
    
    if (use_optimization_)
    {
      ros::Time optimize_start_time = ros::Time::now();
      bool res = optimize(krrt_first_traj);
      ros::Time optimize_end_time = ros::Time::now();
      if (res)
      {
        opt_succ_num_++;
        Trajectory optimized_traj;
        optimizer_ptr_->getTraj(optimized_traj);
        std::vector<StatePVA> vis_x;
        vis_x.clear();
        optimized_traj.sampleWholeTrajectory(&vis_x);
        vis_ptr_->visualizeStates(vis_x, FinalTraj, pos_checker_ptr_->getLocalTime());
        krrt_planner_ptr_->evaluateTraj(optimized_traj, traj_duration, traj_len, traj_seg_nums, traj_acc_itg, traj_jerk_itg);
        o_file_ << (optimize_end_time - optimize_start_time).toSec() * 1e3 << "," 
                << traj_seg_nums << "," 
                << traj_acc_itg << "," 
                << traj_jerk_itg << "," 
                << traj_duration << ","
                << traj_len << "," 
                << optimizer_ptr_->getIterationTimes() << "," << ","; 
      }
      else
      {
        o_file_ << "," 
                << "," 
                << "," 
                << "," 
                << "," 
                << ","
                << "," << ","; 
      }

      optimize_start_time = ros::Time::now();
      bool res_old_opt_method = optimize_old(krrt_first_traj);
      optimize_end_time = ros::Time::now();
      if (res_old_opt_method)
      {
        opt_old_succ_num_++;
        Trajectory optimized_traj;
        optimizer_ptr_->getTraj(optimized_traj);
        std::vector<StatePVA> vis_x;
        vis_x.clear();
        optimized_traj.sampleWholeTrajectory(&vis_x);
        vis_ptr_->visualizeStates(vis_x, OptimizedTraj, pos_checker_ptr_->getLocalTime());
        krrt_planner_ptr_->evaluateTraj(optimized_traj, traj_duration, traj_len, traj_seg_nums, traj_acc_itg, traj_jerk_itg);
        o_file_ << (optimize_end_time - optimize_start_time).toSec() * 1e3 << "," 
                << traj_seg_nums << "," 
                << traj_acc_itg << "," 
                << traj_jerk_itg << "," 
                << traj_duration << ","
                << traj_len << "," 
                << optimizer_ptr_->getIterationTimes() << "," << ","; 
      }
      else
      {
        o_file_ << "," 
                << "," 
                << "," 
                << "," 
                << "," 
                << ","
                << "," << ","; 
      }

      optimize_start_time = ros::Time::now();
      bool res_liu_opt_method = optimize_liu(krrt_first_traj);
      optimize_end_time = ros::Time::now();
      if (res_liu_opt_method)
      {
        opt_liu_succ_num_++;
        Trajectory optimized_traj;
        optimizer_ptr_->getTraj(optimized_traj);
        std::vector<StatePVA> vis_x;
        vis_x.clear();
        optimized_traj.sampleWholeTrajectory(&vis_x);
        vis_ptr_->visualizeStates(vis_x, BestTraj, pos_checker_ptr_->getLocalTime());
        krrt_planner_ptr_->evaluateTraj(optimized_traj, traj_duration, traj_len, traj_seg_nums, traj_acc_itg, traj_jerk_itg);
        o_file_ << (optimize_end_time - optimize_start_time).toSec() * 1e3 << "," 
                << traj_seg_nums << "," 
                << traj_acc_itg << "," 
                << traj_jerk_itg << "," 
                << traj_duration << ","
                << traj_len << "," 
                << optimizer_ptr_->getIterationTimes() << "," << ","; 
      }
      else
      {
        o_file_ << "," 
                << "," 
                << "," 
                << "," 
                << "," 
                << ","
                << "," << ","; 
      }

      optimize_start_time = ros::Time::now();
      fast_planner::NonUniformBspline bspline_traj;
      bool res_bspline_opt = bSplineOpt(krrt_first_traj, bspline_traj);
      optimize_end_time = ros::Time::now();
      if (res_bspline_opt)
      {
        opt_bspline_succ_num_++;
        std::vector<StatePVA> vis_x;
        vis_x.clear();
        
        double dur = bspline_traj.getTimeSum();
        Eigen::VectorXd p_n, v_n, a_n;
        double res = 0.05;
        fast_planner::NonUniformBspline vel_traj = bspline_traj.getDerivative();
        fast_planner::NonUniformBspline acc_traj = bspline_traj.getDerivative().getDerivative();
        for (double t = 0.0; t <= dur + 1e-4; t += res) 
        {
          p_n = bspline_traj.evaluateDeBoorT(t);
          v_n = vel_traj.evaluateDeBoorT(t);
          a_n = acc_traj.evaluateDeBoorT(t);
          StatePVA pva;
          pva.head(3) = p_n;
          pva.segment(3, 3) = v_n;
          pva.tail(3) = a_n;
          vis_x.push_back(pva);
        }
        vis_ptr_->visualizeStates(vis_x, FMTTraj, pos_checker_ptr_->getLocalTime());

        traj_len = bspline_traj.getLength();
        traj_duration = bspline_traj.getTimeSum();
        traj_acc_itg = bspline_traj.getAcc();
        traj_jerk_itg = bspline_traj.getJerk();
        o_file_ << (optimize_end_time - optimize_start_time).toSec() * 1e3 << "," 
                << 0.0 << "," 
                << traj_acc_itg << "," 
                << traj_jerk_itg << "," 
                << traj_duration << ","
                << traj_len << ","
                << bspline_optimizer_->getIterationTimes() << "," << ","; 
      }
      else
      {
        o_file_ << "," 
                << "," 
                << "," 
                << "," 
                << "," 
                << ","
                << "," << ","; 
      }

    }
  }
  else
  {
    return false;
  }
  o_file_ << endl;
  return tried_once;
}


bool PlannerTester::testConvergence(Vector3d start_pos, Vector3d start_vel, Vector3d start_acc,
                        Vector3d end_pos, Vector3d end_vel, Vector3d end_acc,
                        double search_time)
{
  bool tried_once(false);
  int result(false);
  double traj_len(0.0), traj_duration(0.0), traj_acc_itg(0.0), traj_jerk_itg(0.0);
  int traj_seg_nums(0);

  vis_ptr_->visualizeStartAndGoal(start_pos, end_pos, pos_checker_ptr_->getLocalTime());

  /* r3planner   */
  if (use_r3_)
  {
  double len_cost(0.0);
  vector<Vector3d> route;
  vector<vector<Vector3d>> routes;
  ros::Time t1 = ros::Time::now();
  double radius(16); //radius square
  if (r3_planer_ptr_->planOnce(start_pos, end_pos, route, len_cost, radius))
  {
    tried_once = true;
    double r3_use_time = (ros::Time::now() - t1).toSec() * 1e3;
    ROS_ERROR_STREAM("r3 plan solved in: " << r3_use_time << " ms, route len: " << len_cost);
    // vector<vector<Vector3d>> select_paths;
    // size_t v_n = r3_planer_ptr_->getGraph(select_paths);
    // vis_ptr_->visualizePRM(select_paths, Color::Teal(), env_ptr_->getLocalTime());
    // getchar();
    routes.push_back(route);
    // vis_ptr_->visualizePRM(routes, Color::Red(), env_ptr_->getLocalTime());

    // vector<double> radii;
    // for (const auto & p: route)
    // {
    //   Vector3d obs;
    //   double radius = sqrt(pos_checker_ptr_->nearestObs(p, obs));
    //   radii.push_back(radius);
    // }
    // vis_ptr_->visualizeBalls(route, radii, env_ptr_->getLocalTime());
    krrt_planner_ptr_->sampler_.topoSetup(routes);
    fmt_planner_ptr_->sampler_.topoSetup(routes);
    // o_file_ << r3_use_time << "," << ","; // r3 use time (ms)
  }
  else
  {
    ROS_WARN("r3 plan fail");
    return false;
    // o_file_ << "," << ","; // r3 use time (ms)
  }
  }
  else
  {
    tried_once = true;
  }
  

  bool rrt_w_res(false);
  if (test_rrt_w_)
  // while (rrt_w_res != KRRTPlanner::SUCCESS)
  {
  krrt_planner_ptr_->reset();
  ros::Time t_krrt_with_s = ros::Time::now();
  rrt_w_res = krrt_planner_ptr_->plan(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, search_time, Vector3d(0,0,0), Vector3d(0,0,0), 0, 1);
  double time_krrt_w = (ros::Time::now() - t_krrt_with_s).toSec();
  if (rrt_w_res == KRRTPlanner::SUCCESS)
  {
    rrt_w_succ_num_++;
    vector<Trajectory> traj_list;
    vector<double> solution_cost_list;
    vector<double> solution_time_list;
    krrt_planner_ptr_->getConvergenceInfo(traj_list, solution_cost_list, solution_time_list);
    vector<vector<StatePVA>> trajs_states;
    for (size_t i = 0; i < traj_list.size(); ++i)
    {
      vector<StatePVA> single_traj_states;
      traj_list[i].sampleWholeTrajectory(&single_traj_states);
      trajs_states.push_back(single_traj_states);
      double curr_traj_len(0.0), curr_traj_duration(0.0), curr_traj_acc_itg(0.0), curr_traj_jerk_itg(0.0);
      int curr_traj_seg_nums(0);
      krrt_planner_ptr_->evaluateTraj(traj_list[i], curr_traj_duration, curr_traj_len, curr_traj_seg_nums, curr_traj_acc_itg, curr_traj_jerk_itg);
      o_file_ << solution_time_list[i] * 1e3 << "," 
            << curr_traj_seg_nums << "," 
            << curr_traj_acc_itg << "," 
            << curr_traj_jerk_itg << "," 
            << solution_cost_list[i] << ","
            << curr_traj_duration << ","
            << curr_traj_len << "," << i << endl; 
    }
    vis_ptr_->visualizeTrajList(trajs_states, pos_checker_ptr_->getLocalTime());
    // o_file_ << endl;
    return true;
  }
  }

  bool rrt_wo_res(false);
  if (test_rrt_wo_)
  // while (rrt_w_res != KRRTPlanner::SUCCESS)
  {
  krrt_planner_ptr_->reset();
  ros::Time t_krrt_with_s = ros::Time::now();
  rrt_wo_res = krrt_planner_ptr_->plan(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, search_time, Vector3d(0,0,0), Vector3d(0,0,0), 0, 0);
  double time_krrt_w = (ros::Time::now() - t_krrt_with_s).toSec();
  if (rrt_wo_res == KRRTPlanner::SUCCESS)
  {
    rrt_w_succ_num_++;
    vector<Trajectory> traj_list;
    vector<double> solution_cost_list;
    vector<double> solution_time_list;
    krrt_planner_ptr_->getConvergenceInfo(traj_list, solution_cost_list, solution_time_list);
    vector<vector<StatePVA>> trajs_states;
    for (size_t i = 0; i < traj_list.size(); ++i)
    {
      vector<StatePVA> single_traj_states;
      traj_list[i].sampleWholeTrajectory(&single_traj_states);
      trajs_states.push_back(single_traj_states);
      double curr_traj_len(0.0), curr_traj_duration(0.0), curr_traj_acc_itg(0.0), curr_traj_jerk_itg(0.0);
      int curr_traj_seg_nums(0);
      krrt_planner_ptr_->evaluateTraj(traj_list[i], curr_traj_duration, curr_traj_len, curr_traj_seg_nums, curr_traj_acc_itg, curr_traj_jerk_itg);
      o_file_ << solution_time_list[i] * 1e3 << "," 
            << curr_traj_seg_nums << "," 
            << curr_traj_acc_itg << "," 
            << curr_traj_jerk_itg << "," 
            << solution_cost_list[i] << ","
            << curr_traj_duration << ","
            << curr_traj_len << "," << i << endl; 
    }
    vis_ptr_->visualizeTrajList(trajs_states, pos_checker_ptr_->getLocalTime());
    // o_file_ << endl;
    return true;
  }
  }

}



bool PlannerTester::searchForTraj(Vector3d start_pos, Vector3d start_vel, Vector3d start_acc,
                        Vector3d end_pos, Vector3d end_vel, Vector3d end_acc,
                        double search_time)
{
  bool tried_once(false);
  int result(false);
  double traj_len(0.0), traj_duration(0.0), traj_acc_itg(0.0), traj_jerk_itg(0.0);
  int traj_seg_nums(0);

  vis_ptr_->visualizeStartAndGoal(start_pos, end_pos, pos_checker_ptr_->getLocalTime());

  /* r3planner   */
  if (use_r3_)
  {
  double len_cost(0.0);
  vector<Vector3d> route;
  vector<vector<Vector3d>> routes;
  ros::Time t1 = ros::Time::now();
  double radius(16); //radius square
  if (r3_planer_ptr_->planOnce(start_pos, end_pos, route, len_cost, radius))
  {
    tried_once = true;
    double r3_use_time = (ros::Time::now() - t1).toSec() * 1e3;
    ROS_ERROR_STREAM("r3 plan solved in: " << r3_use_time << " ms, route len: " << len_cost);
    // vector<vector<Vector3d>> select_paths;
    // size_t v_n = r3_planer_ptr_->getGraph(select_paths);
    // vis_ptr_->visualizePRM(select_paths, Color::Teal(), env_ptr_->getLocalTime());
    // getchar();
    routes.push_back(route);
    // vis_ptr_->visualizePRM(routes, Color::Red(), env_ptr_->getLocalTime());

    // vector<double> radii;
    // for (const auto & p: route)
    // {
    //   Vector3d obs;
    //   double radius = sqrt(pos_checker_ptr_->nearestObs(p, obs));
    //   radii.push_back(radius);
    // }
    // vis_ptr_->visualizeBalls(route, radii, env_ptr_->getLocalTime());
    krrt_planner_ptr_->sampler_.topoSetup(routes);
    fmt_planner_ptr_->sampler_.topoSetup(routes);
    o_file_ << r3_use_time << "," << ","; // r3 use time (ms)
  }
  else
  {
    ROS_WARN("r3 plan fail");
    return false;
    o_file_ << "," << ","; // r3 use time (ms)
  }
  }
  else
  {
    tried_once = true;
  }
  

  bool rrt_w_res(false);
  if (test_rrt_w_)
  // while (rrt_w_res != KRRTPlanner::SUCCESS)
  {
  krrt_planner_ptr_->reset();
  ros::Time t_krrt_with_s = ros::Time::now();
  rrt_w_res = krrt_planner_ptr_->plan(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, search_time, Vector3d(0,0,0), Vector3d(0,0,0), 0, 1);
  double time_krrt_w = (ros::Time::now() - t_krrt_with_s).toSec();
  if (rrt_w_res == KRRTPlanner::SUCCESS)
  {
    rrt_w_succ_num_++;
    Trajectory krrt_final_traj, krrt_first_traj;
    krrt_planner_ptr_->getFirstTraj(krrt_first_traj);
    krrt_planner_ptr_->getTraj(krrt_final_traj);
    std::vector<StatePVA> vis_x;
    vis_x.clear();
    krrt_first_traj.sampleWholeTrajectory(&vis_x);
    vis_ptr_->visualizeStates(vis_x, FirstTraj, pos_checker_ptr_->getLocalTime());
    // vis_x.clear();
    // krrt_final_traj.sampleWholeTrajectory(&vis_x);
    // vis_ptr_->visualizeStates(vis_x, FinalTraj, pos_checker_ptr_->getLocalTime());
    
    double first_traj_len(0.0), first_traj_duration(0.0), first_traj_acc_itg(0.0), first_traj_jerk_itg(0.0);
    double final_traj_len(0.0), final_traj_duration(0.0), final_traj_acc_itg(0.0), final_traj_jerk_itg(0.0);
    int first_traj_seg_nums(0), final_traj_seg_nums(0);
    krrt_planner_ptr_->evaluateTraj(krrt_first_traj, first_traj_duration, first_traj_len, first_traj_seg_nums, first_traj_acc_itg, first_traj_jerk_itg);
    krrt_planner_ptr_->evaluateTraj(krrt_final_traj, final_traj_duration, final_traj_len, final_traj_seg_nums, final_traj_acc_itg, final_traj_jerk_itg);
    double first_traj_use_time = krrt_planner_ptr_->getFirstTrajTimeUsage();
    double final_traj_use_time = krrt_planner_ptr_->getFinalTrajTimeUsage();
    cout << "==================== KRRT w/ =========================================" << endl;
    ROS_INFO_STREAM("[KRRT]: [front-end first path]: " << endl 
        << "    -   seg nums: " << first_traj_seg_nums << endl 
        << "    -   time: " << first_traj_use_time * 1e3 << " ms" << ", time + func_call: " << time_krrt_w * 1e3 << " ms" << endl 
        << "    -   acc integral: " << first_traj_acc_itg << endl
        << "    -   jerk integral: " << first_traj_jerk_itg << endl
        << "    -   traj duration: " << first_traj_duration << endl 
        << "    -   path length: " << first_traj_len << " m");
        
    ROS_INFO_STREAM("[KRRT]: [front-end final path]: " << endl 
        << "    -   seg nums: " << final_traj_seg_nums << endl 
        << "    -   time: " << final_traj_use_time * 1e3 << " ms"<< endl 
        << "    -   acc integral: " << final_traj_acc_itg << endl
        << "    -   jerk integral: " << final_traj_jerk_itg << endl
        << "    -   traj duration: " << final_traj_duration << endl 
        << "    -   path length: " << final_traj_len << " m");
    cout << "=======================================================================" << endl;
    o_file_ << first_traj_use_time * 1e3 << "," 
            << first_traj_seg_nums << "," 
            << first_traj_acc_itg << "," 
            << first_traj_jerk_itg << "," 
            << first_traj_duration << ","
            << first_traj_len << "," << ","; 

    o_file_ << final_traj_use_time * 1e3 << "," 
            << final_traj_seg_nums << "," 
            << final_traj_acc_itg << "," 
            << final_traj_jerk_itg << "," 
            << final_traj_duration << ","
            << final_traj_len << "," << ","; 
    
    if (use_optimization_)
    {
      ros::Time optimize_start_time = ros::Time::now();
      bool res = optimize(krrt_first_traj);
      ros::Time optimize_end_time = ros::Time::now();
      if (res)
      {
        opt_succ_num_++;
        Trajectory optimized_traj;
        optimizer_ptr_->getTraj(optimized_traj);
        std::vector<StatePVA> vis_x;
        vis_x.clear();
        optimized_traj.sampleWholeTrajectory(&vis_x);
        vis_ptr_->visualizeStates(vis_x, FinalTraj, pos_checker_ptr_->getLocalTime());
        krrt_planner_ptr_->evaluateTraj(optimized_traj, traj_duration, traj_len, traj_seg_nums, traj_acc_itg, traj_jerk_itg);
        o_file_ << (optimize_end_time - optimize_start_time).toSec() * 1e3 << "," 
                << traj_seg_nums << "," 
                << traj_acc_itg << "," 
                << traj_jerk_itg << "," 
                << traj_duration << ","
                << traj_len << "," << ","; 
      }
      else
      {
        o_file_ << "," 
                << "," 
                << "," 
                << "," 
                << "," 
                << "," << ","; 
      }

      optimize_start_time = ros::Time::now();
      bool res_old_opt_method = optimize_old(krrt_first_traj);
      optimize_end_time = ros::Time::now();
      if (res_old_opt_method)
      {
        opt_old_succ_num_++;
        Trajectory optimized_traj;
        optimizer_ptr_->getTraj(optimized_traj);
        std::vector<StatePVA> vis_x;
        vis_x.clear();
        optimized_traj.sampleWholeTrajectory(&vis_x);
        vis_ptr_->visualizeStates(vis_x, OptimizedTraj, pos_checker_ptr_->getLocalTime());
        krrt_planner_ptr_->evaluateTraj(optimized_traj, traj_duration, traj_len, traj_seg_nums, traj_acc_itg, traj_jerk_itg);
        o_file_ << (optimize_end_time - optimize_start_time).toSec() * 1e3 << "," 
                << traj_seg_nums << "," 
                << traj_acc_itg << "," 
                << traj_jerk_itg << "," 
                << traj_duration << ","
                << traj_len << "," << ","; 
      }
      else
      {
        o_file_ << "," 
                << "," 
                << "," 
                << "," 
                << "," 
                << "," << ","; 
      }

      optimize_start_time = ros::Time::now();
      bool res_liu_opt_method = optimize_liu(krrt_first_traj);
      optimize_end_time = ros::Time::now();
      if (res_liu_opt_method)
      {
        opt_old_succ_num_++;
        Trajectory optimized_traj;
        optimizer_ptr_->getTraj(optimized_traj);
        std::vector<StatePVA> vis_x;
        vis_x.clear();
        optimized_traj.sampleWholeTrajectory(&vis_x);
        vis_ptr_->visualizeStates(vis_x, BestTraj, pos_checker_ptr_->getLocalTime());
        krrt_planner_ptr_->evaluateTraj(optimized_traj, traj_duration, traj_len, traj_seg_nums, traj_acc_itg, traj_jerk_itg);
        o_file_ << (optimize_end_time - optimize_start_time).toSec() * 1e3 << "," 
                << traj_seg_nums << "," 
                << traj_acc_itg << "," 
                << traj_jerk_itg << "," 
                << traj_duration << ","
                << traj_len << "," << ","; 
      }
      else
      {
        o_file_ << "," 
                << "," 
                << "," 
                << "," 
                << "," 
                << "," << ","; 
      }

    }
  }
  else
  {
    o_file_ << "," 
            << "," 
            << "," 
            << "," 
            << "," 
            << "," << ","; 
    
    o_file_ << "," 
                << "," 
                << "," 
                << "," 
                << "," 
                << "," << ","; 

    o_file_ << "," 
                << "," 
                << "," 
                << "," 
                << "," 
                << "," << ","; 
    
    o_file_ << "," 
                << "," 
                << "," 
                << "," 
                << "," 
                << "," << ","; 

    o_file_ << "," 
                << "," 
                << "," 
                << "," 
                << "," 
                << "," << ","; 
  }
  }
  
  bool rrt_wo_res(false);
  // while (rrt_wo_res != KRRTPlanner::SUCCESS)
  if (test_rrt_wo_)
  {
  // w/o regional opt
  krrt_planner_ptr_->reset();
  ros::Time t_krrt_without_s = ros::Time::now();
  rrt_wo_res = krrt_planner_ptr_->plan(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, search_time, Vector3d(0,0,0), Vector3d(0,0,0), 0, 0);
  double time_krrt_without = (ros::Time::now() - t_krrt_without_s).toSec();
  if (rrt_wo_res == KRRTPlanner::SUCCESS)
  {
    rrt_wo_suss_num_++;
    Trajectory krrt_final_traj, krrt_first_traj;
    krrt_planner_ptr_->getFirstTraj(krrt_first_traj);
    krrt_planner_ptr_->getTraj(krrt_final_traj);
    std::vector<StatePVA> vis_x;
    vis_x.clear();
    krrt_first_traj.sampleWholeTrajectory(&vis_x);
    vis_ptr_->visualizeStates(vis_x, TrackedTraj, pos_checker_ptr_->getLocalTime());
    // vis_x.clear();
    // krrt_final_traj.sampleWholeTrajectory(&vis_x);
    // vis_ptr_->visualizeStates(vis_x, FinalTraj, pos_checker_ptr_->getLocalTime());
    
    double first_traj_len(0.0), first_traj_duration(0.0), first_traj_acc_itg(0.0), first_traj_jerk_itg(0.0);
    double final_traj_len(0.0), final_traj_duration(0.0), final_traj_acc_itg(0.0), final_traj_jerk_itg(0.0);
    int first_traj_seg_nums(0), final_traj_seg_nums(0);
    krrt_planner_ptr_->evaluateTraj(krrt_first_traj, first_traj_duration, first_traj_len, first_traj_seg_nums, first_traj_acc_itg, first_traj_jerk_itg);
    krrt_planner_ptr_->evaluateTraj(krrt_final_traj, final_traj_duration, final_traj_len, final_traj_seg_nums, final_traj_acc_itg, final_traj_jerk_itg);
    double first_traj_use_time = krrt_planner_ptr_->getFirstTrajTimeUsage();
    double final_traj_use_time = krrt_planner_ptr_->getFinalTrajTimeUsage();
    cout << "==================== KRRT w/o ======================" << endl;
    ROS_INFO_STREAM("[KRRT w/o regional opt]: [front-end first path]: " << endl 
        << "    -   seg nums: " << first_traj_seg_nums << endl 
        << "    -   time: " << first_traj_use_time * 1e3 << " ms" << ", time + func_call: " << time_krrt_without * 1e3 << " ms" << endl 
        << "    -   acc integral: " << first_traj_acc_itg << endl
        << "    -   jerk integral: " << first_traj_jerk_itg << endl
        << "    -   traj duration: " << first_traj_duration << endl 
        << "    -   path length: " << first_traj_len << " m");
        
    ROS_INFO_STREAM("[KRRT]: [front-end final path]: " << endl 
        << "    -   seg nums: " << final_traj_seg_nums << endl 
        << "    -   time: " << final_traj_use_time * 1e3 << " ms"<< endl 
        << "    -   acc integral: " << final_traj_acc_itg << endl
        << "    -   jerk integral: " << final_traj_jerk_itg << endl
        << "    -   traj duration: " << final_traj_duration << endl 
        << "    -   path length: " << final_traj_len << " m");
    cout << "====================================================" << endl;
    o_file_ << first_traj_use_time * 1e3 << "," 
            << first_traj_seg_nums << "," 
            << first_traj_acc_itg << "," 
            << first_traj_jerk_itg << "," 
            << first_traj_duration << ","
            << first_traj_len << "," << ","; 

    o_file_ << final_traj_use_time * 1e3 << "," 
            << final_traj_seg_nums << "," 
            << final_traj_acc_itg << "," 
            << final_traj_jerk_itg << "," 
            << final_traj_duration << ","
            << final_traj_len << "," << ","; 
  }
  else
  {
    o_file_ << "," 
            << "," 
            << "," 
            << "," 
            << "," 
            << "," << ","; 
    
    o_file_ << "," 
                << "," 
                << "," 
                << "," 
                << "," 
                << "," << ","; 
  }
  }

  /* ---k a star --- */
  if (test_a_double_)
  {
  kastar_traj_finder_->reset();
  ros::Time t_ka_s = ros::Time::now();
  result = kastar_traj_finder_->search(start_pos, start_vel, start_acc, end_pos, end_vel, false, 0, -1.0);
  ros::Time t_ka_e = ros::Time::now();
  if (result == KinodynamicAstar::REACH_END)
  {
    a_double_succ_num_++;
    vector<StatePVA> vis_x;
    kastar_traj_finder_->getVisTraj(0.01, vis_x);
    vis_ptr_->visualizeStates(vis_x, OptimizedTraj, pos_checker_ptr_->getLocalTime());
    double traj_duration; double ctrl_cost; double traj_length; int seg_nums;
    kastar_traj_finder_->getTrajAttributes(traj_duration, ctrl_cost, traj_length, seg_nums);
    cout << "==================== A* Acc =======================------------------" << endl;
    ROS_WARN_STREAM("k a use: " << (t_ka_e - t_ka_s).toSec() * 1e3 << " ms");
    ROS_WARN_STREAM("k a star path found" <<
    "\n[kA*] traj_duration: " << traj_duration << 
    "\n[kA*] ctrl_cost: " << ctrl_cost <<
    "\n[kA*] traj_length: " << traj_length << 
    "\n[kA*] seg_nums: " << seg_nums);
    cout << "===================================================------------------" << endl;
    o_file_ << (t_ka_e - t_ka_s).toSec() * 1e3 << "," 
            << seg_nums << "," 
            << ctrl_cost << "," 
            << "none" << "," 
            << traj_duration << ","
            << traj_length << "," << ","; 
  }
  else
  {
    ROS_WARN("no path from k a star");
    o_file_ << "," 
            << ","  
            << ","  
            << ","  
            << "," 
            << "," << ","; 
  }
  }
  /* ---k a star --- */


  /* ----k a jerk star */
  if (test_a_triple_)
  {
  kastar_jerk_finder_->reset();
  ros::Time t_ka_j_s = ros::Time::now();
  result = kastar_jerk_finder_->search(start_pos, start_vel, start_acc, end_pos,end_vel, end_acc, true, 0, -1.0);
  ros::Time t_ka_j_e = ros::Time::now();
  if (result == KinodynamicAstarJ::REACH_END) 
  {
    a_tripla_succ_num_++;
    Trajectory a_star_jerk_traj;
    kastar_jerk_finder_->getTraj(a_star_jerk_traj);
    std::vector<StatePVA> vis_x;
    vis_x.clear();
    a_star_jerk_traj.sampleWholeTrajectory(&vis_x);
    vis_ptr_->visualizeStates(vis_x, BestTraj, pos_checker_ptr_->getLocalTime());
    krrt_planner_ptr_->evaluateTraj(a_star_jerk_traj, traj_duration, traj_len, traj_seg_nums, traj_acc_itg, traj_jerk_itg);
    o_file_ << (t_ka_j_e - t_ka_j_s).toSec() * 1e3 << "," 
            << traj_seg_nums << "," 
            << traj_acc_itg << "," 
            << traj_jerk_itg << "," 
            << traj_duration << ","
            << traj_len << "," << ","; 
    cout << "====================Jerk Input===========" << endl;
    ROS_WARN_STREAM(
      "k a jerk use: " << (t_ka_j_e - t_ka_j_s).toSec() * 1e3 << " ms");
    cout << "Trajectory duration: " << traj_duration << endl;
    cout << "Acc cost: " << traj_acc_itg << endl;
    cout << "Jerk cost: " << traj_jerk_itg << endl;
    cout << "Trajectory length: " << traj_len << endl;
    cout << "Segment numbers: " << traj_seg_nums << endl;
    cout << "=========================================" << endl;
  } 
  else 
  {
    ROS_WARN("no path from k a star-jerk");
    o_file_ << "," 
            << "," 
            << "," 
            << "," 
            << "," 
            << "," << ","; 
  }
  cout << "Kinodynamic a star jerk end" << endl;
  }
  /* ----k a jerk star */


  /* -------------------- fmt w/ -----------------*/ 
  bool fmt_w_res(false);
  if (test_fmt_w_)
  // while (fmt_w_res != KFMTPlanner::SUCCESS)
  {
  fmt_w_res = fmt_planner_ptr_->plan(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, search_time, Vector3d(0,0,0), Vector3d(0,0,0), 0, 1);
  if (fmt_w_res == KFMTPlanner::SUCCESS)
  {
    fmt_w_succ_num_++;
    Trajectory kfmt_final_traj;
    fmt_planner_ptr_->getTraj(kfmt_final_traj);
    std::vector<StatePVA> vis_x;
    vis_x.clear();
    kfmt_final_traj.sampleWholeTrajectory(&vis_x);
    vis_ptr_->visualizeStates(vis_x, FMTTraj, pos_checker_ptr_->getLocalTime());
    // vis_x.clear();
    // krrt_final_traj.sampleWholeTrajectory(&vis_x);
    // vis_ptr_->visualizeStates(vis_x, FinalTraj, pos_checker_ptr_->getLocalTime());
    
    double first_traj_len(0.0), first_traj_duration(0.0), first_traj_acc_itg(0.0), first_traj_jerk_itg(0.0);
    int first_traj_seg_nums(0);
    krrt_planner_ptr_->evaluateTraj(kfmt_final_traj, first_traj_duration, first_traj_len, first_traj_seg_nums, first_traj_acc_itg, first_traj_jerk_itg);
    double traj_use_time = fmt_planner_ptr_->getTimeUsage();
    cout << "==================== KFMT w/ ======================" << endl;
    ROS_INFO_STREAM("[KFMT w/ regional opt]: [front-end first path]: " << endl 
        << "    -   seg nums: " << first_traj_seg_nums << endl 
        << "    -   time: " << traj_use_time * 1e3 << " ms" << endl 
        << "    -   acc integral: " << first_traj_acc_itg << endl
        << "    -   jerk integral: " << first_traj_jerk_itg << endl
        << "    -   traj duration: " << first_traj_duration << endl 
        << "    -   path length: " << first_traj_len << " m");
        
    cout << "====================================================" << endl;
    o_file_ << traj_use_time * 1e3 << "," 
            << first_traj_seg_nums << "," 
            << first_traj_acc_itg << "," 
            << first_traj_jerk_itg << "," 
            << first_traj_duration << ","
            << first_traj_len << "," << ","; 
  }
  else
  {
    o_file_ << "," 
            << "," 
            << "," 
            << "," 
            << "," 
            << "," << ","; 
  }
  }
  /* -------------------- fmt w/ -----------------*/ 


  /* -------------------- fmt w/o -----------------*/ 
  bool fmt_wo_res(false);
  if (test_fmt_wo_)
  // while (fmt_wo_res != KFMTPlanner::SUCCESS)
  {
  fmt_wo_res = fmt_planner_ptr_->plan(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, search_time, Vector3d(0,0,0), Vector3d(0,0,0), 0, 0);
  if (fmt_wo_res == KFMTPlanner::SUCCESS)
  {
    fmt_wo_succ_num_++;
    Trajectory kfmt_final_traj;
    fmt_planner_ptr_->getTraj(kfmt_final_traj);
    std::vector<StatePVA> vis_x;
    vis_x.clear();
    kfmt_final_traj.sampleWholeTrajectory(&vis_x);
    vis_ptr_->visualizeStates(vis_x, FMTTrajWithout, pos_checker_ptr_->getLocalTime());
    // vis_x.clear();
    // krrt_final_traj.sampleWholeTrajectory(&vis_x);
    // vis_ptr_->visualizeStates(vis_x, FinalTraj, pos_checker_ptr_->getLocalTime());
    
    double first_traj_len(0.0), first_traj_duration(0.0), first_traj_acc_itg(0.0), first_traj_jerk_itg(0.0);
    int first_traj_seg_nums(0);
    krrt_planner_ptr_->evaluateTraj(kfmt_final_traj, first_traj_duration, first_traj_len, first_traj_seg_nums, first_traj_acc_itg, first_traj_jerk_itg);
    double traj_use_time = fmt_planner_ptr_->getTimeUsage();
    cout << "==================== KFMT w/o ======================" << endl;
    ROS_INFO_STREAM("[KFMT w/o regional opt]: [front-end first path]: " << endl 
        << "    -   seg nums: " << first_traj_seg_nums << endl 
        << "    -   time: " << traj_use_time * 1e3 << " ms" << endl 
        << "    -   acc integral: " << first_traj_acc_itg << endl
        << "    -   jerk integral: " << first_traj_jerk_itg << endl
        << "    -   traj duration: " << first_traj_duration << endl 
        << "    -   path length: " << first_traj_len << " m");
        
    cout << "====================================================" << endl;
    o_file_ << traj_use_time * 1e3 << "," 
            << first_traj_seg_nums << "," 
            << first_traj_acc_itg << "," 
            << first_traj_jerk_itg << "," 
            << first_traj_duration << ","
            << first_traj_len << "," << ","; 
  }
  else
  {
    o_file_ << "," 
            << "," 
            << "," 
            << "," 
            << "," 
            << "," << ","; 
  }
  }
  /* -------------------- fmt w/o -----------------*/ 


  o_file_ << endl;
  
  return tried_once;
}

bool PlannerTester::optimize(const Trajectory &traj)
{
  if (!optimizer_ptr_->initialize(traj, TrajOptimizer::SMOOTH_HOMO_OBS))
    return false;
  bool res = optimizer_ptr_->solve_S_H_O();
  return res;
}

bool PlannerTester::optimize_old(const Trajectory &traj)
{
  if (!optimizer_ptr_->initialize(traj, TrajOptimizer::SMOOTH_HOMO))
    return false;
  bool res = optimizer_ptr_->solve_S_H();
  return res;
}

bool PlannerTester::optimize_liu(const Trajectory &traj)
{
  if (!optimizer_ptr_->initialize(traj, TrajOptimizer::SMOOTH))
    return false;
  bool res = optimizer_ptr_->solve_S();
  return res;
}

bool PlannerTester::bSplineOpt(const Trajectory& front_traj, fast_planner::NonUniformBspline& pos)
{
  // parameterize the path to bspline
  vector<Eigen::Vector3d> point_set, start_end_derivatives;
  double front_traj_dura = front_traj.getTotalDuration();
  double ts = ctrl_pt_dist_ / max_vel_;
  int K = floor(front_traj_dura / ts);
  ts = front_traj_dura / double(K + 1);
  for (double t = 0.0; t < front_traj_dura; t += ts) 
  {
      Eigen::Vector3d pos, vel, acc;
      pos = front_traj.getPos(t);
      point_set.push_back(pos);
  }
  start_end_derivatives.push_back(front_traj.getVel(0.0));
  start_end_derivatives.push_back(front_traj.getVel(front_traj_dura));
  start_end_derivatives.push_back(front_traj.getAcc(0.0));
  start_end_derivatives.push_back(front_traj.getAcc(front_traj_dura));

  Eigen::MatrixXd ctrl_pts;
  fast_planner::NonUniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
  fast_planner::NonUniformBspline init(ctrl_pts, 3, ts);

  // bspline trajectory optimization
  int cost_function = fast_planner::BsplineOptimizer::NORMAL_PHASE;
  ctrl_pts = bspline_optimizer_->BsplineOptimizeTraj(ctrl_pts, ts, cost_function, 1, 1);

  // iterative time adjustment
  pos = fast_planner::NonUniformBspline(ctrl_pts, 3, ts);
  double to = pos.getTimeSum();
  pos.setPhysicalLimits(max_vel_, max_acc_);
  bool feasible = pos.checkFeasibility(false);

  int iter_num = 0;
  while (!feasible && ros::ok()) 
  {
    if (++iter_num >= 3) 
      return false;
    feasible = pos.reallocateTime();
  }
  double tn = pos.getTimeSum();
  if (tn / to > 3.0) 
    return false;
  else 
    return true;
}

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "state_machine_node");
  ros::NodeHandle nh("~");
  
  tgk_planner::PlannerTester planners_tester;
  planners_tester.init(nh);
  
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();
}


#endif //_FSM_H_
