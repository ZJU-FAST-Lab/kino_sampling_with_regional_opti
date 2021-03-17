#include "state_machine/fsm.h"
#include <ros/console.h>

namespace tgk_planner
{
  FSM::FSM()
  {
  }

  FSM::~FSM()
  {
  }

  void FSM::init(const ros::NodeHandle &nh)
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

    qrcode_pose_sub_ = nh_.subscribe("/qrcode_detector/qrcode_position", 1, &FSM::qrcodeCallback, this);
    goal_sub_ = nh_.subscribe("/goal", 1, &FSM::goalCallback, this);
    traj_pub_ = nh_.advertise<quadrotor_msgs::PolynomialTrajectory>("planning/poly_traj", 10);
    execution_timer_ = nh_.createTimer(ros::Duration(0.01), &FSM::executionCallback, this); // 100Hz
    track_err_trig_sub_ = nh_.subscribe("/trig/tracking_err", 1, &FSM::trackErrCallback, this);
    rcv_glb_obs_client_ = nh_.serviceClient<self_msgs_and_srvs::GlbObsRcv>("/pub_glb_obs");

    nh.param("fsm/use_optimization", use_optimization_, false);
    nh.param("fsm/replan", replan_, false);
    nh.param("fsm/replan_time", replan_time_, 0.02);
    nh.param("fsm/allow_track_err_replan", allow_track_err_replan_, false);
    nh.param("fsm/e_stop_time_margin", e_stop_time_margin_, 1.0);
    nh.param("fsm/replan_check_duration", replan_check_duration_, 1.0);
    nh.param("fsm/bidirection", bidirection_, false);
    ROS_WARN_STREAM("[fsm] param: use_optimization: " << use_optimization_);
    ROS_WARN_STREAM("[fsm] param: replan: " << replan_);
    ROS_WARN_STREAM("[fsm] param: replan_time: " << replan_time_);
    ROS_WARN_STREAM("[fsm] param: allow_track_err_replan: " << allow_track_err_replan_);
    ROS_WARN_STREAM("[fsm] param: e_stop_time_margin: " << e_stop_time_margin_);
    ROS_WARN_STREAM("[fsm] param: replan_check_duration: " << replan_check_duration_);
    ROS_WARN_STREAM("[fsm] param: bidirection: " << bidirection_);

    track_err_replan_ = false;
    new_goal_ = false;
    started_ = false;
    last_goal_pos_ << 0.0, 0.0, 0.0;
    machine_state_ = INIT;
    cuur_traj_start_time_ = ros::Time::now();
    pos_about_to_collide_ << 0.0, 0.0, 0.0;
    remain_safe_time_ = 0.0;
  }

  void FSM::trackErrCallback(const std_msgs::Empty &msg)
  {
    if (allow_track_err_replan_)
      track_err_replan_ = true;
  }

  void FSM::qrcodeCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
  {
    Vector3d pos;
    pos << msg->point.x,
        msg->point.y,
        msg->point.z;
    if ((last_goal_pos_ - pos).norm() >= 2.0)
    {
      end_pos_ = pos;
      end_vel_.setZero();
      end_acc_.setZero();
      started_ = true;
      new_goal_ = true;
      last_goal_pos_ = end_pos_;
    }
  }

  void FSM::goalCallback(const quadrotor_msgs::PositionCommand::ConstPtr &goal_msg)
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
    last_goal_pos_ = end_pos_;
  }

  void FSM::executionCallback(const ros::TimerEvent &event)
  {
    static ros::Time start_follow_time, collision_detect_time, last_replan_start_time;
    static int replan_state = 1;
    static int fsm_num = 0;
    static StatePVA last_replan_start_state;
    fsm_num++;
    if (fsm_num == 100)
    {
      // printState();
      if (!env_ptr_->odomValid())
      {
        ROS_INFO("no odom.");
      }
      if (!env_ptr_->mapValid())
      {
        ROS_INFO("no map.");
        self_msgs_and_srvs::GlbObsRcv srv;
        if (!rcv_glb_obs_client_.call(srv))
          ROS_WARN("Failed to call service /pub_glb_obs");
      }
      if (!started_)
      {
        ROS_INFO("wait for goal in %lf but actual in %lf", event.current_expected.toSec(), event.current_real.toSec());
      }
      fsm_num = 0;
    }

    switch (machine_state_)
    {
    case INIT:
    {
      if (!env_ptr_->odomValid())
      {
        return;
      }
      if (!env_ptr_->mapValid())
      {
        return;
      }
      if (!started_)
      {
        return;
      }
      changeState(WAIT_GOAL);
      break;
    }

    case WAIT_GOAL:
    {
      if (!new_goal_)
      {
        return;
      }
      else
      {
        new_goal_ = false;
        changeState(GENERATE_TRAJ);
      }
      break;
    }

    case GENERATE_TRAJ:
    {
      start_pos_ = env_ptr_->get_curr_posi();
      start_vel_ = env_ptr_->get_curr_twist();
      //numerical problem
      if (start_vel_.norm() < 0.05)
      {
        start_vel_(0) = 0.0;
        start_vel_(1) = 0.0;
        start_vel_(2) = 0.0;
      }
      start_acc_.setZero();

      Vector3d normal, dire; //unused
      bool success = searchForTraj(start_pos_, start_vel_, start_acc_, end_pos_, end_vel_, end_acc_, replan_time_, normal, dire, false); //TODO what if it can not finish in 10ms?
      if (success)
      {
        Trajectory krrt_first_traj;
        krrt_planner_ptr_->getFirstTraj(krrt_first_traj);
        double traj_len(0.0), traj_duration(0.0), traj_acc_itg(0.0), traj_jerk_itg(0.0);
        int first_traj_seg_nums(0);
        krrt_planner_ptr_->evaluateTraj(krrt_first_traj, traj_duration, traj_len, first_traj_seg_nums, traj_acc_itg, traj_jerk_itg);
        double first_traj_use_time = krrt_planner_ptr_->getFirstTrajTimeUsage();
        double final_traj_use_time = krrt_planner_ptr_->getFinalTrajTimeUsage();
        ROS_INFO_STREAM("[KRRT]: [front-end first path]: " << endl 
            << "    -   seg nums: " << first_traj_seg_nums << endl 
            << "    -   time: " << first_traj_use_time * 1e3 << " ms" << endl 
            << "    -   acc integral: " << traj_acc_itg << endl
            << "    -   jerk integral: " << traj_jerk_itg << endl
            << "    -   traj duration: " << traj_duration << endl 
            << "    -   path length: " << traj_len << " m");
        vector<string> ss;
        vector<Vector3d> ps;
        ss.push_back(std::to_string(first_traj_use_time * 1e3));
        ss.push_back(std::to_string(traj_jerk_itg));


        krrt_planner_ptr_->getTraj(traj_);
        vector<StatePVA> vis_x;
        vis_x.clear();
        traj_.sampleWholeTrajectory(&vis_x);
        vis_ptr_->visualizeStates(vis_x, FinalTraj, pos_checker_ptr_->getLocalTime());


        bool rrt_wo_res(false);
        krrt_planner_ptr_->reset();
        rrt_wo_res = krrt_planner_ptr_->plan(start_pos_, start_vel_, start_acc_, end_pos_, end_vel_, end_acc_, replan_time_, Vector3d(0,0,0), Vector3d(0,0,0), 0, 0);
        std::string str_wo;
        if (rrt_wo_res == KRRTPlanner::SUCCESS)
        {
          Trajectory krrt_traj_wo;
          krrt_planner_ptr_->getFirstTraj(krrt_traj_wo);
          krrt_planner_ptr_->evaluateTraj(krrt_traj_wo, traj_duration, traj_len, first_traj_seg_nums, traj_acc_itg, traj_jerk_itg);
          double first_traj_use_time = krrt_planner_ptr_->getFirstTrajTimeUsage();
          ss.push_back(std::to_string(first_traj_use_time * 1e3));
          ss.push_back(std::to_string(traj_jerk_itg));
          vis_x.clear();
          krrt_traj_wo.sampleWholeTrajectory(&vis_x);
          vis_ptr_->visualizeStates(vis_x, FMTTrajWithout, pos_checker_ptr_->getLocalTime());
        }
        else
        {
          ss.push_back("fail");
          ss.push_back("fail");
          vis_x.clear();
          vis_ptr_->visualizeStates(vis_x, FMTTrajWithout, pos_checker_ptr_->getLocalTime());
        }

        if (use_optimization_)
        {
          vis_x.clear();
          ros::Time optimize_start_time = ros::Time::now();
          bool optimize_succ = optimize();
          ros::Time optimize_end_time = ros::Time::now();
          if (optimize_succ)
          {
            optimizer_ptr_->getTraj(traj_);
            traj_.sampleWholeTrajectory(&vis_x);

            krrt_planner_ptr_->evaluateTraj(traj_, traj_duration, traj_len, first_traj_seg_nums, traj_acc_itg, traj_jerk_itg);
            ss.push_back(std::to_string((optimize_end_time - optimize_start_time).toSec() * 1e3));
            ss.push_back(std::to_string(traj_jerk_itg));
          }
          else
          {
            ss.push_back("fail");
            ss.push_back("fail");
          }
          vis_ptr_->visualizeStates(vis_x, OptimizedTraj, pos_checker_ptr_->getLocalTime());
        }
        vis_ptr_->visualizeText(ss, ps, pos_checker_ptr_->getLocalTime());
        
        replan_state = 1;
        sendTrajToServer(traj_);
        cuur_traj_start_time_ = ros::Time::now();
        new_goal_ = false;
        start_follow_time = ros::Time::now();
        changeState(FOLLOW_TRAJ);
      }
      else
      {
      new_goal_ = false;
      changeState(WAIT_GOAL);
      }
      break;
    }

    case FOLLOW_TRAJ:
    {
      double t_during_traj = (ros::Time::now() - cuur_traj_start_time_).toSec();
      if (reachGoal(0.5))
      {
        changeState(WAIT_GOAL);
      }
      else if (new_goal_)
      {
        ROS_WARN("Replan because of new goal received");
        new_goal_ = false;
        remain_safe_time_ = traj_.getTotalDuration() - t_during_traj;
        // ROS_INFO("t_during_traj: %lf", t_during_traj);
        // ROS_INFO("remain_safe_time: %lf", remain_safe_time_);
        collision_detect_time = ros::Time::now();
        changeState(REPLAN_TRAJ);
      }
      else if (replan_)
      {
        //replan because remaining traj may collide
        if (needReplan())
        {
          ROS_WARN("REPLAN because of future collision");
          collision_detect_time = ros::Time::now();
          changeState(REPLAN_TRAJ);
        }
        else if (track_err_replan_)
        {
          track_err_replan_ = false;
          ROS_WARN("REPLAN because of not tracking closely");
          remain_safe_time_ = traj_.getTotalDuration() - t_during_traj;
          replan_state = 4;
          collision_detect_time = ros::Time::now();
          changeState(REPLAN_TRAJ);
        }
        else if (close_goal_traj_ && (traj_.getTotalDuration() - t_during_traj) < 2)
        {
          close_goal_traj_ = false;
          remain_safe_time_ = traj_.getTotalDuration() - t_during_traj;
          collision_detect_time = ros::Time::now();
          ROS_WARN("REPLAN cause t_remain is run out");
          changeState(REPLAN_TRAJ);
        }
      }
      //       else if ((ros::Time::now() - start_follow_time).toSec() > 2)
      //       {//replan because
      //         changeState(REFINE_REMAINING_TRAJ);
      //       }
      break;
    }

    case REPLAN_TRAJ:
    {
      ros::Time t_replan_start = ros::Time::now();
      //replan once
      double curr_remain_safe_time = remain_safe_time_ - (ros::Time::now() - collision_detect_time).toSec();
      double dt = max(0.0, min(replan_time_, curr_remain_safe_time));
      //start state is in dt (replan_front_time + optimization_time) second from current traj state
      double t_during_traj = (ros::Time::now() - cuur_traj_start_time_).toSec();
      Eigen::VectorXd start_state;
      bool need_consistancy(false);
      Eigen::Vector3d normal, replan_direction;
      if (replan_state == 1)
      {
        // ROS_INFO_STREAM("state 1, replan from tracking traj");
        start_state = getReplanStateFromPath(t_during_traj + dt, traj_); //start in future state
        // for replan consistancy consideration, not used currently
        // if ((start_state.head(3) - last_replan_start_state.head(3)).norm() <= 1)
        // {
        //   Eigen::Vector3d curr_dire = start_state.segment(3, 3);
        //   Eigen::Vector3d last_dire = last_replan_start_state.segment(3, 3);
        //   normal = (curr_dire.cross(last_dire)).normalized();
        //   replan_direction = (curr_dire.cross(normal)).normalized();
        //   need_consistancy = true;
        // }
      }
      else if (replan_state == 4)
      {
        // ROS_INFO_STREAM("state 4, replan from curr state");
        start_state = getReplanStateFromPath(-1.0, traj_); //start in curr state
      }
      ROS_WARN_STREAM("need_consistancy: " << need_consistancy);
      Eigen::Vector3d start_pos, start_vel, start_acc;
      start_pos = start_state.segment(0, 3);
      start_vel = start_state.segment(3, 3);
      start_acc = start_state.segment(6, 3);
      if (start_vel.norm() < 1e-4)
        start_vel = Vector3d(0.0, 0.0, 0.0);
      if (start_acc.norm() < 1e-4)
        start_acc = Vector3d(0.0, 0.0, 0.0);
      double front_time = dt;
      if (dt <= 0.005)
        front_time = replan_time_;
      last_replan_start_state = start_state;
      bool success = searchForTraj(start_pos, start_vel, start_acc, end_pos_, end_vel_, end_acc_, front_time, normal, replan_direction, need_consistancy);
      //found a traj towards goal
      if (success)
      {
        Trajectory krrt_first_traj;
        krrt_planner_ptr_->getFirstTraj(krrt_first_traj);
        double traj_len(0.0), traj_duration(0.0), traj_acc_itg(0.0), traj_jerk_itg(0.0);
        int first_traj_seg_nums(0);
        krrt_planner_ptr_->evaluateTraj(krrt_first_traj, traj_duration, traj_len, first_traj_seg_nums, traj_acc_itg, traj_jerk_itg);
        double first_traj_use_time = krrt_planner_ptr_->getFirstTrajTimeUsage();
        double final_traj_use_time = krrt_planner_ptr_->getFinalTrajTimeUsage();
        ROS_INFO_STREAM("[KRRT]: [front-end first path]: " << endl 
            << "    -   seg nums: " << first_traj_seg_nums << endl 
            << "    -   time: " << first_traj_use_time * 1e3 << " ms" << endl 
            << "    -   acc integral: " << traj_acc_itg << endl
            << "    -   jerk integral: " << traj_jerk_itg << endl
            << "    -   traj duration: " << traj_duration << endl 
            << "    -   path length: " << traj_len << " m");
        vector<string> ss;
        vector<Vector3d> ps;
        ss.push_back(std::to_string(first_traj_use_time * 1e3));
        ss.push_back(std::to_string(traj_jerk_itg));

        krrt_planner_ptr_->getTraj(traj_);
        vector<StatePVA> vis_x;
        vis_x.clear();
        traj_.sampleWholeTrajectory(&vis_x);
        vis_ptr_->visualizeStates(vis_x, FinalTraj, pos_checker_ptr_->getLocalTime());




        bool rrt_wo_res(false);
        krrt_planner_ptr_->reset();
        rrt_wo_res = krrt_planner_ptr_->plan(start_pos, start_vel, start_acc, end_pos_, end_vel_, end_acc_, front_time, Vector3d(0,0,0), Vector3d(0,0,0), 0, 0);
        std::string str_wo;
        if (rrt_wo_res == KRRTPlanner::SUCCESS)
        {
          Trajectory krrt_traj_wo;
          krrt_planner_ptr_->getFirstTraj(krrt_traj_wo);
          krrt_planner_ptr_->evaluateTraj(krrt_traj_wo, traj_duration, traj_len, first_traj_seg_nums, traj_acc_itg, traj_jerk_itg);
          double first_traj_use_time = krrt_planner_ptr_->getFirstTrajTimeUsage();
          ss.push_back(std::to_string(first_traj_use_time * 1e3));
          ss.push_back(std::to_string(traj_jerk_itg));
          vis_x.clear();
          krrt_traj_wo.sampleWholeTrajectory(&vis_x);
          vis_ptr_->visualizeStates(vis_x, FMTTrajWithout, pos_checker_ptr_->getLocalTime());
        }
        else
        {
          ss.push_back("fail");
          ss.push_back("fail");
          vis_x.clear();
          vis_ptr_->visualizeStates(vis_x, FMTTrajWithout, pos_checker_ptr_->getLocalTime());
        }
        



        ROS_WARN("Replan front-end success");
        if (use_optimization_)
        {
          vis_x.clear();
          ros::Time optimize_start_time = ros::Time::now();
          bool optimize_succ = optimize();
          ros::Time optimize_end_time = ros::Time::now();
          if (optimize_succ)
          {
            ROS_WARN("Replan back-end success");
            optimizer_ptr_->getTraj(traj_);
            traj_.sampleWholeTrajectory(&vis_x);
            
            krrt_planner_ptr_->evaluateTraj(traj_, traj_duration, traj_len, first_traj_seg_nums, traj_acc_itg, traj_jerk_itg);
            ss.push_back(std::to_string((optimize_end_time - optimize_start_time).toSec() * 1e3));
            ss.push_back(std::to_string(traj_jerk_itg));
          }
          else
          {
            ss.push_back("fail");
            ss.push_back("fail");
          }
          vis_ptr_->visualizeStates(vis_x, OptimizedTraj, pos_checker_ptr_->getLocalTime());
        }
        vis_ptr_->visualizeText(ss, ps, pos_checker_ptr_->getLocalTime());

        double replan_duration = (ros::Time::now() - t_replan_start).toSec();
        if (replan_duration < dt)
        {
          ROS_WARN("wait for it: %lf", dt - replan_duration);
          ros::Duration(dt - replan_duration).sleep();
        }
        replan_state = 1;
        sendTrajToServer(traj_);
        cuur_traj_start_time_ = ros::Time::now();
        new_goal_ = false;
        start_follow_time = ros::Time::now();
        changeState(FOLLOW_TRAJ);
      }
      else
      {
        double curr_remain_safe_time = remain_safe_time_ - (ros::Time::now() - collision_detect_time).toSec();
        ROS_ERROR("Replan fail, %lf seconds to collide", curr_remain_safe_time);
        if (curr_remain_safe_time < e_stop_time_margin_)
        {
          sendEStopToServer();
          ROS_ERROR("ABOUT TO CRASH!! SERVER EMERGENCY STOP!!");
          changeState(GENERATE_TRAJ);
        }
        else
        {
          ROS_WARN("keep replanning");
        }
        break;
      }
      if (needReplan())
      {
        ROS_WARN("update future collision info");
        collision_detect_time = ros::Time::now();
      }
      else
      {
        ROS_WARN("future collision unchanged");
      }
      break;
    }

    case EMERGENCY_TRAJ:
    {

      break;
    }

    default:
      break;
    }
  }

  bool FSM::searchForTraj(Vector3d start_pos, Vector3d start_vel, Vector3d start_acc,
                          Vector3d end_pos, Vector3d end_vel, Vector3d end_acc,
                          double search_time, const Vector3d &normal, const Vector3d &dire, bool need_consistancy)
  {
    vis_ptr_->visualizeStartAndGoal(start_pos, end_pos, pos_checker_ptr_->getLocalTime());
    int result(false);

    /* r3planner  If uncomment, then use the resulting path to guide the sampling */
    // double len_cost(0.0);
    // vector<Vector3d> route;
    // vector<vector<Vector3d>> routes;
    // ros::Time t1 = ros::Time::now();
    // double radius(16); //radius square
    // if (r3_planer_ptr_->planOnce(start_pos, end_pos, route, len_cost, radius))
    // {
    //   ROS_ERROR_STREAM("r3 plan solved in: " << (ros::Time::now() - t1).toSec() * 1e3 << " ms, route len: " << len_cost);
    //   // vector<vector<Eigen::Vector3d>> select_paths;
    //   // size_t v_n = r3_planer_ptr_->getGraph(select_paths);
    //   // vis_ptr_->visualizePRM(select_paths, Color::Teal(), env_ptr_->getLocalTime());
    //   // getchar();
    //   routes.push_back(route);
    //   vis_ptr_->visualizePRM(routes, Color::Red(), env_ptr_->getLocalTime());

    //   // vector<double> radii;
    //   // for (const auto & p: route)
    //   // {
    //   //   Vector3d obs;
    //   //   double radius = sqrt(pos_checker_ptr_->nearestObs(p, obs));
    //   //   radii.push_back(radius);
    //   // }
    //   // vis_ptr_->visualizeBalls(route, radii, env_ptr_->getLocalTime());
    //   krrt_planner_ptr_->sampler_.topoSetup(routes);
    //   fmt_planner_ptr_->sampler_.topoSetup(routes);
    // }
    // else
    // {
    //   ROS_WARN("r3 plan fail");
    // }
    /* r3planner   */


    krrt_planner_ptr_->reset();
    result = krrt_planner_ptr_->plan(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, search_time, normal, dire, need_consistancy, 1);

    if (result == KRRTPlanner::SUCCESS)
    {
      close_goal_traj_ = false;
      return true;
    }
    else if (result == KRRTPlanner::SUCCESS_CLOSE_GOAL)
    {
      close_goal_traj_ = true;
      return true;
    }
    else
      return false;
  }

  bool FSM::optimize()
  {
    if (!optimizer_ptr_->initialize(traj_, TrajOptimizer::SMOOTH_HOMO_OBS))
      return false;
    bool res = optimizer_ptr_->solve_S_H_O();
    return res;
  }

  void FSM::sendTrajToServer(const Trajectory &poly_traj)
  {
    static int traj_id = 0;
    int path_seg_num = poly_traj.getPieceNum();
    if (path_seg_num < 1)
      return;
    quadrotor_msgs::PolynomialTrajectory traj;
    traj.trajectory_id = ++traj_id;
    traj.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ADD;
    traj.num_segment = path_seg_num;
    traj.time = poly_traj.getDurations();

    for (int i = 0; i < path_seg_num; ++i)
    {
      traj.order.push_back(5);
      for (size_t j = 0; j <= traj.order[i]; ++j)
      {
        CoefficientMat posCoeffsMat = poly_traj[i].getCoeffMat();
        traj.coef_x.push_back(posCoeffsMat(0, j));
        traj.coef_y.push_back(posCoeffsMat(1, j));
        traj.coef_z.push_back(posCoeffsMat(2, j));
      }
    }

    traj.header.frame_id = "map";
    traj.header.stamp = ros::Time::now();
    traj_pub_.publish(traj);
  }

  void FSM::sendEStopToServer()
  {
    quadrotor_msgs::PolynomialTrajectory traj;
    traj.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ABORT;

    traj.header.frame_id = "map";
    traj.header.stamp = ros::Time::now();
    traj_pub_.publish(traj);
  }

  bool FSM::reachGoal(double radius)
  {
    Eigen::Vector3d pos_now = env_ptr_->get_curr_posi();
    if ((end_pos_ - pos_now).norm() < radius)
      return true;
    else
      return false;
  }

  inline bool FSM::needReplan()
  {
    double t_during_traj = (ros::Time::now() - cuur_traj_start_time_).toSec();
    double t_check_until_traj = std::min(traj_.getTotalDuration(), t_during_traj + replan_check_duration_);
    if (!pos_checker_ptr_->checkPolyTraj(traj_, t_during_traj, t_check_until_traj, pos_about_to_collide_, remain_safe_time_))
    {
      ROS_INFO_STREAM("about to collide pos: " << pos_about_to_collide_.transpose() << ", remain safe time: " << remain_safe_time_);
      vis_ptr_->visualizeCollision(pos_about_to_collide_, pos_checker_ptr_->getLocalTime());
      return true;
    }
    return false;
  }

  Eigen::VectorXd FSM::getReplanStateFromPath(double t, const Trajectory &poly_traj)
  {
    Eigen::Vector3d pos(0.0, 0.0, 0.0), vel(0.0, 0.0, 0.0), acc(0.0, 0.0, 0.0);
    Eigen::VectorXd start_state(9);
    if (t < 0)
    {
      ROS_ERROR("not tracking well! use curr state as start state");
      pos = env_ptr_->get_curr_posi();
      vel = env_ptr_->get_curr_twist();
      acc = env_ptr_->get_curr_acc();
    }
    else
    {
      t = std::min(t, poly_traj.getTotalDuration());
      pos = poly_traj.getPos(t);
      vel = poly_traj.getVel(t);
      acc = poly_traj.getAcc(t);
    }
    //TODO what if pos is not free??
    start_state << pos[0], pos[1], pos[2], vel[0], vel[1], vel[2], acc[0], acc[1], acc[2];
    return start_state;
  }

  void FSM::printState()
  {
    string state_str[6] = {"INIT", "WAIT_GOAL", "GENERATE_TRAJ", "FOLLOW_TRAJ", "REPLAN_TRAJ", "EMERGENCY_TRAJ"};
    ROS_INFO_STREAM("[FSM]: state: " << state_str[int(machine_state_)]);
  }

  void FSM::changeState(FSM::MACHINE_STATE new_state)
  {
    string state_str[6] = {"INIT", "WAIT_GOAL", "GENERATE_TRAJ", "FOLLOW_TRAJ", "REPLAN_TRAJ", "EMERGENCY_TRAJ"};
    ROS_INFO_STREAM("[FSM]: change from " << state_str[int(machine_state_)] << " to " << state_str[int(new_state)]);
    machine_state_ = new_state;
  }

} // namespace tgk_planner
