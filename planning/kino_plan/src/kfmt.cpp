#include "kino_plan/kfmt.h"
#include <queue>

namespace tgk_planner
{
KFMTPlanner::KFMTPlanner(const ros::NodeHandle& nh): sampler_(nh)
{
}

KFMTPlanner::~KFMTPlanner()
{
  for (int i = 0; i < preallocate_node_pool_num_; i++) {
    delete node_pool_[i];
  }
  for (int i = 0; i < preallocate_regional_pool_num_; i++) {
    delete regional_node_pool_[i];
  }
}

void KFMTPlanner::init(const ros::NodeHandle& nh)
{
  nh.param("kfmt/vel_limit", vel_limit_, -1.0);
  nh.param("kfmt/acc_limit", acc_limit_, -1.0);
  nh.param("kfmt/jerk_limit", jerk_limit_, -1.0);
  nh.param("kfmt/debug_vis", debug_vis_, false);
  nh.param("kfmt/rho", rho_, -1.0);
  nh.param("kfmt/preallocate_node_pool_num", preallocate_node_pool_num_, 0);
  nh.param("kfmt/preallocate_regional_node_pool_num", preallocate_regional_pool_num_, 0);
  nh.param("kfmt/sample_num_per_meter", sample_num_per_meter_, 0);
  nh.param("kfmt/radius_cost_between_two_states", radius_cost_between_two_states_, 0.0);
  nh.param("kfmt/allow_close_goal", allow_close_goal_, false);
  nh.param("kfmt/lambda_heu", lambda_heu_, -1.0);
  nh.param("kfmt/tie_breaker", tie_breaker_, -1.0);
  nh.param("kfmt/search_time", search_time_, -1.0);
  nh.param("kfmt/use_regional_opt", use_regional_opt_, false);

  ROS_WARN_STREAM("[kfmt] param: vel_limit: " << vel_limit_);
  ROS_WARN_STREAM("[kfmt] param: acc_limit: " << acc_limit_);
  ROS_WARN_STREAM("[kfmt] param: jerk_limit: " << jerk_limit_);
  ROS_WARN_STREAM("[kfmt] param: debug_vis: " << debug_vis_);
  ROS_WARN_STREAM("[kfmt] param: rho: " << rho_);
  ROS_WARN_STREAM("[kfmt] param: preallocate_node_pool_num: " << preallocate_node_pool_num_);
  ROS_WARN_STREAM("[kfmt] param: preallocate_regional_node_pool_num: " << preallocate_regional_pool_num_);
  ROS_WARN_STREAM("[kfmt] param: sample_num_per_meter: " << sample_num_per_meter_);
  ROS_WARN_STREAM("[kfmt] param: radius_cost_between_two_states: " << radius_cost_between_two_states_);
  ROS_WARN_STREAM("[kfmt] param: allow_close_goal: " << allow_close_goal_);
  ROS_WARN_STREAM("[kfmt] param: lambda_heu: " << lambda_heu_);
  ROS_WARN_STREAM("[kfmt] param: tie_breaker: " << tie_breaker_);
  ROS_WARN_STREAM("[kfmt] param: search_time: " << search_time_);
  ROS_WARN_STREAM("[kfmt] param: use_regional_opt: " << use_regional_opt_);

  bvp_.init(TRIPLE_INTEGRATOR);
  bvp_.setRho(rho_);

  nh_ = nh;
  sampler_client_ = nh_.serviceClient<self_msgs_and_srvs::LearningSampler>("/learning_sampler_service");

  //pre allocate memory
  node_pool_.resize(preallocate_node_pool_num_);
  for (int i = 0; i < preallocate_node_pool_num_; i++) 
  {
    node_pool_[i] = new FMTNode;
  }
  regional_node_pool_.resize(preallocate_regional_pool_num_);
  for (int i = 0; i < preallocate_regional_pool_num_; i++) 
  {
    regional_node_pool_[i] = new FMTNode;
  }
  sampled_node_num_ = 0;
  regional_node_num_ = 0;
}

void KFMTPlanner::setPosChecker(const PosChecker::Ptr &checker)
{
  pos_checker_ptr_ = checker;
  sampler_.setPosChecker(checker);
}

void KFMTPlanner::setVisualizer(const VisualRviz::Ptr &vis)
{
  vis_ptr_ = vis;
}

// reset() is called every time before plan();
void KFMTPlanner::reset()
{
  std::priority_queue<FMTNodePtr, std::vector<FMTNodePtr>, FMTNodeComparator> empty_queue;
  open_set_.swap(empty_queue);

  for (int i = 0; i < sampled_node_num_; i++) 
  {
    node_pool_[i]->parent = nullptr;
    node_pool_[i]->children.clear();
    node_pool_[i]->status = UNVISITED;
    node_pool_[i]->f_score = DBL_MAX;
    node_pool_[i]->g_score = DBL_MAX;
  }
  iter_num_ = 0;
  sampled_node_num_ = 0;

  for (int i = 0; i < regional_node_num_; i++) 
  {
    regional_node_pool_[i]->parent = nullptr;
    regional_node_pool_[i]->children.clear();
    regional_node_pool_[i]->status = UNVISITED;
    regional_node_pool_[i]->f_score = DBL_MAX;
    regional_node_pool_[i]->g_score = DBL_MAX;
  }
  regional_node_num_ = 0;
}

int KFMTPlanner::plan(Vector3d start_pos, Vector3d start_vel, Vector3d start_acc, 
                      Vector3d end_pos, Vector3d end_vel, Vector3d end_acc, 
                      double search_time, const Vector3d &normal, const Vector3d &dire, bool need_consistancy, bool use_regional_opt)
{
  search_time_ = search_time;
  use_regional_opt_ = use_regional_opt;
  ros::Time plan_start_time = ros::Time::now();
  reset();
  if (pos_checker_ptr_->getVoxelState(start_pos) != 0) 
  {
    ROS_ERROR("[KFMT]: Start pos collide or out of bound");
    return FAILURE;
  }
  
  if (!pos_checker_ptr_->validatePosSurround(end_pos)) 
  {
    Vector3d shift[20] = {Vector3d(-1.0,0.0,0.0), Vector3d(0.0,1.0,0.0), Vector3d(0.0,-1.0,0.0), 
                         Vector3d(0.0,0.0,1.0), Vector3d(0.0,0.0,-1.0), Vector3d(1.0,0.0,0.0), 
                         Vector3d(-2.0,0.0,0.0), Vector3d(0.0,2.0,0.0), Vector3d(0.0,-2.0,0.0), 
                         Vector3d(0.0,0.0,2.0), Vector3d(0.0,0.0,-2.0), Vector3d(2.0,0.0,0.0), 

                         Vector3d(1.0,1.0,1.0), Vector3d(1.0,1.0,-1.0), Vector3d(1.0,-1.0,1.0), 
                         Vector3d(1.0,-1.0,-1.0), Vector3d(-1.0,1.0,-1.0), Vector3d(-1.1,1.0,1.0), 
                         Vector3d(-1.0,-1.0,1.0), Vector3d(-1.0,-1.0,-1.0)};
    ROS_WARN("[KFMT]: End pos collide or out of bound, search for other safe end");
    int i = 0;
    for (; i < 20; ++i)
    {
      end_pos += shift[i] * 0.2;
      if (pos_checker_ptr_->validatePosSurround(end_pos))
        break;
    }
    if (i == 20)
    {
      ROS_ERROR("found no valid end pos, plan fail");
      return FAILURE;
    }
  }
  
  if((start_pos - end_pos).norm() < 1e-3 && (start_vel - end_vel).norm() < 1e-4)
  {
    ROS_ERROR("[KFMT]: start state & end state too close");
    return FAILURE;
  }
  
  if (need_consistancy)
  { 
    vis_ptr_->visualizeReplanDire(start_pos, dire, pos_checker_ptr_->getLocalTime());
    replan_normal_ = normal;
    replan_dire_ = dire;
    need_consistancy_ = need_consistancy;
  }
  
  /* construct start and goal nodes */
  start_node_ = node_pool_[0];
  start_node_->x.head(3) = start_pos;
  start_node_->x.segment(3, 3) = start_vel;
  start_node_->x.tail(3) = start_acc;
  start_node_->g_score = 0.0;
  start_node_->status = OPEN;
  goal_node_ = node_pool_[1];
  goal_node_->x.head(3) = end_pos;
  if (end_vel.norm() >= vel_limit_)
  {
    end_vel.normalize();
    end_vel = end_vel * vel_limit_;
  }
  goal_node_->x.segment(3, 3) = end_vel;
  goal_node_->x.tail(3) = end_acc;
  start_node_->f_score = 0.0;//lambda_heu_ * bvp_.estimateHeuristic(start_node_->x.head(6), goal_node_->x.head(6));
  sampled_node_num_ = 2;

  /* init sampling space */
  vector<pair<Vector3d, Vector3d>> traversal_lines;
  ROS_INFO_STREAM("Kino FMT* starts planning");
  if (bvp_.solve(start_node_->x, goal_node_->x))
  {
    CoefficientMat coeff;
    bvp_.getCoeff(coeff);
    double best_tau = bvp_.getTauStar();
    double best_cost = bvp_.getCostStar();
    ROS_INFO("[KFMT]: Best cost: %lf, best tau: %lf", best_cost, best_tau);
    Piece poly_seg(best_tau, coeff);
    /* for traj vis */
    vector<StatePVA> vis_x;
    // vis_x.clear();
    // poly_seg.sampleOneSeg(&vis_x);
    // vis_ptr_->visualizeStates(vis_x, BestTraj, pos_checker_ptr_->getLocalTime());

    bool vel_cons = poly_seg.checkMaxVelRate(vel_limit_);
    bool acc_cons = poly_seg.checkMaxAccRate(acc_limit_);
    bool jerk_cons = poly_seg.checkMaxAccRate(jerk_limit_);
    bool pos_cons = getTraversalLines(poly_seg, traversal_lines);
    if (vel_cons && acc_cons && jerk_cons && pos_cons)
    {
      ROS_WARN("Best traj collision free, one shot connected");
      goal_node_->g_score = best_cost;
      goal_node_->parent = start_node_;
      goal_node_->poly_seg = poly_seg;
      fillTraj(goal_node_, traj_);
      time_usage_ = (ros::Time::now() - plan_start_time).toSec();
  
      /* for traj vis */
      vis_x.clear();
      traj_.sampleWholeTrajectory(&vis_x);
      vis_ptr_->visualizeStates(vis_x, FinalTraj, pos_checker_ptr_->getLocalTime());
      double final_traj_len(0.0), final_traj_duration(0.0), final_traj_acc_itg(0.0), final_traj_jerk_itg(0.0);
      int final_traj_seg_nums(0);
      evaluateTraj(traj_, final_traj_duration, final_traj_len, final_traj_seg_nums, final_traj_acc_itg, final_traj_jerk_itg);
      ROS_INFO_STREAM("[KFMT]: [front-end final path]: " << endl 
          << "    -   seg nums: " << final_traj_seg_nums << endl 
          << "    -   time: " << time_usage_ << endl 
          << "    -   acc integral: " << final_traj_acc_itg << endl
          << "    -   jerk integral: " << final_traj_jerk_itg << endl
          << "    -   traj duration: " << final_traj_duration << endl 
          << "    -   path length: " << final_traj_len);
      return SUCCESS;
    }
  }
  else
  {
    ROS_WARN("sth. wrong with the bvp solver");
  }
  sampler_.topoSetup(traversal_lines, start_pos, end_pos);
  vector<Vector3d> p_head, tracks;
  sampler_.getTopo(p_head, tracks);
  vis_ptr_->visualizeTopo(p_head, tracks, pos_checker_ptr_->getLocalTime());

  return fmtStar(start_node_->x, goal_node_->x, preallocate_node_pool_num_ - 2);
}

int KFMTPlanner::fmtStar(const StatePVA& x_init, const StatePVA& x_final, int sample_num)
{ 
  ros::Time fmt_start_time = ros::Time::now();
  ros::Time final_goal_found_time;
  
  /* local variables */
  vector<StatePVA> vis_x;
  vector<StatePVA> region_traj_vis_x;
  bool close_goal_found = false;
  bool goal_found = false;

  /* kd tree init */
  kdtree *kd_tree = kd_create(3);
  //Add start and goal nodes to kd tree
  kd_insert3(kd_tree, start_node_->x[0], start_node_->x[1], start_node_->x[2], start_node_);
  kd_insert3(kd_tree, goal_node_->x[0], goal_node_->x[1], goal_node_->x[2], goal_node_);

  /* sample a batch of samples */
  batchSampling(sample_num, kd_tree);
  // batchSamplingLearning(sample_num, kd_tree);

  //TODO changable radius
  double tau_for_instance = radius_cost_between_two_states_ * 0.75; //maximum
  double fwd_radius_p = getForwardRadius(tau_for_instance, radius_cost_between_two_states_);  
  double bcwd_radius_p = getBackwardRadius(tau_for_instance, radius_cost_between_two_states_);
  ROS_INFO_STREAM("bcwd_radius_p: " << bcwd_radius_p);
  ROS_INFO_STREAM("fwd_radius_p: " << fwd_radius_p);

  /* main loop */
  open_set_.push(start_node_);
  while (!open_set_.empty() && (ros::Time::now() - fmt_start_time).toSec() < search_time_) 
  {
    FMTNodePtr cur_node = open_set_.top();
    open_set_.pop();
    struct kdres *p_fwd_nbr_set;
    p_fwd_nbr_set = getForwardNeighbour(cur_node->x, kd_tree, tau_for_instance, fwd_radius_p);
    if (p_fwd_nbr_set == nullptr)
    {
      ROS_FATAL_STREAM("fwd kd range query error");
      break;
    }
    while (!kd_res_end(p_fwd_nbr_set)) 
    {
      FMTNodePtr cur_fwd_nbr = (FMTNodePtr)kd_res_item_data(p_fwd_nbr_set);
      if (cur_fwd_nbr->status != UNVISITED)
      {
        kd_res_next(p_fwd_nbr_set);
        continue;
      }
      struct kdres *p_bcwd_nbr_set;
      p_bcwd_nbr_set = getBackwardNeighbour(cur_fwd_nbr->x, kd_tree, radius_cost_between_two_states_ - tau_for_instance, bcwd_radius_p);
      if (p_bcwd_nbr_set == nullptr)
      {
        ROS_FATAL_STREAM("bkwd kd range query error");
        break;
      }
      double tau, cost;
      CoefficientMat coeff;
      double g_score_hat, cur_fwd_g_score(DBL_MAX);
      FMTNodePtr bcwd_nbr;
      bool parent_cost_valid(false);
      while (!kd_res_end(p_bcwd_nbr_set)) 
      {
        FMTNodePtr cur_bcwd_nbr = (FMTNodePtr)kd_res_item_data(p_bcwd_nbr_set);
        if (cur_bcwd_nbr->status != OPEN)
        {
          kd_res_next(p_bcwd_nbr_set);
          continue;
        }
        bool cost_valid = bvp_.solve(cur_bcwd_nbr->x, cur_fwd_nbr->x);
        g_score_hat = bvp_.getCostStar() + cur_bcwd_nbr->g_score;
        if (cost_valid && cur_fwd_g_score > g_score_hat)
        {
          tau = bvp_.getTauStar();
          cost = bvp_.getCostStar();
          bvp_.getCoeff(coeff);
          cur_fwd_g_score = g_score_hat;
          parent_cost_valid = true;
          bcwd_nbr = cur_bcwd_nbr;
        }
        kd_res_next(p_bcwd_nbr_set); //go to next in kd tree range query result
      }
      double dis = (cur_fwd_nbr->x - bcwd_nbr->x).head(3).norm();
      if (parent_cost_valid)
      {
        Piece seg_from_parent = Piece(tau, coeff);
        bool vel_cons = seg_from_parent.checkMaxVelRate(vel_limit_);
        bool acc_cons = seg_from_parent.checkMaxAccRate(acc_limit_);
        bool jerk_cons = seg_from_parent.checkMaxAccRate(jerk_limit_);
        pair<Vector3d, Vector3d> collide_pts_one_seg;
        pair<double, double> t_s_e;
        bool need_region_opt(false);
        bool pos_cons = pos_checker_ptr_->checkPolySeg(seg_from_parent, collide_pts_one_seg, t_s_e, need_region_opt);
        bool connected = vel_cons && acc_cons && jerk_cons && pos_cons;
        // if (checkSegmentConstraints(seg_from_parent))
        if (connected)
        {
          bcwd_nbr->children.push_back(cur_fwd_nbr);
          cur_fwd_nbr->parent = bcwd_nbr;
          cur_fwd_nbr->g_score = g_score_hat;
          cur_fwd_nbr->f_score = cur_fwd_nbr->g_score;// + lambda_heu_ * bvp_.estimateHeuristic(cur_fwd_nbr->x.head(6), goal_node_->x.head(6));
          cur_fwd_nbr->status = OPEN;
          cur_fwd_nbr->poly_seg = Piece(tau, coeff);
          cur_fwd_nbr->x.segment(6, 3) = cur_fwd_nbr->poly_seg.getAcc(tau);
          open_set_.push(cur_fwd_nbr);
          close_goal_node_ = cur_fwd_nbr;
          close_goal_found = true;
          /* -------- visualize a single step */
          if (debug_vis_)
          {
            fillTraj(cur_fwd_nbr, traj_);  
            vis_x.clear();
            traj_.sampleWholeTrajectory(&vis_x);
            vis_ptr_->visualizeStates(vis_x, FirstTraj, pos_checker_ptr_->getLocalTime());
            vis_x.clear();
            vector<Vector3d> knots;
            sampleWholeTree(start_node_, &vis_x, knots);
            vis_ptr_->visualizeStates(vis_x, TreeTraj, pos_checker_ptr_->getLocalTime());
            vis_ptr_->visualizeKnots(knots, pos_checker_ptr_->getLocalTime());
            // getchar();
          }
          // goal found
          if (cur_fwd_nbr == goal_node_)
          {
            // ROS_INFO("goal found!");
            goal_found = true;
            kd_res_free(p_bcwd_nbr_set); //reset kd tree range query
            break;
          } 
          //try to connect to goal
          if ((cur_fwd_nbr->x.head(3) - goal_node_->x.head(3)).norm() <= 10) //only try to connect to goal if within 10m
            continue;
          if (bvp_.solve(cur_fwd_nbr->x, goal_node_->x, ACC_KNOWN))
          {
            CoefficientMat coeff;
            bvp_.getCoeff(coeff);
            Piece seg2goal = Piece(bvp_.getTauStar(), coeff);
            bool connected_to_goal = checkSegmentConstraints(seg2goal);
            if (connected_to_goal) 
            {
              goal_node_->parent = cur_fwd_nbr;
              goal_node_->g_score = cur_fwd_nbr->g_score + bvp_.getCostStar();
              goal_node_->f_score = goal_node_->g_score;
              goal_node_->poly_seg = seg2goal;
              cur_fwd_nbr->children.push_back(goal_node_);
              goal_found = true;
            }
          }
        }
        else if (use_regional_opt_ && need_region_opt && regional_node_num_ < preallocate_regional_pool_num_ - 3
        && (dis < 5.0 && dis > 2.0 && (t_s_e.second - t_s_e.first) <= 0.5 && ((cur_fwd_nbr->x-x_init).head(3).norm() - (bcwd_nbr->x-x_init).head(3).norm()) > 3))//TODO minus 3
        {
          // ROS_ERROR_STREAM("[FMT] regional_opt start");
          Trajectory local_opt_traj;
          // bool regional_opt_result(false);
          // ros::Time r_o_ts = ros::Time::now();

          // if (searcher_->AstarSearch(pos_checker_ptr_->getResolution(), collide_pts_one_seg.first, collide_pts_one_seg.second))
          // {
          //   vector<Eigen::Vector3d> grid_path = searcher_->getPath();
          //   vis_ptr_->visualizeKnots(grid_path, pos_checker_ptr_->getLocalTime());
          // }
          // // getchar();
          // regional_opt_result = regional_opt_ptr_->generateRegionalTraj(bcwd_nbr->x, cur_fwd_nbr->x, collide_pts_one_seg, local_opt_traj);
          // ROS_ERROR_STREAM("[FMT] regional_opt costs " << 1e3 * (ros::Time::now() - r_o_ts).toSec() << " ms. Result: " << regional_opt_result);
          // getchar();


          bool regional_opt_result(false);
          ros::Time r_o_ts = ros::Time::now();

          CoefficientMat coeff1 = seg_from_parent.getCoeffMat(), coeff2;
          double duration1 = seg_from_parent.getDuration() / 2.0;
          Trajectory pre_regional_traj;
          pre_regional_traj.reserve(2);
          pre_regional_traj.emplace_back(Piece(duration1, coeff1));
          seg_from_parent.cutPiece(seg_from_parent, duration1, coeff2);
          pre_regional_traj.emplace_back(Piece(duration1, coeff2));
          if (searcher_->AstarSearch(pos_checker_ptr_->getResolution(), collide_pts_one_seg.first, collide_pts_one_seg.second))
          {
            vector<Eigen::Vector3d> grid_path = searcher_->getPath();
            // vis_ptr_->visualizeKnots(grid_path, pos_checker_ptr_->getLocalTime());

            std::vector<pair<int, int>> seg_num_obs_size;  //first: # of segment in a traj; second: number of attract_pt in this segment. 
            std::vector<double> t_s, t_e; // each attract_pt's timestamp of start and end in its corresponding segment. Size should be the same as the sum of seg_num_obs_size.second.
            std::vector<Eigen::Vector3d> attract_pts;
            pos_checker_ptr_->getRegionalAttractPts(pre_regional_traj, grid_path, t_s_e, seg_num_obs_size, attract_pts, t_s, t_e);
            if (optimizer_ptr_->initialize(pre_regional_traj, TrajOptimizer::SMOOTH_HOMO_OBS))
              regional_opt_result = optimizer_ptr_->solveRegionalOpt(seg_num_obs_size, attract_pts, t_s, t_e);
          }
          
          if (regional_opt_result)
          {
            optimizer_ptr_->getTraj(local_opt_traj);

            int n_mid_with_last_pt = local_opt_traj.getPieceNum();
            double cost[n_mid_with_last_pt];
            double regional_traj_cost(0.0);
            for (int j = 0; j < n_mid_with_last_pt; j++)
            {
              cost[j] = local_opt_traj[j].calCost(rho_); //calculate cost of one piece 
              regional_traj_cost += cost[j];
            }

            local_opt_traj.sampleWholeTrajectory(&region_traj_vis_x);
            // ROS_WARN("[FMT] regional optimized");

            StatePVA mid_x; 
            Vector3d pos, vel, acc;
            FMTNode *curr_node_in_regional_traj(nullptr), *last_node(nullptr);
            for (int j = 0; j < n_mid_with_last_pt; j++)
            {
              double tau = local_opt_traj[j].getDuration();
              pos = local_opt_traj[j].getPos(tau);
              vel = local_opt_traj[j].getVel(tau);
              acc = local_opt_traj[j].getAcc(tau);
              mid_x.segment(0, 3) = pos;
              mid_x.segment(3, 3) = vel;
              mid_x.segment(6, 3) = acc;
              /* 1.1 add the randomly sampled node to fmt_tree */
              if (j == 0)
              {
                curr_node_in_regional_traj = regional_node_pool_[regional_node_num_]; 
                regional_node_num_++;
                curr_node_in_regional_traj->x = mid_x;
                curr_node_in_regional_traj->parent = bcwd_nbr;
                bcwd_nbr->children.push_back(curr_node_in_regional_traj);
                curr_node_in_regional_traj->g_score = bcwd_nbr->g_score + cost[j];
              }
              else if (j == n_mid_with_last_pt - 1)
              {
                curr_node_in_regional_traj = cur_fwd_nbr;
                cur_fwd_nbr->parent = last_node;
                last_node->children.push_back(cur_fwd_nbr);
                cur_fwd_nbr->g_score = last_node->g_score + cost[j];
                cur_fwd_nbr->f_score = cur_fwd_nbr->g_score;// + lambda_heu_ * bvp_.estimateHeuristic(cur_fwd_nbr->x.head(6), goal_node_->x.head(6));
                cur_fwd_nbr->status = OPEN;
                open_set_.push(cur_fwd_nbr);
              }
              else
              {
                curr_node_in_regional_traj = regional_node_pool_[regional_node_num_]; 
                regional_node_num_++;
                curr_node_in_regional_traj->x = mid_x;
                curr_node_in_regional_traj->parent = last_node;
                last_node->children.push_back(curr_node_in_regional_traj);
                curr_node_in_regional_traj->g_score = last_node->g_score + cost[j];
              }
              curr_node_in_regional_traj->poly_seg = local_opt_traj[j];
              last_node = curr_node_in_regional_traj;
              /* 1.2 add the randomly sampled node to kd_tree */
              // kd_insert3(kd_tree, mid_x[0], mid_x[1], mid_x[2], curr_node_in_regional_traj);
            }
            close_goal_node_ = cur_fwd_nbr;
            close_goal_found = true;

            // goal found
            if (cur_fwd_nbr == goal_node_)
            {
              // ROS_INFO("goal found!");
              goal_found = true;
              kd_res_free(p_bcwd_nbr_set); //reset kd tree range query
              break;
            } 
            //try to connect to goal
            if ((cur_fwd_nbr->x.head(3) - goal_node_->x.head(3)).norm() <= 10) //only try to connect to goal if within 10m
              continue;
            if (bvp_.solve(cur_fwd_nbr->x, goal_node_->x, ACC_KNOWN))
            {
              CoefficientMat coeff;
              bvp_.getCoeff(coeff);
              Piece seg2goal = Piece(bvp_.getTauStar(), coeff);
              bool connected_to_goal = checkSegmentConstraints(seg2goal);
              if (connected_to_goal) 
              {
                goal_node_->parent = cur_fwd_nbr;
                goal_node_->g_score = cur_fwd_nbr->g_score + bvp_.getCostStar();
                goal_node_->f_score = goal_node_->g_score;
                goal_node_->poly_seg = seg2goal;
                cur_fwd_nbr->children.push_back(goal_node_);
                goal_found = true;
              }
            }

          }
        }
        
      }
      kd_res_free(p_bcwd_nbr_set); //reset kd tree range query
      kd_res_next(p_fwd_nbr_set); //go to next in kd tree range query result
    }
    kd_res_free(p_fwd_nbr_set); //reset kd tree range query
    cur_node->status = CLOSED;
    iter_num_++;
    if (goal_found)
    {
      break;
    }
  }
  kd_free(kd_tree);

  vis_x.clear();
  vector<Vector3d> knots;
  sampleWholeTree(start_node_, &vis_x, knots);
  vis_ptr_->visualizeStates(vis_x, TreeTraj, pos_checker_ptr_->getLocalTime());
  vis_ptr_->visualizeKnots(knots, pos_checker_ptr_->getLocalTime());
  vis_ptr_->visualizePoints(region_traj_vis_x, pos_checker_ptr_->getLocalTime());
  
  if (goal_found)
  {
    fillTraj(goal_node_, traj_);
    time_usage_ = (ros::Time::now() - fmt_start_time).toSec();
    return SUCCESS;
  }
  else if (allow_close_goal_ && close_goal_found)
  {
    ROS_WARN("open set empty, no path, choose bypass");
    // chooseBypass(close_goal_node_, start_node_);
    fillTraj(close_goal_node_, traj_);
    time_usage_ = (ros::Time::now() - fmt_start_time).toSec();
    return SUCCESS_CLOSE_GOAL;
  }
  else if ((ros::Time::now() - fmt_start_time).toSec() >= search_time_)
  {
    ROS_ERROR_STREAM("[KFMT]: NOT CONNECTED TO GOAL after " << (ros::Time::now() - fmt_start_time).toSec() << " seconds");
    ROS_INFO_STREAM("[KFMT]: total iteration times: " << iter_num_);
    return FAILURE;
  }
  else
  {
    ROS_ERROR_STREAM("[KFMT]: Open Set Empty, No Path. Used " << (ros::Time::now() - fmt_start_time).toSec() << " seconds");
    ROS_INFO_STREAM("[KFMT]: total iteration times: " << iter_num_);
    return FAILURE;
  }

}

double KFMTPlanner::getForwardRadius(double tau, double cost)
{
  MatrixXd G(3, 3);
  G.setZero();
  double tau_2 = tau * tau;
  double tau_3 = tau_2 * tau;
  double tau_4 = tau_3 * tau;
  double tau_5 = tau_4 * tau;
  G(0, 0) = 720.0 / tau_5;
  G(1, 1) = 192.0 / tau_3;
  G(2, 2) = 9.0 / tau;
  G(0, 1) = G(1, 0) = -360.0 / tau_4;
  G(0, 2) = G(2, 0) = 60.0 / tau_3;
  G(1, 2) = G(2, 1) = -36.0 / tau_2;
  G = G * rho_ / (cost - tau) * 3.0;
  // ROS_INFO_STREAM("G: \n" << G);
  Eigen::EigenSolver<MatrixXd> es(G);
  // cout << "The eigenvalues of G are:" << endl << es.eigenvalues() << endl;
  // cout << "The matrix of eigenvectors, V, is:" << endl << es.eigenvectors() << endl << endl;
  double radius_p(0.0);
  for (int i = 0; i < 3; ++i)
  {
    radius_p = max(radius_p, sqrt(1.0 / es.eigenvalues()[i].real()) * fabs(es.eigenvectors().col(i)[i]));
  }
  return radius_p * 1.732; //sqrt(1 + 1 + 1)
}

double KFMTPlanner::getBackwardRadius(double tau, double cost)
{
  MatrixXd G(3, 3);
  G.setZero();
  double tau_2 = tau * tau;
  double tau_3 = tau_2 * tau;
  double tau_4 = tau_3 * tau;
  double tau_5 = tau_4 * tau;
  G(0, 0) = 720.0 / tau_5;
  G(1, 1) = 192.0 / tau_3;
  G(2, 2) = 9.0 / tau;
  G(0, 1) = G(1, 0) = -360.0 / tau_4;
  G(0, 2) = G(2, 0) = 60.0 / tau_3;
  G(1, 2) = G(2, 1) = -36.0 / tau_2;
  G = G * rho_ / (cost - tau) * 3.0;
  // ROS_INFO_STREAM("G: \n" << G);
  Eigen::EigenSolver<MatrixXd> es(G);
  // cout << "The eigenvalues of G are:" << endl << es.eigenvalues() << endl;
  // cout << "The matrix of eigenvectors, V, is:" << endl << es.eigenvectors() << endl << endl;
  double radius_p(0.0);
  for (int i = 0; i < 3; ++i)
  {
    radius_p = max(radius_p, sqrt(1.0 / es.eigenvalues()[i].real()) * fabs(es.eigenvectors().col(i)[i]));
  }
  return radius_p * 1.732; //sqrt(1 + 1 + 1)
}

struct kdres *KFMTPlanner::getForwardNeighbour(const StatePVA& x0, struct kdtree *kd_tree, double tau, double radius_p)
{
  double half_tau_square = tau * tau / 2;
  StatePVA x_ba_tau;
  x_ba_tau[0] = x0[0] + x0[3]*tau + x0[6]*half_tau_square;
  x_ba_tau[1] = x0[1] + x0[4]*tau + x0[7]*half_tau_square;
  x_ba_tau[2] = x0[2] + x0[5]*tau + x0[8]*half_tau_square;
  // x_ba_tau[3] = x0[3] + tau*x0[6];
  // x_ba_tau[4] = x0[4] + tau*x0[7];
  // x_ba_tau[5] = x0[5] + tau*x0[8];
  // x_ba_tau[6] = x0[6];
  // x_ba_tau[7] = x0[7];
  // x_ba_tau[8] = x0[8];

  if (debug_vis_)
  {
    vis_ptr_->visualizeReachPos(FORWARD_REACHABLE_POS, x_ba_tau.head(3), 2 * radius_p, pos_checker_ptr_->getLocalTime());
    // vis_ptr_->visualizeReachVel(1, x_ba_tau.head(3) + x_ba_tau.tail(3), 2 * radius_v, pos_checker_ptr_->getLocalTime());
    // getchar();
  }
  return kd_nearest_range3(kd_tree, x_ba_tau[0], x_ba_tau[1], x_ba_tau[2], radius_p);
}

struct kdres *KFMTPlanner::getBackwardNeighbour(const StatePVA& x1, struct kdtree *kd_tree, double tau, double radius_p)
{
  StatePVA expNegATau_x1;
  double half_tau_square = tau * tau / 2;
  expNegATau_x1[0] = x1[0] - x1[3]*tau + x1[6]*half_tau_square;
  expNegATau_x1[1] = x1[1] - x1[4]*tau + x1[7]*half_tau_square;
  expNegATau_x1[2] = x1[2] - x1[5]*tau + x1[8]*half_tau_square;
  // expNegATau_x1[3] = x1[3] - x1[6]*tau;
  // expNegATau_x1[4] = x1[4] - x1[7]*tau;
  // expNegATau_x1[5] = x1[5] - x1[8]*tau;
  // expNegATau_x1[6] = x1[6];
  // expNegATau_x1[7] = x1[7];
  // expNegATau_x1[8] = x1[8];

  if (debug_vis_)
  {
    vis_ptr_->visualizeReachPos(BACKWARD_REACHABLE_POS, expNegATau_x1.head(3), 2 * radius_p, pos_checker_ptr_->getLocalTime());
    // vis_ptr_->visualizeReachVel(1, expNegATau_x1.head(3) + expNegATau_x1.tail(3), 2 * radius_v, pos_checker_ptr_->getLocalTime());
    // getchar();
  }
  return kd_nearest_range3(kd_tree, expNegATau_x1[0], expNegATau_x1[1], expNegATau_x1[2], radius_p);
}

inline double KFMTPlanner::dist(const StatePVA& x0, const StatePVA& x1)
{
  Vector3d p_diff(x0.head(3) - x1.head(3));
  return p_diff.norm();
}

inline void KFMTPlanner::sampleWholeTree(const FMTNodePtr &root, vector< StatePVA >* vis_x, vector<Vector3d>& knots)
{
  if (root == nullptr) 
    return;
  //whatever dfs or bfs
  FMTNode* node = root;
  std::queue<FMTNode*> Q;
  Q.push(node);
  while (!Q.empty()) 
  {
    node = Q.front();
    Q.pop();
    for (const auto& leafptr : node->children) 
    {
      leafptr->poly_seg.sampleOneSeg(vis_x);
      knots.push_back(leafptr->x.head(3));
      Q.push(leafptr);
    }
  }
}

void KFMTPlanner::fillTraj(const FMTNodePtr &goal_leaf, Trajectory& traj)
{
  std::vector<double> durs;
  std::vector<CoefficientMat> coeffMats;
  FMTNodePtr node = goal_leaf;
  while (node->parent) 
  {
    durs.push_back(node->poly_seg.getDuration());
    coeffMats.push_back(node->poly_seg.getCoeffMat());
    node = node->parent;
  }
  std::reverse(std::begin(durs), std::end(durs));
  std::reverse(std::begin(coeffMats), std::end(coeffMats));
  traj = Trajectory(durs, coeffMats);
}

inline bool KFMTPlanner::checkSegmentConstraints(const Piece &seg)
{
  if (!seg.checkMaxVelRate(vel_limit_))
  {
    // ROS_WARN("vel constraints violate!");
    return false;
  }
  if (!seg.checkMaxAccRate(acc_limit_))
  {
    // ROS_WARN("acc constraints violate!");
    return false;
  }
  if (!seg.checkMaxJerkRate(jerk_limit_))
  {
    // ROS_WARN("jerk constraints violate!");
    return false;
  }
  if (!pos_checker_ptr_->checkPolySeg(seg))
  {
    // ROS_WARN("pos constraints violate!");
    return false;
  }
  return true;
}

inline bool KFMTPlanner::getTraversalLines(const Piece &seg, vector<pair<Vector3d, Vector3d>> &traversal_lines)
{
  bool res = pos_checker_ptr_->checkPolySeg(seg, traversal_lines);
  return res;
}

void KFMTPlanner::evaluateTraj(const Trajectory& traj, double& traj_duration, double& traj_length, int& seg_nums, 
                               double& acc_integral, double& jerk_integral)
{
  traj_length = 0.0;
  traj_duration = 0.0;
  seg_nums = traj.getPieceNum();
  acc_integral = 0.0;
  jerk_integral = 0.0;

  double d_t = 0.03;
  for (int i = 0; i < seg_nums; ++i)
  {
    double tau = traj[i].getDuration();
    traj_duration += tau;
    for (double t = 0.0; t < tau; t += d_t) 
    {
      Eigen::Vector3d vel, acc, jerk;
      vel = traj[i].getVel(t);
      acc = traj[i].getAcc(t);
      jerk = traj[i].getJerk(t);
      traj_length += vel.norm() * d_t;
      acc_integral += acc.dot(acc) * d_t;
      jerk_integral += jerk.dot(jerk) * d_t;
    }
  }
}

void KFMTPlanner::batchSampling(int sample_num, kdtree *kd_tree)
{
  vector<Vector3d> p_head, tracks;
  sampler_.getTopo(p_head, tracks);
  vector<double> lengths;
  size_t track_num = tracks.size();
  double track_len_add_up(0.0);
  for (size_t i = 0; i < track_num; i++)
  {
    double len = tracks[i].norm();
    lengths.push_back(len);
    track_len_add_up += len;
  }
  int n = std::min(int(track_len_add_up * sample_num_per_meter_), sample_num);
  StatePVA x_rand;
  vector<StatePVA> valid_samples;
  bool is_good_sample(false);
  ros::Time t_sample_s = ros::Time::now();
  for (size_t i = 0; i < track_num; ++i)
  {
    int sample_num_cur_track = floor(lengths[i] / track_len_add_up * n);
    int good_sample_num(0);
    while (good_sample_num < sample_num_cur_track)
    {
      is_good_sample = sampler_.samplingOnce(i, x_rand);
      if (is_good_sample) 
      {
        good_sample_num++;
        valid_samples.push_back(x_rand);
        FMTNodePtr rand_ptr = node_pool_[sampled_node_num_];
        sampled_node_num_++;
        rand_ptr->x = x_rand;
        kd_insert3(kd_tree, x_rand[0], x_rand[1], x_rand[2], rand_ptr);
      }
    }
  }
  vis_ptr_->visualizeSampledState(valid_samples, pos_checker_ptr_->getLocalTime());
}

void KFMTPlanner::batchSamplingLearning(int sample_num, kdtree *kd_tree)
{
  vector<Vector3d> p_head, tracks;
  sampler_.getTopo(p_head, tracks);
  vector<double> lengths;
  size_t track_num = tracks.size();
  double track_len_add_up(0.0);
  for (size_t i = 0; i < track_num; i++)
  {
    double len = tracks[i].norm();
    lengths.push_back(len);
    track_len_add_up += len;
  }
  int n = std::min(int(track_len_add_up * sample_num_per_meter_), sample_num);
  vector<StatePVA> valid_samples;
  bool is_good_sample(false);

  self_msgs_and_srvs::LearningSampler srv;
  self_msgs_and_srvs::input_point input_point;
  Vector3d p_m;
  vector<Vector3d> topo_pts, sp_pts;
  for (size_t i = 0; i < track_num; ++i)
  {
    int sample_num_cur_track = floor(lengths[i] / track_len_add_up * n);
    Vector3d step = tracks[i] / (sample_num_cur_track + 1);
    p_m = p_head[i];
    for (int j = 0; j < sample_num_cur_track; ++j)
    {
      p_m += step;
      input_point.cur_pos.x = p_m[0];
      input_point.cur_pos.y = p_m[1];
      input_point.cur_pos.z = p_m[2];
      input_point.pred_pos.x = p_head[i][0];
      input_point.pred_pos.y = p_head[i][1];
      input_point.pred_pos.z = p_head[i][2];
      input_point.succ_pos.x = p_head[i][0] + tracks[i][0];
      input_point.succ_pos.y = p_head[i][1] + tracks[i][1];
      input_point.succ_pos.z = p_head[i][2] + tracks[i][2];
      srv.request.input_traj.push_back(input_point);

      topo_pts.push_back(p_m);
    }
  }
  
  if (sampler_client_.call(srv))
  {
    StatePVA x_rand;
    int num = srv.response.output_traj.size();
    for (int i = 0; i < num; ++i)
    {
      x_rand[0] = srv.response.output_traj[i].pos_mu.x;
      x_rand[1] = srv.response.output_traj[i].pos_mu.y;
      x_rand[2] = srv.response.output_traj[i].pos_mu.z;
      x_rand[3] = srv.response.output_traj[i].vel_mu.x;
      x_rand[4] = srv.response.output_traj[i].vel_mu.y;
      x_rand[5] = srv.response.output_traj[i].vel_mu.z;
      Vector3d pt = x_rand.head(3);
      sp_pts.push_back(pt);
      if (pos_checker_ptr_->validatePosSurround(pt))
      {
        valid_samples.push_back(x_rand);
        FMTNodePtr rand_ptr = node_pool_[sampled_node_num_];
        sampled_node_num_++;
        rand_ptr->x = x_rand;
        kd_insert3(kd_tree, x_rand[0], x_rand[1], x_rand[2], rand_ptr);
      }
    }
  }
  else
  {
    ROS_ERROR("Failed to call service /learning_sampler_service");
    return;
  }

  vis_ptr_->visualizeLearning(topo_pts, sp_pts, pos_checker_ptr_->getLocalTime());
  vis_ptr_->visualizeSampledState(valid_samples, pos_checker_ptr_->getLocalTime());
}

void KFMTPlanner::getTraj(Trajectory &traj)
{
  traj = traj_;
}

// void KFMTPlanner::chooseBypass(FMTNodePtr &goal_leaf, const FMTNodePtr &tree_start_node)
// {
//   goal_leaf = node_pool_[valid_start_tree_node_nums_ - 1];
//   double max_score(0.0);
//   if (need_consistancy_)
//   {
//     Vector3d p1 = tree_start_node->x.head(3);
//     for (const auto &leaf : tree_start_node->children)
//     {
//       Vector3d p2 = leaf->x.head(3);
//       Vector3d dire = (p2 - p1).normalized();
//       double score = dire.dot(replan_dire_);
//       if (score >= max_score)
//       {
//         max_score = score;
//         goal_leaf = leaf;
//       }
//     }
//   }
//   else
//   {
//     double close_dist(DBL_MAX);
//     for (int i = 2; i < valid_start_tree_node_nums_; ++i)
//     {
//       double dist_to_goal = dist(node_pool_[i]->x, goal_node_->x);
//       if (dist_to_goal < close_dist)
//       {
//         close_dist = dist_to_goal;
//         close_goal_node_ = node_pool_[i];
//       }
//     }
//   }
// }

} //namespace tgk_planner
