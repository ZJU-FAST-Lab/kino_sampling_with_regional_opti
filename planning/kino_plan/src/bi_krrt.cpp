#include "kino_plan/bi_krrt.h"
#include <queue>
#include <unordered_set>

namespace kino_planner
{
BIKRRT::BIKRRT(const ros::NodeHandle& nh): sampler_(nh)
{
}

BIKRRT::~BIKRRT()
{
  for (int i = 0; i < tree_node_nums_; i++) 
    delete node_pool_[i];
}

void BIKRRT::init(const ros::NodeHandle& nh)
{
  nh.param("bikrrt/vel_limit", vel_limit_, -1.0);
  nh.param("bikrrt/acc_limit", acc_limit_, -1.0);
  nh.param("bikrrt/jerk_limit", jerk_limit_, -1.0);
  nh.param("bikrrt/debug_vis", debug_vis_, false);
  nh.param("bikrrt/rho", rho_, -1.0);
  nh.param("bikrrt/tree_node_nums", tree_node_nums_, 0);
  nh.param("bikrrt/radius_cost_between_two_states", radius_cost_between_two_states_, 0.0);
  nh.param("bikrrt/allow_close_goal", allow_close_goal_, false);
  nh.param("bikrrt/stop_after_first_traj_found", stop_after_first_traj_found_, false);
  nh.param("bikrrt/rewire", rewire_, true);
  nh.param("bikrrt/use_regional_opt", use_regional_opt_, false);
  nh.param("bikrrt/test_convergency", test_convergency_, false);
  
  ROS_WARN_STREAM("[bikrrt] param: vel_limit: " << vel_limit_);
  ROS_WARN_STREAM("[bikrrt] param: acc_limit: " << acc_limit_);
  ROS_WARN_STREAM("[bikrrt] param: jerk_limit: " << jerk_limit_);
  ROS_WARN_STREAM("[bikrrt] param: debug_vis: " << debug_vis_);
  ROS_WARN_STREAM("[bikrrt] param: rho: " << rho_);
  ROS_WARN_STREAM("[bikrrt] param: tree_node_nums: " << tree_node_nums_);
  ROS_WARN_STREAM("[bikrrt] param: radius_cost_between_two_states: " << radius_cost_between_two_states_);
  ROS_WARN_STREAM("[bikrrt] param: allow_close_goal: " << allow_close_goal_);
  ROS_WARN_STREAM("[bikrrt] param: stop_after_first_traj_found: " << stop_after_first_traj_found_);
  ROS_WARN_STREAM("[bikrrt] param: rewire: " << rewire_);
  ROS_WARN_STREAM("[bikrrt] param: use_regional_opt: " << use_regional_opt_);
  ROS_WARN_STREAM("[bikrrt] param: test_convergency: " << test_convergency_);

  bvp_.init(TRIPLE_INTEGRATOR);
  bvp_.setRho(rho_);

  valid_start_tree_node_nums_ = 0;
  
  //pre allocate memory
  node_pool_.resize(tree_node_nums_);
  for (int i = 0; i < tree_node_nums_; i++) 
  {
    node_pool_[i] = new RRTNode;
  }
}

void BIKRRT::setPosChecker(const PosChecker::Ptr &checker)
{
  pos_checker_ptr_ = checker;
  sampler_.setPosChecker(checker);
}

void BIKRRT::setVisualizer(const VisualRviz::Ptr &vis)
{
  vis_ptr_ = vis;
}

// reset() is called every time before plan();
void BIKRRT::reset()
{
  for (int i = 0; i <= valid_start_tree_node_nums_; i++) 
  {
    node_pool_[i]->parent = nullptr;
    node_pool_[i]->children.clear();
  }
  valid_start_tree_node_nums_ = 0;
}

int BIKRRT::plan(Vector3d start_pos, Vector3d start_vel, Vector3d start_acc, 
                      Vector3d end_pos, Vector3d end_vel, Vector3d end_acc, 
                      double search_time)
{
  t_start_ = ros::Time::now();

  if (pos_checker_ptr_->getVoxelState(start_pos) != 0) 
  {
    ROS_ERROR("[BIKRRT]: Start pos collide or out of bound");
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
    ROS_WARN("[BIKRRT]: End pos collide or out of bound, search for other safe end");
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
    ROS_ERROR("[BIKRRT]: start state & end state too close");
    return FAILURE;
  }
  
  /* construct start and goal nodes */
  start_node_ = node_pool_[1]; //init ptr
  start_node_->x.head(3) = start_pos;
  start_node_->x.segment(3, 3) = start_vel;
  start_node_->x.tail(3) = start_acc;
  start_node_->cost_from_start = 0.0;
  start_node_->tau_from_start = 0.0;
  goal_node_ = node_pool_[0]; //init ptr
  goal_node_->x.head(3) = end_pos;
  if (end_vel.norm() >= vel_limit_)
  {
    end_vel.normalize();
    end_vel = end_vel * vel_limit_;
  }
  goal_node_->x.segment(3, 3) = end_vel;
  goal_node_->x.tail(3) = end_acc;
  goal_node_->cost_from_start = 0; //important
  goal_node_->tau_from_start = 0; //important
  goal_node_->tree_type = GOAL_TREE;
  valid_start_tree_node_nums_ = 2; //start and goal already in node_pool_

  /* init sampling space */
  vector<pair<Vector3d, Vector3d>> traversal_lines;
	ROS_INFO_STREAM("Kino RRT* starts planning");
  if (bvp_.solve(start_node_->x, goal_node_->x, ACC_KNOWN))
  {
    CoefficientMat coeff;
    bvp_.getCoeff(coeff);
    double best_tau = bvp_.getTauStar();
    double best_cost = bvp_.getCostStar();
    ROS_INFO("[BIKRRT]: Best cost: %lf, best tau: %lf", best_cost, best_tau);
    Piece poly_seg(best_tau, coeff);
    /* for traj vis */
    vector<StatePVA> vis_x;
    // vis_x.clear();
    // poly_seg.sampleOneSeg(&vis_x);
    // vis_ptr_->visualizeStates(vis_x, BLUE, pos_checker_ptr_->getLocalTime());

    bool vel_cons = poly_seg.checkMaxVelRate(vel_limit_);
    bool acc_cons = poly_seg.checkMaxAccRate(acc_limit_);
    bool jerk_cons = poly_seg.checkMaxJerkRate(jerk_limit_);
    bool pos_cons = getTraversalLines(poly_seg, traversal_lines);
    if (vel_cons && acc_cons && jerk_cons && pos_cons)
    {
      ROS_WARN("Best traj collision free, one shot connected");
      RRTNodePtr goal_node_ptr = addTreeNode(start_node_, goal_node_->x, poly_seg, best_cost, best_tau);
      // RRTNodePtr goal_node_ptr = node_pool_[valid_start_tree_node_nums_++];
      // goal_node_ptr->cost_from_start = best_cost;
      // goal_node_ptr->parent = start_node_;
      // goal_node_ptr->tau_from_start = best_tau;
      // goal_node_ptr->poly_seg = poly_seg;
      // goal_node_ptr->cost_from_parent = best_cost;
      // goal_node_ptr->tau_from_parent = best_tau;
      fillTraj(goal_node_ptr, traj_);
      fillTraj(goal_node_ptr, first_traj_);
      final_traj_use_time_ = (ros::Time::now() - t_start_).toSec();
      first_traj_use_time_ = final_traj_use_time_;
  
      /* for traj vis */
      // vis_x.clear();
      // traj_.sampleWholeTrajectory(&vis_x);
      // vis_ptr_->visualizeStates(vis_x, GRAY, pos_checker_ptr_->getLocalTime());
      // double final_traj_len(0.0), final_traj_duration(0.0), final_traj_acc_itg(0.0), final_traj_jerk_itg(0.0);
      // int final_traj_seg_nums(0);
      // evaluateTraj(traj_, final_traj_duration, final_traj_len, final_traj_seg_nums, final_traj_acc_itg, final_traj_jerk_itg);
      // ROS_INFO_STREAM("[BIKRRT]: [front-end final path]: " << endl 
      //     << "    -   seg nums: " << final_traj_seg_nums * 1e3 << " ms" << endl 
      //     << "    -   time: " << final_traj_use_time * 1e3 << " ms" << endl 
      //     << "    -   acc integral: " << final_traj_acc_itg << endl
      //     << "    -   jerk integral: " << final_traj_jerk_itg << endl
      //     << "    -   traj duration: " << final_traj_duration << endl 
      //     << "    -   path length: " << final_traj_len);
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

  int n = tree_node_nums_ - 20; // reserved for new node when two trees connects
  return rrtStar(start_node_->x, goal_node_->x, n, search_time, radius_cost_between_two_states_, rewire_);
}

int BIKRRT::rrtStar(const StatePVA& x_init, const StatePVA& x_final, int n, double search_time, double radius, const bool rewire)
{ 
  ros::Time rrt_start_time = ros::Time::now();
  ros::Time first_goal_found_time, final_goal_found_time;
  double first_general_cost(0.0);
  double curr_best_solution_cost(DBL_MAX);
  RRTNodePtr bridge_node_start_tree(nullptr), bridge_node_goal_tree(nullptr);
  unordered_set<RRTNodePtr> curr_traj_node_set_start_tree, curr_traj_node_set_goal_tree;
  
  /* local variables */
  valid_sample_nums_ = 0; //random samples in obs free area
  vector<StatePVA> vis_x;
  vis_x.reserve(3000);
  vector<StatePVA> region_traj_vis_x;
  bool first_time_find_goal(true);
  bool close_goal_found(false);
  double close_dist(DBL_MAX);
  bool goal_found(false);
  traj_list_.clear();
  solution_cost_list_.clear();
  solution_time_list_.clear();

  /* kd tree init */
  kdtree *kd_tree = kd_create(3);
  //Add start and goal nodes to kd tree
  kd_insert3(kd_tree, start_node_->x[0], start_node_->x[1], start_node_->x[2], start_node_);
  kd_insert3(kd_tree, goal_node_->x[0], goal_node_->x[1], goal_node_->x[2], goal_node_);

  //TODO changable radius
  double tau_for_instance = radius * 0.75; //maximum
  double fwd_radius_p = getForwardRadius(tau_for_instance, radius);  
  double bcwd_radius_p = getBackwardRadius(tau_for_instance, radius);
  // ROS_INFO_STREAM("bcwd_radius_p: " << bcwd_radius_p);
  // ROS_INFO_STREAM("fwd_radius_p: " << fwd_radius_p);

  /* main loop */
  vector<StatePVA> samples, valid_samples;
  int idx = 0;
  for (idx = 0; (ros::Time::now() - rrt_start_time).toSec() < search_time && valid_start_tree_node_nums_ < n ; ++idx) 
  {
    /* biased random sampling */
    StatePVA x_rand;
    bool good_sample = sampler_.samplingOnce(idx, x_rand);
    // samples.push_back(x_rand);
    if (!good_sample) 
    {
      continue;
    }
    /* make sure that nearby nodes are in different lattices */
    struct kdres *p_nbr_set = kd_nearest_range3(kd_tree, x_rand[0], x_rand[1], x_rand[2], 0.1);
    if (p_nbr_set == nullptr)
    {
      ROS_ERROR("kd range query error");
      break;
    }
    while(!kd_res_end(p_nbr_set)) 
    {
      RRTNodePtr curr_node = (RRTNodePtr)kd_res_item_data(p_nbr_set);
      Vector3d dir1(x_rand[3], x_rand[4], x_rand[5]);
      Vector3d dir2(curr_node->x[3], curr_node->x[4], curr_node->x[5]);
      double angle_diff();
      if (dir1.dot(dir2) > 0) 
      {
        good_sample = false;
        break;
      }
      kd_res_next(p_nbr_set);
    }
    kd_res_free(p_nbr_set); 
    if (!good_sample) 
    {
      continue;
    }
    /* make sure that nearby nodes are in different lattices */

    valid_samples.push_back(x_rand);
    ++valid_sample_nums_;
    
    /* choose parent from kd tree range query result*/
    double min_dist_start_tree(DBL_MAX), min_dist_goal_tree(DBL_MAX);
    double tau_from_s_start_tree(DBL_MAX), tau_from_s_goal_tree(DBL_MAX);
    double cost_from_p_start_tree(0.0), cost_from_p_goal_tree(0.0);
    double tau_from_p_start_tree(0.0), tau_from_p_goal_tree(0.0);
    RRTNode *x_near_start_tree(nullptr), *x_near_goal_tree(nullptr); //parent
    Piece find_parent_seg_start_tree, find_parent_seg_goal_tree;
    priority_queue<regionalCandidate> regional_candidate_queue_start_tree, regional_candidate_queue_goal_tree;
    // vector<RRTNode*> regional_parents_start_tree, regional_parents_goal_tree; //parents for regional optimization
    // vector<pair<Vector3d, Vector3d>> collide_pts_start_tree, collide_pts_goal_tree;
    // vector<pair<double, double>> collide_timestamp_start_tree, collide_timestamp_goal_tree;
    // vector<Piece> regional_seg_start_tree, regional_seg_goal_tree;
     
    /* kd_tree bounds search for parent */
    struct kdres *p_bcwd_nbr_set;
    p_bcwd_nbr_set = getBackwardNeighbour(x_rand, kd_tree, radius - tau_for_instance, bcwd_radius_p);
    if (p_bcwd_nbr_set == nullptr)
    {
      ROS_ERROR("bkwd kd range query error");
      break;
    }
    while(!kd_res_end(p_bcwd_nbr_set)) 
    {
      RRTNodePtr curr_node = (RRTNodePtr)kd_res_item_data(p_bcwd_nbr_set);
      if (curr_node->tree_type == START_TREE) 
      { 
        if (bvp_.solve(curr_node->x, x_rand, ACC_UNKNOWN))
        {
          CoefficientMat coeff;
          bvp_.getCoeff(coeff);
          Piece seg_from_parent = Piece(bvp_.getTauStar(), coeff);
          // bool connected = checkSegmentConstraints(seg_from_parent);
          bool vel_cons = seg_from_parent.checkMaxVelRate(vel_limit_);
          bool acc_cons = seg_from_parent.checkMaxAccRate(acc_limit_);
          bool jerk_cons = seg_from_parent.checkMaxJerkRate(jerk_limit_);
          pair<Vector3d, Vector3d> collide_pts_one_seg;
          pair<double, double> t_s_e;
          bool need_region_opt(false);
          bool pos_cons = pos_checker_ptr_->checkPolySeg(seg_from_parent, collide_pts_one_seg, t_s_e, need_region_opt);
          bool connected = vel_cons && acc_cons && jerk_cons && pos_cons;
          if (connected) 
          {
            if (min_dist_start_tree > (curr_node->cost_from_start + bvp_.getCostStar()))
            {
              cost_from_p_start_tree = bvp_.getCostStar();
              tau_from_p_start_tree = bvp_.getTauStar();
              min_dist_start_tree = curr_node->cost_from_start + cost_from_p_start_tree;
              tau_from_s_start_tree = curr_node->tau_from_start + tau_from_p_start_tree;
              find_parent_seg_start_tree = seg_from_parent;
              x_near_start_tree = curr_node;
            } 
          }
          else if (!goal_found && need_region_opt && use_regional_opt_ && seg_from_parent.getAcc(bvp_.getTauStar()).norm() < acc_limit_)
          {
            // double dis = (curr_node->x - x_rand).head(3).norm();
            // only 2m < distance with potential parent < 5m and collision duration < 0.5 are stored to be optimized
            // if (dis < 5.0 && dis > 2.0 && (t_s_e.second - t_s_e.first) <= 0.5 && ((x_rand-x_init).head(3).norm() - (curr_node->x-x_init).head(3).norm()) > 3) 
            {
              //find segs to be regionally optimized
              regional_candidate_queue_start_tree.emplace(curr_node, collide_pts_one_seg, t_s_e, seg_from_parent, bvp_.getCostStar());
              // regional_parents_start_tree.push_back(curr_node);
              // collide_pts_start_tree.push_back(collide_pts_one_seg);
              // regional_seg_start_tree.push_back(seg_from_parent);
              // collide_timestamp_start_tree.push_back(t_s_e);
              // ROS_WARN_STREAM("region parent: " << curr_node->x.transpose());
              // ROS_WARN_STREAM("collides: " << collide_pts_one_seg.first.transpose() << ",  " << collide_pts_one_seg.second.transpose());
            }
          }
        }
        else
        {
          ROS_ERROR("sth. wrong with the bvp solver");
        }
      }
      // else 
      // {
      //   if (bvp_.solve(x_rand, curr_node->x, ACC_KNOWN))
      //   {
      //     CoefficientMat coeff;
      //     bvp_.getCoeff(coeff);
      //     Piece seg_to_parent = Piece(bvp_.getTauStar(), coeff);
      //     // bool connected = checkSegmentConstraints(seg_to_parent);
      //     bool vel_cons = seg_to_parent.checkMaxVelRate(vel_limit_);
      //     bool acc_cons = seg_to_parent.checkMaxAccRate(acc_limit_);
      //     bool jerk_cons = seg_to_parent.checkMaxJerkRate(jerk_limit_);
      //     pair<Vector3d, Vector3d> collide_pts_one_seg;
      //     pair<double, double> t_s_e;
      //     bool need_region_opt(false);
      //     bool pos_cons = pos_checker_ptr_->checkPolySeg(seg_to_parent, collide_pts_one_seg, t_s_e, need_region_opt);
      //     bool connected = vel_cons && acc_cons && jerk_cons && pos_cons;
      //     if (connected) 
      //     {
      //       if (min_dist_goal_tree > (curr_node->cost_from_start + bvp_.getCostStar()))
      //       {
      //         cost_from_p_goal_tree = bvp_.getCostStar();
      //         tau_from_p_goal_tree = bvp_.getTauStar();
      //         min_dist_goal_tree = curr_node->cost_from_start + cost_from_p_goal_tree;
      //         tau_from_s_goal_tree = curr_node->tau_from_start + tau_from_p_goal_tree;
      //         find_parent_seg_goal_tree = seg_to_parent;
      //         x_near_goal_tree = curr_node;
      //       } 
      //     }
      //     else if (!goal_found && need_region_opt && use_regional_opt_)
      //     {
      //       // double dis = (curr_node->x - x_rand).head(3).norm();
      //       // only 2m < distance with potential parent < 5m and collision duration < 0.5 are stored to be optimized
      //       // if (dis < 5.0 && dis > 2.0 && (t_s_e.second - t_s_e.first) <= 0.5 && ((x_rand-x_final).head(3).norm() - (curr_node->x-x_final).head(3).norm()) > 3) 
      //       {
      //         //find segs to be regionally optimized
      //         regional_candidate_queue_goal_tree.emplace(curr_node, collide_pts_one_seg, t_s_e, seg_to_parent, bvp_.getTauStar());
      //         // regional_parents_goal_tree.push_back(curr_node);
      //         // collide_pts_goal_tree.push_back(collide_pts_one_seg);
      //         // regional_seg_goal_tree.push_back(seg_to_parent);
      //         // collide_timestamp_goal_tree.push_back(t_s_e);
      //         // ROS_WARN_STREAM("region parent: " << curr_node->x.transpose());
      //         // ROS_WARN_STREAM("collides: " << collide_pts_one_seg.first.transpose() << ",  " << collide_pts_one_seg.second.transpose());
      //       }
      //     }
      //   }
      //   else
      //   {
      //     ROS_ERROR("sth. wrong with the bvp solver");
      //   }
      // }
      kd_res_next(p_bcwd_nbr_set); //go to next in kd tree range query result
    }
    kd_res_free(p_bcwd_nbr_set); //reset kd tree range query

    // size_t n_r_p_start_tree = regional_parents_start_tree.size();
    RRTNode* sampled_node_start_tree(nullptr);
    // if (x_near_start_tree == nullptr && n_r_p_start_tree > 0) 
    if (x_near_start_tree == nullptr && !regional_candidate_queue_start_tree.empty()) 
    {
      bool promising_node(true);
      Trajectory local_opt_traj;
      // for (size_t i = 0; i < n_r_p_start_tree; ++i)
      while(!regional_candidate_queue_start_tree.empty())
      {
        regionalCandidate curr_regional_candidate = regional_candidate_queue_start_tree.top();
        regional_candidate_queue_start_tree.pop();
        ros::Time r_o_ts = ros::Time::now();
        // bool regional_opt_result = regionalOpt(regional_seg_start_tree[i], collide_pts_start_tree[i], collide_timestamp_start_tree[i]);
        bool regional_opt_result = regionalOpt(curr_regional_candidate.regional_seg, curr_regional_candidate.collide_pts, curr_regional_candidate.collide_timestamp);
        // bool regional_opt_result = stOpt(curr_regional_candidate.regional_seg, curr_regional_candidate.parent, local_opt_traj);
        // ROS_ERROR_STREAM("regional_opt costs " << 1e3 * (ros::Time::now() - r_o_ts).toSec() << " ms. Result: " << regional_opt_result);
        // getchar();
        if (regional_opt_result)
        {
          optimizer_ptr_->getTraj(local_opt_traj);

          /* regional optimization succeeds when find parent, then:
           * 1. add nodes of the local traj to rrt and kd_tree; 
           */
          //sample rejection
          int n_mid_with_last_pt = local_opt_traj.getPieceNum();
          double cost[n_mid_with_last_pt];
          x_rand.tail(3) = local_opt_traj.getAcc(local_opt_traj.getTotalDuration());
          if(bvp_.solve(x_rand, goal_node_->x, ACC_KNOWN))
          {
            double regional_traj_cost = local_opt_traj.calCost(rho_, cost);
            if (curr_regional_candidate.parent->cost_from_start + regional_traj_cost + bvp_.getCostStar() >= curr_best_solution_cost) 
            {
              // ROS_WARN("regionally optimized but sample rejected");
              promising_node = false;
            }
          }
          else
          {
            ROS_ERROR("sth. wrong with the bvp solver");
          }

          if (debug_vis_)
            local_opt_traj.sampleWholeTrajectory(&region_traj_vis_x);
          // ROS_WARN("regional optimized");
          if (promising_node)
          {
            StatePVA mid_x; 
            Vector3d pos, vel, acc;
            RRTNode *curr_node_in_regional_traj(nullptr), *last_node(nullptr);
            for (int j = 0; j < n_mid_with_last_pt; j++)
            {
              double tau = local_opt_traj[j].getDuration();
              pos = local_opt_traj[j].getPos(tau);
              vel = local_opt_traj[j].getVel(tau);
              acc = local_opt_traj[j].getAcc(tau);
              mid_x.segment(0, 3) = pos;
              mid_x.segment(3, 3) = vel;
              mid_x.segment(6, 3) = acc;
              /* 1.1 add the randomly sampled node to rrt_tree */
              if (j == 0) 
              {
                curr_node_in_regional_traj = addTreeNode(curr_regional_candidate.parent, mid_x, local_opt_traj[j], cost[j], tau);
                curr_node_in_regional_traj->tree_type = START_TREE;
              }
              else 
              {
                curr_node_in_regional_traj = addTreeNode(last_node, mid_x, local_opt_traj[j], cost[j], tau);
                curr_node_in_regional_traj->tree_type = START_TREE;
              }
              last_node = curr_node_in_regional_traj;
            }
            /* 1.2 add the randomly sampled node to kd_tree */
            kd_insert3(kd_tree, mid_x[0], mid_x[1], mid_x[2], curr_node_in_regional_traj);
            sampled_node_start_tree = curr_node_in_regional_traj;
            break;
          }
        }
      }
    } 
    else if (x_near_start_tree != nullptr)
    {
      /* parent found within radius, then add a node to rrt and kd_tree */
      //sample rejection
      bool promising_node(true);
      Vector3d calculated_acc = find_parent_seg_start_tree.getAcc(tau_from_p_start_tree);
      if (calculated_acc.norm() >= acc_limit_)
        promising_node = false;
      x_rand.tail(3) = calculated_acc;
      if(bvp_.solve(x_rand, goal_node_->x, ACC_KNOWN))
      {
        if (min_dist_start_tree + bvp_.getCostStar() >= curr_best_solution_cost) 
        {
          // ROS_WARN("parent found but sample rejected");
          promising_node = false;
        }
      }
      else
      {
        ROS_ERROR("sth. wrong with the bvp solver");
      }
      if (promising_node)
      {
        /* 1.1 add the randomly sampled node to rrt_tree */
        sampled_node_start_tree = addTreeNode(x_near_start_tree, x_rand, find_parent_seg_start_tree, min_dist_start_tree, tau_from_s_start_tree, cost_from_p_start_tree, tau_from_p_start_tree);
        sampled_node_start_tree->tree_type = START_TREE;

        /* 1.2 add the randomly sampled node to kd_tree */
        kd_insert3(kd_tree, x_rand[0], x_rand[1], x_rand[2], sampled_node_start_tree);
      }
    }

    /* kd_tree bounds search for parent */
    struct kdres *p_fwd_nbr_set;
    p_fwd_nbr_set = getForwardNeighbour(x_rand, kd_tree, tau_for_instance, fwd_radius_p);
    if (p_fwd_nbr_set == nullptr)
    {
      ROS_ERROR("search parent in goal tree, fwd kd range query error");
      break;
    }
    while(!kd_res_end(p_fwd_nbr_set)) 
    {
      RRTNodePtr curr_node = (RRTNodePtr)kd_res_item_data(p_fwd_nbr_set);
      if (curr_node->tree_type == GOAL_TREE)
      {
        if (bvp_.solve(x_rand, curr_node->x, INITIAL_ACC_UNKNOWN))
        {
          CoefficientMat coeff;
          bvp_.getCoeff(coeff);
          Piece seg_to_parent = Piece(bvp_.getTauStar(), coeff);
          // bool connected = checkSegmentConstraints(seg_to_parent);
          bool vel_cons = seg_to_parent.checkMaxVelRate(vel_limit_);
          bool acc_cons = seg_to_parent.checkMaxAccRate(acc_limit_);
          bool jerk_cons = seg_to_parent.checkMaxJerkRate(jerk_limit_);
          pair<Vector3d, Vector3d> collide_pts_one_seg;
          pair<double, double> t_s_e;
          bool need_region_opt(false);
          bool pos_cons = pos_checker_ptr_->checkPolySeg(seg_to_parent, collide_pts_one_seg, t_s_e, need_region_opt);
          bool connected = vel_cons && acc_cons && jerk_cons && pos_cons;
          if (connected) 
          {
            if (min_dist_goal_tree > (curr_node->cost_from_start + bvp_.getCostStar()))
            {
              cost_from_p_goal_tree = bvp_.getCostStar();
              tau_from_p_goal_tree = bvp_.getTauStar();
              min_dist_goal_tree = curr_node->cost_from_start + cost_from_p_goal_tree;
              tau_from_s_goal_tree = curr_node->tau_from_start + tau_from_p_goal_tree;
              find_parent_seg_goal_tree = seg_to_parent;
              x_near_goal_tree = curr_node;
            } 
          }
          else if (!goal_found && need_region_opt && use_regional_opt_)
          {
            // double dis = (curr_node->x - x_rand).head(3).norm();
            // only 2m < distance with potential parent < 5m and collision duration < 0.5 are stored to be optimized
            // if (dis < 5.0 && dis > 2.0 && (t_s_e.second - t_s_e.first) <= 0.5 && ((x_rand-x_final).head(3).norm() - (curr_node->x-x_final).head(3).norm()) > 3) 
            {
              //find segs to be regionally optimized
              regional_candidate_queue_goal_tree.emplace(curr_node, collide_pts_one_seg, t_s_e, seg_to_parent, bvp_.getTauStar());
              // regional_parents_goal_tree.push_back(curr_node);
              // collide_pts_goal_tree.push_back(collide_pts_one_seg);
              // regional_seg_goal_tree.push_back(seg_to_parent);
              // collide_timestamp_goal_tree.push_back(t_s_e);
              // ROS_WARN_STREAM("region parent: " << curr_node->x.transpose());
              // ROS_WARN_STREAM("collides: " << collide_pts_one_seg.first.transpose() << ",  " << collide_pts_one_seg.second.transpose());
            }
          }
        }
        else
        {
          ROS_ERROR("sth. wrong with the bvp solver");
        }
      }
      kd_res_next(p_fwd_nbr_set); //go to next in kd tree range query result
    }
    kd_res_free(p_fwd_nbr_set); //reset kd tree range query

    RRTNode *sampled_node_goal_tree(nullptr);
    // size_t n_r_p_goal_tree = regional_parents_goal_tree.size();
    // if (x_near_goal_tree == nullptr && n_r_p_goal_tree > 0) 
    if (x_near_goal_tree == nullptr && !regional_candidate_queue_goal_tree.empty()) 
    {
      bool promising_node(true);
      Trajectory local_opt_traj;
      // for (size_t i = 0; i < n_r_p_goal_tree; ++i)
      while (!regional_candidate_queue_goal_tree.empty())
      {
        regionalCandidate curr_regional_candidate = regional_candidate_queue_goal_tree.top();
        regional_candidate_queue_goal_tree.pop();
        ros::Time r_o_ts = ros::Time::now();
        // bool regional_opt_result = regionalOpt(regional_seg_goal_tree[i], collide_pts_goal_tree[i], collide_timestamp_goal_tree[i]);
        bool regional_opt_result = regionalOpt(curr_regional_candidate.regional_seg, curr_regional_candidate.collide_pts, curr_regional_candidate.collide_timestamp);
        // bool regional_opt_result = stOpt(curr_regional_candidate.regional_seg, curr_regional_candidate.parent, local_opt_traj);
        // ROS_ERROR_STREAM("regional_opt costs " << 1e3 * (ros::Time::now() - r_o_ts).toSec() << " ms. Result: " << regional_opt_result);
        // getchar();
        if (regional_opt_result)
        {
          optimizer_ptr_->getTraj(local_opt_traj);

          /* regional optimization succeeds when find parent, then:
           * 1. add nodes of the local traj to rrt and kd_tree; 
           */
          //sample rejection
          int n_first_pt_with_mid = local_opt_traj.getPieceNum();
          double cost[n_first_pt_with_mid];
          x_rand.tail(3) = local_opt_traj.getAcc(0.0);
          if(bvp_.solve(start_node_->x, x_rand, ACC_KNOWN))
          {
            double regional_traj_cost = local_opt_traj.calCost(rho_, cost);
            if (curr_regional_candidate.parent->cost_from_start + regional_traj_cost + bvp_.getCostStar() >= curr_best_solution_cost) 
            {
              // ROS_WARN("regionally optimized but sample rejected");
              promising_node = false;
            }
          }
          else
          {
            ROS_ERROR("sth. wrong with the bvp solver");
          }

          if (debug_vis_)
            local_opt_traj.sampleWholeTrajectory(&region_traj_vis_x);
          // ROS_WARN("regional optimized");
          if (promising_node)
          {
            StatePVA mid_x; 
            Vector3d pos, vel, acc;
            RRTNode *curr_node_in_regional_traj(nullptr), *last_node(nullptr);
            for (int j = n_first_pt_with_mid - 1; j >= 0; j--)
            {
              double tau = local_opt_traj[j].getDuration();
              pos = local_opt_traj[j].getPos(0);
              vel = local_opt_traj[j].getVel(0);
              acc = local_opt_traj[j].getAcc(0);
              mid_x.segment(0, 3) = pos;
              mid_x.segment(3, 3) = vel;
              mid_x.segment(6, 3) = acc;
              /* 1.1 add the randomly sampled node to rrt_tree */
              if (j == n_first_pt_with_mid - 1) 
              {
                curr_node_in_regional_traj = addTreeNode(curr_regional_candidate.parent, mid_x, local_opt_traj[j], cost[j], tau);
                curr_node_in_regional_traj->tree_type = GOAL_TREE;
              }
              else 
              {
                curr_node_in_regional_traj = addTreeNode(last_node, mid_x, local_opt_traj[j], cost[j], tau);
                curr_node_in_regional_traj->tree_type = GOAL_TREE;
              }
              last_node = curr_node_in_regional_traj;
            }
            /* 1.2 add the randomly sampled node to kd_tree */
            kd_insert3(kd_tree, mid_x[0], mid_x[1], mid_x[2], curr_node_in_regional_traj);
            sampled_node_goal_tree = curr_node_in_regional_traj;
            break;
          }
        }
      }
    } 
    else if (x_near_goal_tree != nullptr)
    {
      /* parent found within radius, then add a node to rrt and kd_tree */
      //sample rejection
      bool promising_node(true);
      Vector3d calculated_acc = find_parent_seg_goal_tree.getAcc(0.0);
      if (calculated_acc.norm() >= acc_limit_)
        promising_node = false;
      x_rand.tail(3) = calculated_acc;
      if(bvp_.solve(start_node_->x, x_rand, ACC_KNOWN))
      {
        if (min_dist_goal_tree + bvp_.getCostStar() >= curr_best_solution_cost) 
        {
          // ROS_WARN("parent found but sample rejected");
          promising_node = false;
        }
      }
      else
      {
        ROS_ERROR("sth. wrong with the bvp solver");
      }
      if (promising_node)
      {
        /* 1.1 add the randomly sampled node to rrt_tree */
        sampled_node_goal_tree = addTreeNode(x_near_goal_tree, x_rand, find_parent_seg_goal_tree, min_dist_goal_tree, tau_from_s_goal_tree, cost_from_p_goal_tree, tau_from_p_goal_tree);
        sampled_node_goal_tree->tree_type = GOAL_TREE;

        /* 1.2 add the randomly sampled node to kd_tree */
        kd_insert3(kd_tree, x_rand[0], x_rand[1], x_rand[2], sampled_node_goal_tree);
      }
    }

    if (sampled_node_goal_tree != nullptr)
    {
      if (rewire) 
      {
        //kd_tree bounds search
        struct kdres *p_fwd_nbr_set;
        p_fwd_nbr_set = getBackwardNeighbour(x_rand, kd_tree, radius - tau_for_instance, bcwd_radius_p);
        if (p_fwd_nbr_set == nullptr)
        {
          ROS_ERROR("fwd kd range query error");
          break;
        }
        while(!kd_res_end(p_fwd_nbr_set)) 
        {
          RRTNode* curr_node = (RRTNode*)kd_res_item_data(p_fwd_nbr_set);
          if (curr_node->tree_type == START_TREE || curr_node == goal_node_) 
          {
            kd_res_next(p_fwd_nbr_set);
            continue;
          }
          Piece seg_rewire;
          if(bvp_.solve(curr_node->x, x_rand, ACC_KNOWN))
          {
            CoefficientMat coeff;
            bvp_.getCoeff(coeff);
            seg_rewire = Piece(bvp_.getTauStar(), coeff);
          }
          else
          {
            ROS_ERROR("sth. wrong with the bvp solver");
          }
          bool connected = checkSegmentConstraints(seg_rewire);
          if (connected && sampled_node_goal_tree->cost_from_start + bvp_.getCostStar() < curr_node->cost_from_start) 
          {
            // If we can get to a node via the sampled_node faster than via it's existing parent then change the parent
            RRTNodePtr old_parent(curr_node->parent);
            changeNodeParent(curr_node, sampled_node_goal_tree, seg_rewire, bvp_.getCostStar(), bvp_.getTauStar());
            if (test_convergency_ && goal_found)
            {
              if (curr_traj_node_set_goal_tree.count(curr_node) > 0) // rewire a node in curr best traj
              {
                curr_traj_node_set_goal_tree.insert(sampled_node_goal_tree);
                curr_traj_node_set_goal_tree.erase(old_parent);
                curr_best_solution_cost = bridge_node_start_tree->cost_from_start + bridge_node_goal_tree->cost_from_start;
                ros::Time curr_goal_found_time = ros::Time::now();
                double curr_traj_use_time_ = (ros::Time::now() - t_start_).toSec();
                Trajectory traj;
                fillTraj(bridge_node_start_tree, bridge_node_goal_tree, traj);
                traj_list_.emplace_back(traj);
                solution_cost_list_.emplace_back(curr_best_solution_cost);
                solution_time_list_.emplace_back(curr_traj_use_time_);
                ROS_WARN_STREAM("curr cost: " << curr_best_solution_cost);
              }
            }
          }
          /* go to the next entry */
          kd_res_next(p_fwd_nbr_set);
        }
        kd_res_free(p_fwd_nbr_set);
      }/* end of rewire */
    }
    /* end of find parent */

    if (sampled_node_goal_tree != nullptr && sampled_node_start_tree != nullptr) 
    {
      if (curr_best_solution_cost <= sampled_node_start_tree->cost_from_start + sampled_node_goal_tree->cost_from_start)
      {
        continue;
      }
      curr_best_solution_cost = sampled_node_start_tree->cost_from_start + sampled_node_goal_tree->cost_from_start;
      bridge_node_start_tree = sampled_node_start_tree;
      bridge_node_goal_tree = sampled_node_goal_tree;
      goal_found = true;

      if (test_convergency_)
      {
        ros::Time curr_goal_found_time = ros::Time::now();
        double curr_traj_use_time_ = (ros::Time::now() - t_start_).toSec();
        Trajectory traj;
        fillTraj(bridge_node_start_tree, bridge_node_goal_tree, traj);
        traj_list_.emplace_back(traj);
        solution_cost_list_.emplace_back(curr_best_solution_cost);
        solution_time_list_.emplace_back(curr_traj_use_time_);
        ROS_WARN_STREAM("curr cost: " << curr_best_solution_cost);

        curr_traj_node_set_start_tree.clear();
        curr_traj_node_set_goal_tree.clear();
        RRTNodePtr node(bridge_node_start_tree);
        while (node->parent)
        {
          curr_traj_node_set_start_tree.insert(node);
          node = node->parent;
        }
        node = bridge_node_goal_tree;
        while (node->parent)
        {
          curr_traj_node_set_goal_tree.insert(node);
          node = node->parent;
        }
      }
      if (first_time_find_goal) 
      {
        first_general_cost = curr_best_solution_cost;
        first_goal_found_time = ros::Time::now();
        first_traj_use_time_ = (first_goal_found_time - t_start_).toSec();
        first_time_find_goal = false;
        fillTraj(bridge_node_start_tree, bridge_node_goal_tree, first_traj_);

        RRTNodePtr node(bridge_node_start_tree);
        while (node->parent)
        {
          curr_traj_node_set_start_tree.insert(node);
          node = node->parent;
        }
        node = bridge_node_goal_tree;
        while (node->parent)
        {
          curr_traj_node_set_goal_tree.insert(node);
          node = node->parent;
        }
      }
      if (stop_after_first_traj_found_) 
        break; //stop searching after first time find the goal?
    }
    /* end of try to connect to goal */

    // vis_x.clear();
    // vector<Vector3d> knots;
    // sampleWholeTree(start_node_, &vis_x, knots);
    // sampleWholeTree(goal_node_, &vis_x, knots);
    // vis_ptr_->visualizeStates(vis_x, TreeTraj, pos_checker_ptr_->getLocalTime());

  }/* end of sample once */
  t_end_ = ros::Time::now();

  // vis_x.clear();
  // vector<Vector3d> knots;
  // sampleWholeTree(start_node_, &vis_x, knots);
  // sampleWholeTree(goal_node_, &vis_x, knots);
  // vis_ptr_->visualizeStates(vis_x, TreeTraj, pos_checker_ptr_->getLocalTime());
  // vis_ptr_->visualizeKnots(knots, pos_checker_ptr_->getLocalTime());
  // // vis_ptr_->visualizeSampledState(samples, pos_checker_ptr_->getLocalTime());
  // vis_ptr_->visualizeSampledState(valid_samples, pos_checker_ptr_->getLocalTime());
  // vis_ptr_->visualizePoints(region_traj_vis_x, pos_checker_ptr_->getLocalTime());

  if (goal_found) 
  {
    final_traj_use_time_ = (t_end_ - t_start_).toSec();
    fillTraj(bridge_node_start_tree, bridge_node_goal_tree, traj_);  
    /* for traj vis */
    ROS_INFO_STREAM("[BIKRRT]: total sample times: " << idx);
    ROS_INFO_STREAM("[BIKRRT]: valid sample times: " << valid_sample_nums_);
    ROS_INFO_STREAM("[BIKRRT]: valid tree node nums: " << valid_start_tree_node_nums_);
    return SUCCESS;
  } 
  else if (allow_close_goal_ && valid_start_tree_node_nums_ > 2)
  {
    ROS_WARN("Not connectting to goal, choose a bypass");
    t_end_ = ros::Time::now();
    chooseBypass(close_goal_node_, start_node_);
    fillTraj(close_goal_node_, traj_);  
    /* for traj vis */
    vis_x.clear();
    traj_.sampleWholeTrajectory(&vis_x);
    vis_ptr_->visualizeStates(vis_x, GRAY, pos_checker_ptr_->getLocalTime());
    
    double final_traj_len(0.0), final_traj_duration(0.0), final_traj_acc_itg(0.0), final_traj_jerk_itg(0.0);
    int final_traj_seg_nums(0);
    evaluateTraj(traj_, final_traj_duration, final_traj_len, final_traj_seg_nums, final_traj_acc_itg, final_traj_jerk_itg);
    final_traj_use_time_ = t_end_.toSec()-t_start_.toSec();
    ROS_INFO_STREAM("[BIKRRT]: total sample times: " << idx);
    ROS_INFO_STREAM("[BIKRRT]: valid sample times: " << valid_sample_nums_);
    ROS_INFO_STREAM("[BIKRRT]: valid tree node nums: " << valid_start_tree_node_nums_);
    ROS_INFO_STREAM("[BIKRRT]: [front-end final path]: " << endl 
         << "    -   seg nums: " << final_traj_seg_nums << endl 
         << "    -   time: " << final_traj_use_time_ * 1e3 << " ms"<< endl 
         << "    -   acc integral: " << final_traj_acc_itg << endl
         << "    -   jerk integral: " << final_traj_jerk_itg << endl
         << "    -   traj duration: " << final_traj_duration << endl 
         << "    -   path length: " << final_traj_len << " m" << endl 
         << "    -   general cost: " << close_goal_node_->cost_from_start);
    return SUCCESS_CLOSE_GOAL;
  }
  else if (valid_start_tree_node_nums_ == n)
  {
    ROS_ERROR_STREAM("[BIKRRT]: NOT CONNECTED TO GOAL after " << n << " nodes added to rrt-tree");
    ROS_INFO_STREAM("[BIKRRT]: total sample times: " << idx);
    ROS_INFO_STREAM("[BIKRRT]: valid sample times: " << valid_sample_nums_);
    ROS_INFO_STREAM("[BIKRRT]: valid tree node nums: " << valid_start_tree_node_nums_);
    return FAILURE;
  }
  else if ((ros::Time::now() - rrt_start_time).toSec() >= search_time)
  {
    ROS_ERROR_STREAM("[BIKRRT]: NOT CONNECTED TO GOAL after " << (ros::Time::now() - rrt_start_time).toSec() << " seconds");
    ROS_INFO_STREAM("[BIKRRT]: total sample times: " << idx);
    ROS_INFO_STREAM("[BIKRRT]: valid sample times: " << valid_sample_nums_);
    ROS_INFO_STREAM("[BIKRRT]: valid tree node nums: " << valid_start_tree_node_nums_);
    return FAILURE;
  }
  else
  {
    return FAILURE;
  }
  
}

double BIKRRT::getForwardRadius(double tau, double cost)
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
  return radius_p * 1.732;
}

double BIKRRT::getBackwardRadius(double tau, double cost)
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
  return radius_p * 1.732;
}

struct kdres *BIKRRT::getForwardNeighbour(const StatePVA& x0, struct kdtree *kd_tree, double tau, double radius_p)
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

struct kdres *BIKRRT::getBackwardNeighbour(const StatePVA& x1, struct kdtree *kd_tree, double tau, double radius_p)
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

inline RRTNodePtr BIKRRT::addTreeNode(RRTNodePtr& parent, const StatePVA& state, const Piece& piece, 
                                           const double& cost_from_start, const double& tau_from_start, 
                                           const double& cost_from_parent, const double& tau_from_parent)
{
  RRTNodePtr new_node_ptr = node_pool_[valid_start_tree_node_nums_++];
  new_node_ptr->parent = parent;
  parent->children.push_back(new_node_ptr);
  new_node_ptr->x = state;
  new_node_ptr->poly_seg = piece;
  new_node_ptr->cost_from_start = cost_from_start;
  new_node_ptr->tau_from_start = tau_from_start;
  new_node_ptr->cost_from_parent = cost_from_parent;
  new_node_ptr->tau_from_parent = tau_from_parent;
  return new_node_ptr;
}

inline RRTNodePtr BIKRRT::addTreeNode(RRTNodePtr& parent, const StatePVA& state, const Piece& piece, 
                                           const double& cost_from_parent, const double& tau_from_parent)
{
  RRTNodePtr new_node_ptr = node_pool_[valid_start_tree_node_nums_++];
  new_node_ptr->parent = parent;
  parent->children.push_back(new_node_ptr);
  new_node_ptr->x = state;
  new_node_ptr->poly_seg = piece;
  new_node_ptr->cost_from_start = parent->cost_from_start + cost_from_parent;
  new_node_ptr->tau_from_start = parent->tau_from_start + tau_from_parent;
  new_node_ptr->cost_from_parent = cost_from_parent;
  new_node_ptr->tau_from_parent = tau_from_parent;
  return new_node_ptr;
}

inline void BIKRRT::changeNodeParent(RRTNodePtr& node, RRTNodePtr& parent, const Piece& piece, 
                                           const double& cost_from_parent, const double& tau_from_parent)
{
  if (node->parent)
    node->parent->children.remove(node);  //DON'T FORGET THIS, remove it form its parent's children list
  node->parent = parent;
  node->cost_from_parent = cost_from_parent;
  node->tau_from_parent = tau_from_parent;
  node->cost_from_start = parent->cost_from_start + cost_from_parent;
  node->tau_from_start = parent->tau_from_start + tau_from_parent;
  node->poly_seg = piece;
  parent->children.push_back(node);

  // for all its descedants, change the cost_from_start and tau_from_start;
  RRTNode* descendant(node);
  std::queue<RRTNode*> Q;
  Q.push(descendant);
  while (!Q.empty()) 
  {
    descendant = Q.front();
    Q.pop();
    for (const auto& leafptr : descendant->children) 
    {
      leafptr->cost_from_start = leafptr->cost_from_parent + descendant->cost_from_start;
      leafptr->tau_from_start = leafptr->tau_from_parent + descendant->tau_from_start;
      Q.push(leafptr);
    }
  }
}

inline bool BIKRRT::regionalOpt(const Piece& oringin_seg, const pair<Vector3d, Vector3d>& collide_pts_one_seg, const pair<double, double>& t_s_e)
{
  int split_seg_num = 2;
  Trajectory pre_regional_traj;
  pre_regional_traj.reserve(split_seg_num);
  CoefficientMat coeff = oringin_seg.getCoeffMat();
  double duration = oringin_seg.getDuration() / split_seg_num;
  pre_regional_traj.emplace_back(Piece(duration, coeff));
  for (int i = 1; i < split_seg_num; ++i)
  {
    oringin_seg.cutPiece(oringin_seg, duration * i, coeff);
    pre_regional_traj.emplace_back(Piece(duration, coeff));
  }

  if (searcher_->AstarSearch(pos_checker_ptr_->getResolution(), collide_pts_one_seg.first, collide_pts_one_seg.second))
  {
    vector<Eigen::Vector3d> grid_path = searcher_->getPath();
    // vis_ptr_->visualizeKnots(grid_path, pos_checker_ptr_->getLocalTime());
    std::vector<pair<int, int>> seg_num_obs_size;  //first: # of segment in a traj; second: number of attract_pt in this segment. 
    std::vector<double> t_s, t_e; // each attract_pt's timestamp of start and end in its corresponding segment. Size should be the same as the sum of seg_num_obs_size.second.
    std::vector<Eigen::Vector3d> attract_pts;
    pos_checker_ptr_->getRegionalAttractPts(pre_regional_traj, grid_path, t_s_e, seg_num_obs_size, attract_pts, t_s, t_e);
    if (!optimizer_ptr_->initialize(pre_regional_traj, TrajOptimizer::SMOOTH_HOMO_OBS))
    {
      return false;
    }
    return (optimizer_ptr_->solveRegionalOpt(seg_num_obs_size, attract_pts, t_s, t_e));
  }
  else
  {
    return false;
  }
}

inline double BIKRRT::dist(const StatePVA& x0, const StatePVA& x1)
{
  Vector3d p_diff(x0.head(3) - x1.head(3));
  return p_diff.norm();
}

inline void BIKRRT::sampleWholeTree(const RRTNodePtr &root, vector< StatePVA >* vis_x, vector<Vector3d>& knots)
{
  if (root == nullptr) 
    return;
  //whatever dfs or bfs
  RRTNode* node = root;
  std::queue<RRTNode*> Q;
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

void BIKRRT::fillTraj(const RRTNodePtr &goal_leaf, Trajectory& traj)
{
  std::vector<double> durs;
  std::vector<CoefficientMat> coeffMats;
  RRTNodePtr node = goal_leaf;
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

void BIKRRT::fillTraj(const RRTNodePtr &bridge_node_start_tree, const RRTNodePtr &bridge_node_goal_tree, Trajectory& traj)
{
  std::vector<double> durs;
  std::vector<CoefficientMat> coeffMats;
  RRTNodePtr node = bridge_node_start_tree;
  while (node->parent) 
  {
    durs.push_back(node->poly_seg.getDuration());
    coeffMats.push_back(node->poly_seg.getCoeffMat());
    node = node->parent;
  }
  std::reverse(std::begin(durs), std::end(durs));
  std::reverse(std::begin(coeffMats), std::end(coeffMats));
  node = bridge_node_goal_tree;
  while (node->parent) 
  {
    durs.push_back(node->poly_seg.getDuration());
    coeffMats.push_back(node->poly_seg.getCoeffMat());
    node = node->parent;
  }
  traj = Trajectory(durs, coeffMats);
}

inline bool BIKRRT::checkSegmentConstraints(const Piece &seg)
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

inline bool BIKRRT::getTraversalLines(const Piece &seg, vector<pair<Vector3d, Vector3d>> &traversal_lines)
{
  bool res = pos_checker_ptr_->checkPolySeg(seg, traversal_lines);
  return res;
}

double BIKRRT::evaluateTraj(const Trajectory& traj, double& traj_duration, double& traj_length, int& seg_nums, 
                               double& acc_integral, double& jerk_integral)
{
  traj_length = 0.0;
  traj_duration = 0.0;
  seg_nums = traj.getPieceNum();
  acc_integral = 0.0;
  jerk_integral = 0.0;

  double d_t = 0.02;
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
  double c[seg_nums];
  return traj.calCost(rho_, c);
}

void BIKRRT::chooseBypass(RRTNodePtr &goal_leaf, const RRTNodePtr &tree_start_node)
{
  goal_leaf = node_pool_[valid_start_tree_node_nums_ - 1];
  double close_dist(DBL_MAX);
  for (int i = 2; i < valid_start_tree_node_nums_; ++i)
  {
    double dist_to_goal = dist(node_pool_[i]->x, goal_node_->x);
    if (dist_to_goal < close_dist)
    {
      close_dist = dist_to_goal;
      close_goal_node_ = node_pool_[i];
    }
  }
}

} //namespace kino_planner