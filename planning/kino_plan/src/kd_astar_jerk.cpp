#include <kino_plan/kd_astar_jerk.h>
#include <sstream>

using namespace std;
using namespace Eigen;

namespace tgk_planner {
KinodynamicAstarJ::~KinodynamicAstarJ() {
  for (int i = 0; i < allocate_num_; i++) {
    delete path_node_pool_[i];
  }
}

int KinodynamicAstarJ::search(Eigen::Vector3d start_pt, Eigen::Vector3d start_v,
                              Eigen::Vector3d start_a, Eigen::Vector3d end_pt,
                              Eigen::Vector3d end_v, Eigen::Vector3d end_a,
                              bool init, bool dynamic, double time_start) {
  ros::Time t_ka_s = ros::Time::now();
  start_vel_ = start_v;
  start_acc_ = start_a;

  /* ---------- initialize ---------- */
  PathNodeJPtr cur_node = path_node_pool_[0];
  cur_node->parent = NULL;
  cur_node->state.head(3) = start_pt;
  cur_node->state.segment(3, 3) = start_v;
  cur_node->state.tail(3) = start_a;
  cur_node->index = pos_checker_ptr_->posToIndex(start_pt);
  cur_node->g_score = 0.0;

  Eigen::VectorXd end_state(9);
  Eigen::Vector3i end_index;
  double time_to_goal;

  end_state.head(3) = end_pt;
  end_state.segment(3, 3) = end_v;
  end_state.tail(3) = end_a;
  end_index = pos_checker_ptr_->posToIndex(end_pt);
  cur_node->f_score =
      lambda_heu_ * estimateHeuristic(cur_node->state, end_state, time_to_goal);
  cur_node->node_state = IN_OPEN_SET;

  open_set_.push(cur_node);
  use_node_num_ += 1;

  // retrievePath(cur_node);
  // bool succ = computeShotTraj(cur_node->state, end_state, time_to_goal);

  // if (succ) {
  //   return REACH_END;
  // } else {
  //   return NO_PATH;
  // }

  if (dynamic) {
    time_origin_ = time_start;
    cur_node->time = time_start;
    cur_node->time_idx = timeToIndex(time_start);
    expanded_nodes_.insert(cur_node->index, cur_node->time_idx, cur_node);
    // cout << "time start: " << time_start << endl;
  } else
    expanded_nodes_.insert(cur_node->index, cur_node);

  PathNodeJPtr neighbor = NULL;
  PathNodeJPtr terminate_node = NULL;
  double init_search = init;
  // const int tolerance = ceil(1 / resolution_);
  const int tolerance = ceil(1 / resolution_);

  vector<Eigen::Vector3d> inputs;
  vector<double> durations;
  /* ---------- search loop ---------- */
  while (!open_set_.empty()) {
    /* ---------- get lowest f_score node ---------- */
    cur_node = open_set_.top();
    // cout << "pos: " << cur_node->state.head(3).transpose() << endl;
    // cout << "time: " << cur_node->time << endl;
    // cout << "dist: " << edt_env_->evaluateCoarseEDT(cur_node->state.head(3),
    // cur_node->time) << endl;

    /* ---------- determine termination ---------- */

    bool near_end = abs(cur_node->index(0) - end_index(0)) <= tolerance &&
                    abs(cur_node->index(1) - end_index(1)) <= tolerance &&
                    abs(cur_node->index(2) - end_index(2)) <= tolerance;
    bool reach_horizon = (cur_node->state.head(3) - end_pt).norm() <= horizon_;

    if (reach_horizon || near_end) {

      if (near_end) {
        cout << "[Kino Astar]: near end." << endl;

        /* one shot trajectory */
        estimateHeuristic(cur_node->state, end_state, time_to_goal);
        computeShotTraj(cur_node->state, end_state, time_to_goal);

        //         if (terminate_node->parent == NULL && !is_shot_succ_)
        //           return NO_PATH;
        if (is_shot_succ_) {
          cout << "[Kino Astar Jerk]: used node num: " << use_node_num_
               << ", iter num: " << iter_num_ << endl;
          terminate_node = cur_node;
          retrievePath(terminate_node);
          has_path_ = true;
          return REACH_END;
        } else
          return NO_PATH;
      } else if (reach_horizon) {
        computeShotTraj(cur_node->state, end_state, time_to_goal);

        //         if (terminate_node->parent == NULL && !is_shot_succ_)
        //           return NO_PATH;
        if (is_shot_succ_) {
          cout << "[Kino Astar Jerk]: Reach horizon_" << endl;
          cout << "[Kino Astar Jerk]: used node num: " << use_node_num_
               << ", iter num: " << iter_num_ << endl;
          terminate_node = cur_node;
          retrievePath(terminate_node);
          has_path_ = true;
          return REACH_END;
        }
      }
    }

    /* ---------- pop node and add to close set ---------- */
    open_set_.pop();
    cur_node->node_state = IN_CLOSE_SET;
    iter_num_ += 1;

    /* ---------- init state propagation ---------- */
    // input_res_ = 1 / 2.0;
    // time_res_ = 1 / 1.0;
    // time_res_init_ = 1 / 8.0;

    Eigen::Matrix<double, 9, 1> cur_state = cur_node->state;
    Eigen::Matrix<double, 9, 1> pro_state;
    vector<PathNodeJPtr> tmp_expand_nodes;
    Eigen::Vector3d um;
    double pro_t;

    inputs.clear();
    durations.clear();
    if (init_search) {
      double sign_x = (end_pt(0) - start_pt(0) > 0 ? 1 : -1);
      double sign_y = (end_pt(1) - start_pt(1) > 0 ? 1 : -1);
      double sign_z = (end_pt(2) - start_pt(2) > 0 ? 1 : -1);

      for (double jx = 0; jx <= max_jerk_ + 1e-3;
           jx += max_jerk_ * input_res_init_)
        for (double jy = 0; jy <= max_jerk_ + 1e-3;
             jy += max_jerk_ * input_res_init_)
          for (double jz = 0; jz <= max_jerk_ + 1e-3;
               jz += max_jerk_ * input_res_init_) {
            um << sign_x * jx, sign_y * jy, 0.5 * sign_z * jz;
            inputs.push_back(um);
          }

      for (double tau = time_res_init_ * init_max_tau_; tau <= init_max_tau_;
           tau += time_res_init_ * init_max_tau_)
        durations.push_back(tau);
    } else {
      for (double jx = -max_jerk_; jx <= max_jerk_ + 1e-3;
           jx += max_jerk_ * input_res_)
        for (double jy = -max_jerk_; jy <= max_jerk_ + 1e-3;
             jy += max_jerk_ * input_res_)
          for (double jz = -max_jerk_; jz <= max_jerk_ + 1e-3;
               jz += max_jerk_ * input_res_) {
            um << jx, jy, 0.5 * jz;
            inputs.push_back(um);
          }

      for (double tau = time_res_ * max_tau_; tau <= max_tau_ + 1e-3;
           tau += time_res_ * max_tau_)
        durations.push_back(tau);
    }

    /* ---------- state propagation loop ---------- */
    // cout << "cur state: " << cur_state.head(3).transpose() << endl;
    // cout << "inputs size: " << inputs.size() << endl;
    // cout << "durations size: " << durations.size() << endl;

    for (int i = 0; i < inputs.size(); ++i) {
      for (int j = 0; j < durations.size(); ++j) {

        um = inputs[i];
        double tau = durations[j];
        stateTransit(cur_state, pro_state, um, tau);
        // cout << pro_state.transpose() << endl;
        pro_t = cur_node->time + tau;

        // geometry_msgs::Point p;
        // p.x = pro_state(0);
        // p.y = pro_state(1);
        // p.z = pro_state(2);

        // visited_points.header.stamp = ros::Time::now();
        // visited_points.action = visualization_msgs::Marker::DELETE;
        // point_pub_.publish(visited_points);

        // visited_points.points.push_back(p);
        // visited_points.action = visualization_msgs::Marker::ADD;
        // visited_points.header.stamp = ros::Time::now();
        // point_pub_.publish(visited_points);

        /* ---------- check if in free space ---------- */
        /* inside map range */
        if (!pos_checker_ptr_->validatePosSurround(
                Eigen::Vector3d(pro_state(0), pro_state(1), pro_state(2)))) {
          // cout << "outside map" << endl;
          continue;
        }

        /* not in close set */
        Eigen::Vector3i pro_id =
            pos_checker_ptr_->posToIndex(pro_state.head(3));
        int pro_t_id = timeToIndex(pro_t);

        PathNodeJPtr pro_node = dynamic ? expanded_nodes_.find(pro_id, pro_t_id)
                                        : expanded_nodes_.find(pro_id);

        if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET) {
          // cout << "in closeset" << endl;
          continue;
        }

        /* vel feasibe */
        Eigen::Vector3d pro_v = pro_state.segment(3, 3);
        if (pro_v.norm() > max_vel_) {
          // cout << "end point vel infeasible" << endl;
          continue;
        }

        /* acc feasible */
        Eigen::Vector3d pro_a = pro_state.tail(3);
        if (pro_a.norm() > max_acc_) {
          // cout << "end point acc infeasible" << endl;
          continue;
        }

        /* not in the same voxel */
        Eigen::Vector3i diff = pro_id - cur_node->index;
        int diff_time = pro_t_id - cur_node->time_idx;
        if (diff.norm() == 0 && ((!dynamic) || diff_time == 0)) {
          // cout << "in same voxel" << endl;
          continue;
        }

        Eigen::Matrix<double, 9, 1> xt;
        bool is_occ = false, vel_violate = false, acc_violate = false;
        // double t;
        // size_t numPoints = ceil(tau / 0.02);
        // size_t step = 1;
        // while (step < numPoints)
        //   step *= 2;
        // double actual_deltaT = tau / numPoints;
        // for (; step > 1; step /= 2) {
        //   for (size_t i = step / 2; i < numPoints; i += step) {
        //     t = actual_deltaT * i;
        //     stateTransit(cur_state, xt, um, t);
        //     Eigen::Vector3d pos(xt.head(3)), vel(xt.segment(3, 3)),
        //         acc(xt.tail(3));
        //     if (vel.norm() > max_vel_) {
        //       vel_violate = true;
        //       break;
        //     }
        //     if (!pos_checker_ptr_->validatePosSurround(pos)) {
        //       is_occ = true;
        //       break;
        //     }
        //   }
        // }

        for (double t = 0; t < tau; t += 0.02) {
          stateTransit(cur_state, xt, um, t);
          Eigen::Vector3d pos(xt.head(3)), vel(xt.segment(3, 3)),
              acc(xt.tail(3));

          if (vel.norm() > max_vel_) {
            vel_violate = true;
            // cout << "vel voilate!" << endl;
            break;
          }

          if (acc.norm() > max_acc_) {
            acc_violate = true;
            // cout << "acc voilate!" << endl;
            break;
          }
          if (!pos_checker_ptr_->validatePosSurround(pos)) {
            is_occ = true;
            // cout << "collision!" << endl;
            break;
          }
        }

        if (is_occ || vel_violate || acc_violate) {
          continue;
        }

        /* ---------- compute cost ---------- */
        double time_to_goal, tmp_g_score, tmp_f_score;
        tmp_g_score = (um.squaredNorm() + w_time_) * tau + cur_node->g_score;
        // std::cout << "current g score: " << tmp_g_score << std::endl;
        // std::cout << "current heuristic: "
        // << estimateHeuristic(pro_state, end_state, time_to_goal) <<
        // std::endl;
        tmp_f_score =
            tmp_g_score +
            lambda_heu_ * estimateHeuristic(pro_state, end_state, time_to_goal);

        /* ---------- compare expanded node in this loop ---------- */

        bool prune = false;
        for (int j = 0; j < tmp_expand_nodes.size(); ++j) {
          PathNodeJPtr expand_node = tmp_expand_nodes[j];
          if ((pro_id - expand_node->index).norm() == 0 &&
              ((!dynamic) || pro_t_id == expand_node->time_idx)) {
            prune = true;
            if (tmp_f_score < expand_node->f_score) {
              expand_node->f_score = tmp_f_score;
              expand_node->g_score = tmp_g_score;
              expand_node->state = pro_state;
              expand_node->input = um;
              expand_node->duration = tau;
              if (dynamic)
                expand_node->time = cur_node->time + tau;
            }
            break;
          }
        }

        /* ---------- new neighbor in this loop ---------- */

        if (!prune) {
          if (pro_node == NULL) {
            pro_node = path_node_pool_[use_node_num_];
            pro_node->index = pro_id;
            pro_node->state = pro_state;
            pro_node->f_score = tmp_f_score;
            pro_node->g_score = tmp_g_score;
            pro_node->input = um;
            pro_node->duration = tau;
            pro_node->parent = cur_node;
            pro_node->node_state = IN_OPEN_SET;
            if (dynamic) {
              pro_node->time = cur_node->time + tau;
              pro_node->time_idx = timeToIndex(pro_node->time);
            }
            open_set_.push(pro_node);

            if (dynamic)
              expanded_nodes_.insert(pro_id, pro_node->time, pro_node);
            else
              expanded_nodes_.insert(pro_id, pro_node);

            tmp_expand_nodes.push_back(pro_node);

            use_node_num_ += 1;
            if (use_node_num_ == allocate_num_) {
              cout << "run out of memory." << endl;
              return NO_PATH;
            }
          } else if (pro_node->node_state == IN_OPEN_SET) {
            if (tmp_g_score < pro_node->g_score) {
              // pro_node->index = pro_id;
              pro_node->state = pro_state;
              pro_node->f_score = tmp_f_score;
              pro_node->g_score = tmp_g_score;
              pro_node->input = um;
              pro_node->duration = tau;
              pro_node->parent = cur_node;
              if (dynamic)
                pro_node->time = cur_node->time + tau;
            }
          } else {
            cout << "error type in searching: " << pro_node->node_state << endl;
          }
        }

        /* ----------  ---------- */
      }
    }
    init_search = false;
  }
  ROS_INFO_STREAM("k a jerk use: " << (ros::Time::now() - t_ka_s).toSec());
  /* ---------- open set empty, no path ---------- */
  cout << "open set empty, no path!" << endl;
  cout << "use node num: " << use_node_num_ << endl;
  cout << "iter num: " << iter_num_ << endl;
  return NO_PATH;
}

void KinodynamicAstarJ::setParam(const ros::NodeHandle &nh) {
  nh.param("search/jerk/max_tau", max_tau_, -1.0);
  nh.param("search/jerk/init_max_tau", init_max_tau_, -1.0);
  nh.param("search/jerk/max_vel", max_vel_, -1.0);
  nh.param("search/jerk/max_acc", max_acc_, -1.0);

  nh.param("search/jerk/w_time", w_time_, -1.0);
  nh.param("search/jerk/horizon", horizon_, -1.0);
  nh.param("search/jerk/resolution_astar", resolution_, -1.0);
  nh.param("search/jerk/time_resolution", time_resolution_, -1.0);
  nh.param("search/jerk/lambda_heu", lambda_heu_, -1.0);
  nh.param("search/jerk/allocate_num", allocate_num_, -1);

  nh.param("search/jerk/max_jerk", max_jerk_, -1.0);
  // nh.param("search/jerk/rho", rho_, -1.0);

  nh.param("search/jerk/input_res", input_res_, -1.0);
  nh.param("search/jerk/input_res_int", input_res_init_, -1.0);
  nh.param("search/jerk/time_res", time_res_, -1.0);
  nh.param("search/jerk/time_res_init", time_res_init_, -1.0);

  cout << "====================================" << endl;
  cout << "input res: " << input_res_ << endl;
  cout << "input res init: " << input_res_init_ << endl;
  cout << "time res: " << time_res_ << endl;
  cout << "time res init: " << time_res_init_ << endl;
  cout << "====================================" << endl;

  // nh.param("search/jerk/rho", rho_, -1.0);
  rho_ = 1.0 / w_time_;
  bvp_.init(TRIPLE_INTEGRATOR);
  cout << "rho_: " << rho_ << endl;
  cout << "horizon: " << horizon_ << endl;
  cout << "max v: " << max_vel_ << endl;
  cout << "max a: " << max_acc_ << endl;
  cout << "max j: " << max_jerk_ << endl;
  bvp_.setRho(rho_);

  nh_ = nh;
  point_pub_ =
      nh_.advertise<visualization_msgs::Marker>("jerk_control_points", 10);
}

void KinodynamicAstarJ::retrievePath(PathNodeJPtr end_node) {
  PathNodeJPtr cur_node = end_node;
  path_nodes_.push_back(cur_node);

  while (cur_node->parent != NULL) {
    cur_node = cur_node->parent;
    path_nodes_.push_back(cur_node);
  }

  reverse(path_nodes_.begin(), path_nodes_.end());
}

void KinodynamicAstarJ::getTraj(Trajectory &traj) {
  traj.clear();
  // traj.emplace_back();

  for (size_t i = 0; i < path_nodes_.size() - 1; i++) {
    PathNodeJPtr temp_node = path_nodes_[i];
    double dur = path_nodes_[i + 1]->duration;
    CoefficientMat coeff;
    coeff.row(0) << 0, 0, 1.0 / 6 * path_nodes_[i + 1]->input(0),
        1.0 / 2 * temp_node->state(6), temp_node->state(3), temp_node->state(0);
    coeff.row(1) << 0, 0, 1.0 / 6 * path_nodes_[i + 1]->input(1),
        1.0 / 2 * temp_node->state(7), temp_node->state(4), temp_node->state(1);
    coeff.row(2) << 0, 0, 1.0 / 6 * path_nodes_[i + 1]->input(2),
        1.0 / 2 * temp_node->state(8), temp_node->state(5), temp_node->state(2);

    Piece temp_piece(dur, coeff);
    traj.emplace_back(temp_piece);
  }

  if (is_shot_succ_) {
    Piece temp_piece(t_shot_, coef_shot_);
    traj.emplace_back(temp_piece);
  }

  return;
}

double KinodynamicAstarJ::estimateHeuristic(Eigen::VectorXd x1,
                                            Eigen::VectorXd x2,
                                            double &optimal_time) {
  if (bvp_.solve(x1, x2, ACC_KNOWN)) {
    double cost = w_time_ * bvp_.getCostStar();
    optimal_time = bvp_.getTauStar();
    // cout << "In esitimate heuristic, the cost: " << cost
    //      << " The distance is: " << (x2.head(3) - x1.head(3)).norm() <<
    //      endl;
    return 1.0 * (1 + tie_breaker_) * cost;
  } else {
    std::cout << "sth. wrong with the bvp solver" << std::endl;
    return 0;
  }
}

bool KinodynamicAstarJ::computeShotTraj(Eigen::VectorXd state1,
                                        Eigen::VectorXd state2,
                                        double time_to_goal) {
  is_shot_succ_ = false;
  // cout << "IN compute shot traj" << endl;
  // cout << state1.transpose() << endl;
  // cout << state2.transpose() << endl;
  /* ---------- get coefficient ---------- */
  CoefficientMat coef;
  if (bvp_.solve(state1, state2, ACC_KNOWN)) {
    bvp_.getCoeff(coef);
  } else {
    std::cout << "sth. wrong with the bvp solver" << std::endl;
    return false;
  }

  /* ---------- forward checking of trajectory ---------- */
  // double t_delta = t_d / 10;
  double t_d = bvp_.getTauStar();
  double t_delta = 0.02;

  Eigen::MatrixXd Tm(6, 6);
  Tm << 0, 5, 0, 0, 0, 0, //
      0, 0, 4, 0, 0, 0,   //
      0, 0, 0, 3, 0, 0,   //
      0, 0, 0, 0, 2, 0,   //
      0, 0, 0, 0, 0, 1,   //
      0, 0, 0, 0, 0, 0;

  Vector3d coord, vel, acc;
  VectorXd poly1d, t, polyv, polya;
  Vector3i index;

  for (double time = t_delta; time <= t_d; time += t_delta) {
    t = VectorXd::Zero(6);
    for (int j = 0; j < 6; j++)
      t(j) = pow(time, 5 - j);

    for (int dim = 0; dim < 3; dim++) {
      poly1d = coef.row(dim);
      coord(dim) = poly1d.dot(t);
    }

    if (!pos_checker_ptr_->validatePosSurround(coord)) {
      // cout << " Shot traj collision! " << endl;
      return false;
    }
  }

  int n(0);
  Piece shot_traj(t_d, coef);
  while (1)
  {
    if (shot_traj.checkMaxAccRate(max_acc_))
    {
      break;
    }
    else if (n < 3)
    {
      n++;
      double k(0.97);
      shot_traj.scaleTime(k);
    }
    else
      return false;
  }
  n = 0;
  while (1)
  {
    if (shot_traj.checkMaxAccRate(max_acc_))
    {
      break;
    }
    else if (n < 3)
    {
      n++;
      double k(0.97);
      shot_traj.scaleTime(k);
    }
    else
      return false;
  }

  coef_shot_ = shot_traj.getCoeffMat();
  t_shot_ = shot_traj.getDuration();
  is_shot_succ_ = true;
  return true;
}

void KinodynamicAstarJ::init() {
  /* ---------- map params ---------- */
  this->inv_resolution_ = 1.0 / resolution_;
  inv_time_resolution_ = 1.0 / time_resolution_;

  /* ---------- pre-allocated node ---------- */
  path_node_pool_.resize(allocate_num_);
  for (int i = 0; i < allocate_num_; i++) {
    path_node_pool_[i] = new PathNodeJ;
  }

  phi_ = Eigen::MatrixXd::Identity(9, 9);

  use_node_num_ = 0;
  iter_num_ = 0;

  visited_points.header.frame_id = "map";
  visited_points.header.stamp = ros::Time::now();
  visited_points.id = 1;
  visited_points.action = visualization_msgs::Marker::ADD;
  visited_points.type = visualization_msgs::Marker::POINTS;
  visited_points.scale.x = 0.08;
  visited_points.scale.y = 0.08;
  visited_points.scale.z = 0.08;
  visited_points.color.a = 1.0;
  visited_points.color.r = 1.0;
  visited_points.color.g = 0;
  visited_points.color.b = 0;

  visited_points.points.clear();
}

void KinodynamicAstarJ::setPosChecker(const PosChecker::Ptr &checker) {
  this->pos_checker_ptr_ = checker;
}

void KinodynamicAstarJ::reset() {
  expanded_nodes_.clear();
  path_nodes_.clear();

  std::priority_queue<PathNodeJPtr, std::vector<PathNodeJPtr>, NodeComparatorJ>
      empty_queue;
  open_set_.swap(empty_queue);

  for (int i = 0; i < use_node_num_; i++) {
    PathNodeJPtr node = path_node_pool_[i];
    node->parent = NULL;
    node->node_state = NOT_EXPAND;
  }

  use_node_num_ = 0;
  iter_num_ = 0;
  is_shot_succ_ = false;
  visited_points.points.clear();
}

std::vector<Eigen::Vector3d> KinodynamicAstarJ::getKinoTraj(double delta_t) {
  vector<Vector3d> state_list;

  /* ---------- get traj of searching ---------- */
  PathNodeJPtr node = path_nodes_.back();
  Matrix<double, 9, 1> x0, xt;

  while (node->parent != NULL) {
    Vector3d ut = node->input;
    double duration = node->duration;
    x0 = node->parent->state;

    for (double t = duration; t >= -1e-5; t -= delta_t) {
      stateTransit(x0, xt, ut, t);
      state_list.push_back(xt.head(3));
    }
    node = node->parent;
  }
  reverse(state_list.begin(), state_list.end());

  /* ---------- get traj of one shot ---------- */
  if (is_shot_succ_) {
    Vector3d coord;
    VectorXd poly1d, time(6);

    for (double t = delta_t; t <= t_shot_; t += delta_t) {
      for (int j = 0; j < 6; j++)
        time(j) = pow(t, 5 - j);

      for (int dim = 0; dim < 3; dim++) {
        poly1d = coef_shot_.row(dim);
        coord(dim) = poly1d.dot(time);
      }
      state_list.push_back(coord);
    }
  }

  return state_list;
}

void KinodynamicAstarJ::getVisTraj(
    double delta_t, vector<Eigen::Matrix<double, 9, 1>> &states) {
  /* ---------- get traj of searching ---------- */
  PathNodeJPtr node = path_nodes_.back();
  Matrix<double, 9, 1> x0, xt;

  while (node->parent != NULL) {
    Vector3d ut = node->input;
    double duration = node->duration;
    x0 = node->parent->state;

    for (double t = duration; t >= -1e-5; t -= delta_t) {
      stateTransit(x0, xt, ut, t);
      states.push_back(xt);
    }
    node = node->parent;
  }
  reverse(states.begin(), states.end());

  /* ---------- get traj of one shot ---------- */
  if (is_shot_succ_) {
    Eigen::Matrix<double, 9, 1> state;
    VectorXd poly1d, time_p(6), time_v(6), time_a(6);
    time_p.setZero();
    time_v.setZero();
    time_a.setZero();

    for (double t = delta_t; t <= t_shot_; t += delta_t) {
      for (int j = 0; j < 6; j++)
        time_p(j) = pow(t, 5 - j);

      for (int j = 0; j < 5; j++)
        time_v(j) = (5 - j) * pow(t, 4 - j);

      for (int j = 0; j < 4; j++)
        time_a(j) = (5 - j) * (4 - j) * pow(t, 3 - j);

      for (int dim = 0; dim < 3; dim++) {
        poly1d = coef_shot_.row(dim);
        state(dim + 0) = poly1d.dot(time_p);
        state(dim + 3) = poly1d.dot(time_v);
        state(dim + 6) = poly1d.dot(time_a);
      }

      states.push_back(state);
    }
  }
}

void KinodynamicAstarJ::getTrajAttributes(double &traj_duration,
                                          double &ctrl_cost,
                                          double &traj_length, int &seg_nums) {
  ctrl_cost = 0.0;
  traj_duration = 0.0;
  traj_length = 0.0;
  seg_nums = 0;

  /* ---------- get traj of searching ---------- */
  PathNodeJPtr node = path_nodes_.back();
  Matrix<double, 9, 1> x0, xt;
  double delta_t = 0.03;

  while (node->parent != NULL) {
    Vector3d ut = node->input;
    double duration = node->duration;
    x0 = node->parent->state;

    seg_nums++;
    traj_duration += duration;
    ctrl_cost += duration * ut.dot(ut);
    for (double t = duration; t >= -1e-5; t -= delta_t) {
      stateTransit(x0, xt, ut, t);
      double speed =
          sqrt(xt(3, 0) * xt(3, 0) + xt(4, 0) * xt(4, 0) + xt(5, 0) * xt(5, 0));
      traj_length += speed * delta_t;
    }
    node = node->parent;
  }

  /* ---------- get traj of one shot ---------- */
  if (is_shot_succ_) {
    seg_nums++;
    Vector3d vel, control;
    VectorXd poly1d, time_v(6), time_a(6), time_j(6);
    time_v.setZero();
    time_a.setZero();

    for (double t = delta_t; t <= t_shot_; t += delta_t) {
      // for (int j = 0; j < 6; j++)
      //   time_p(j) = pow(t, 5 - j);

      for (int j = 0; j < 5; j++)
        time_v(j) = (5 - j) * pow(t, 4 - j);

      // for (int j = 0; j < 4; j++)
      //   time_a(j) = (5 - j) * (4 - j) * pow(t, 3 - j);

      for (int j = 0; j < 3; j++)
        time_j(j) = (5 - j) * (4 - j) * (3 - j) * pow(t, 2 - j);

      for (int dim = 0; dim < 3; dim++) {
        // poly1d = coef_shot_.row(dim);
        // vel(dim) = poly1d.dot(time_v);
        // control(dim) = poly1d.dot(time_a);

        poly1d = coef_shot_.row(dim);
        vel(dim) = poly1d.dot(time_v);
        control(dim) = poly1d.dot(time_j);
      }
      ctrl_cost += delta_t * control.dot(control);
      traj_length += vel.norm() * delta_t;
    }
    traj_duration += t_shot_;
  }
}

std::vector<PathNodeJPtr> KinodynamicAstarJ::getVisitedNodes() {
  vector<PathNodeJPtr> visited;
  visited.assign(path_node_pool_.begin(),
                 path_node_pool_.begin() + use_node_num_ - 1);
  return visited;
}

int KinodynamicAstarJ::timeToIndex(double time) {
  int idx = floor((time - time_origin_) * inv_time_resolution_);
}

void KinodynamicAstarJ::stateTransit(Eigen::Matrix<double, 9, 1> &state0,
                                     Eigen::Matrix<double, 9, 1> &state1,
                                     Eigen::Vector3d um, double tau) {
  for (int i = 0; i < 6; ++i) {
    phi_(i, i + 3) = tau;
  }

  for (int i = 0; i < 3; ++i) {
    phi_(i, i + 6) = 0.5 * pow(tau, 2);
  }

  Eigen::Matrix<double, 9, 1> integral;
  integral.head(3) = 1.0 / 6 * pow(tau, 3) * um;
  integral.segment(3, 3) = 0.5 * pow(tau, 2) * um;
  integral.tail(3) = tau * um;

  state1 = phi_ * state0 + integral;
}

} // namespace dyn_planner