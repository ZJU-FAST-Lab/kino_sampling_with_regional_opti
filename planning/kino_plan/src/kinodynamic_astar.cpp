#include <kino_plan/kinodynamic_astar.h>
#include <sstream>

using namespace std;
using namespace Eigen;

namespace tgk_planner
{
KinodynamicAstar::~KinodynamicAstar()
{
  for (int i = 0; i < allocate_num_; i++)
  {
    delete path_node_pool_[i];
  }
}

int KinodynamicAstar::search(Eigen::Vector3d start_pt, Eigen::Vector3d start_v, Eigen::Vector3d start_a,
                             Eigen::Vector3d end_pt, Eigen::Vector3d end_v, bool init, bool dynamic, double time_start)
{
  ros::Time t_ka_s = ros::Time::now();
  start_vel_ = start_v;
  start_acc_ = start_a;

  /* ---------- initialize ---------- */
  PathNodePtr cur_node = path_node_pool_[0];
  cur_node->parent = NULL;
  cur_node->state.head(3) = start_pt;
  cur_node->state.tail(3) = start_v;
  cur_node->index = pos_checker_ptr_->posToIndex(start_pt);
  cur_node->g_score = 0.0;

  Eigen::VectorXd end_state(6);
  Eigen::Vector3i end_index;
  double time_to_goal;

  end_state.head(3) = end_pt;
  end_state.tail(3) = end_v;
  end_index = pos_checker_ptr_->posToIndex(end_pt);
  cur_node->f_score = lambda_heu_ * estimateHeuristic(cur_node->state, end_state, time_to_goal);
  cur_node->node_state = IN_OPEN_SET;

  open_set_.push(cur_node);
  use_node_num_ += 1;

  if (dynamic)
  {
    time_origin_ = time_start;
    cur_node->time = time_start;
    cur_node->time_idx = timeToIndex(time_start);
    expanded_nodes_.insert(cur_node->index, cur_node->time_idx, cur_node);
    // cout << "time start: " << time_start << endl;
  }
  else
    expanded_nodes_.insert(cur_node->index, cur_node);

  PathNodePtr neighbor = NULL;
  PathNodePtr terminate_node = NULL;
  bool init_search = init;
  const int tolerance = ceil(1 / resolution_);

  /* ---------- search loop ---------- */
  while (!open_set_.empty())
  {
    /* ---------- get lowest f_score node ---------- */
    cur_node = open_set_.top();
    // cout << "pos: " << cur_node->state.head(3).transpose() << endl;
    // cout << "time: " << cur_node->time << endl;
    // cout << "dist: " << edt_env_->evaluateCoarseEDT(cur_node->state.head(3), cur_node->time) << endl;

    /* ---------- determine termination ---------- */

    bool near_end = abs(cur_node->index(0) - end_index(0)) <= tolerance &&
                    abs(cur_node->index(1) - end_index(1)) <= tolerance &&
                    abs(cur_node->index(2) - end_index(2)) <= tolerance;
    bool reach_horizon = (cur_node->state.head(3) - start_pt).norm() >= horizon_;

    if (reach_horizon || near_end)
    {
      cout << "[Kino Astar]: used node num: " << use_node_num_ << ", iter num: " << iter_num_ << endl;
      terminate_node = cur_node;
      retrievePath(terminate_node);
      has_path_ = true;

      if (near_end)
      {
        cout << "[Kino Astar]: near end." << endl;

        /* one shot trajectory */
        estimateHeuristic(cur_node->state, end_state, time_to_goal);
        computeShotTraj(cur_node->state, end_state, time_to_goal);

//         if (terminate_node->parent == NULL && !is_shot_succ_)
//           return NO_PATH;
        if (is_shot_succ_)
          return REACH_END; 
        else 
          return NO_PATH;
      }
      else if (reach_horizon)
      {
        cout << "[Kino Astar]: Reach horizon_" << endl;
        return REACH_HORIZON;
      }
    }

    /* ---------- pop node and add to close set ---------- */
    open_set_.pop();
    cur_node->node_state = IN_CLOSE_SET;
    iter_num_ += 1;

    /* ---------- init state propagation ---------- */
    double res = 1 / 3.0, time_res = 1 / 1.0, time_res_init = 1 / 8.0;

    Eigen::Matrix<double, 6, 1> cur_state = cur_node->state;
    Eigen::Matrix<double, 6, 1> pro_state;
    vector<PathNodePtr> tmp_expand_nodes;
    Eigen::Vector3d um;
    double pro_t;

    vector<Eigen::Vector3d> inputs;
    vector<double> durations;
    double max_acc = max_acc_ / 1.732;
    double ctrl_step = max_acc * res;

    if (init_search)
    {
      inputs.push_back(start_acc_);
      for (double tau = time_res_init * init_max_tau_; tau <= init_max_tau_; tau += time_res_init * init_max_tau_)
        durations.push_back(tau);
    }
    else
    {
      for (double ax = -max_acc; ax <= max_acc + 1e-3; ax += ctrl_step)
        for (double ay = -max_acc; ay <= max_acc + 1e-3; ay += ctrl_step)
          for (double az = -max_acc; az <= max_acc + 1e-3; az += ctrl_step)
          {
            um << ax, ay, 0.5 * az;
            inputs.push_back(um);
          }
      for (double tau = time_res * max_tau_; tau <= max_tau_; tau += time_res * max_tau_)
        durations.push_back(tau);
    }

    /* ---------- state propagation loop ---------- */
    // cout << "cur state:" << cur_state.head(3).transpose() << endl;
    for (int i = 0; i < inputs.size(); ++i)
      for (int j = 0; j < durations.size(); ++j)
      {
        init_search = false;
        um = inputs[i];
        double tau = durations[j];
        stateTransit(cur_state, pro_state, um, tau);
        pro_t = cur_node->time + tau;

        /* ---------- check if in free space ---------- */

        /* inside map range */
        if (!pos_checker_ptr_->validatePosSurround(Eigen::Vector3d(pro_state(0), pro_state(1), pro_state(2))))
        {
          // cout << "outside map" << endl;
          continue;
        }

        /* not in close set */
        Eigen::Vector3i pro_id = pos_checker_ptr_->posToIndex(pro_state.head(3));
        int pro_t_id = timeToIndex(pro_t);

        PathNodePtr pro_node = dynamic ? expanded_nodes_.find(pro_id, pro_t_id) : expanded_nodes_.find(pro_id);

        if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET)
        {
          // cout << "in closeset" << endl;
          continue;
        }

        /* vel feasibe */
        Eigen::Vector3d pro_v = pro_state.tail(3);
        if (pro_v.norm() > max_vel_)
        {
          // cout << "vel infeasible" << endl;
          continue;
        }

        /* not in the same voxel */
        Eigen::Vector3i diff = pro_id - cur_node->index;
        int diff_time = pro_t_id - cur_node->time_idx;
        if (diff.norm() == 0 && ((!dynamic) || diff_time == 0))
        {
          continue;
        }

        Eigen::Matrix<double, 6, 1> xt;
        bool is_occ = false, vel_violate = false;
        double t;
        size_t numPoints = ceil(tau / 0.02);
        size_t step = 1;
        while (step < numPoints) step *= 2;
        double actual_deltaT = tau/numPoints;
        for ( ; step > 1; step /= 2) {
          for (size_t i = step / 2; i < numPoints; i += step) {
            t = actual_deltaT*i;
            stateTransit(cur_state, xt, um, t);
            Eigen::Vector3d pos(xt.head(3)), vel(xt.tail(3)), acc(0.0, 0.0, 0.0);
            if (vel.norm() > max_vel_)
            {
              vel_violate = true;
              break;
            }
            if (!pos_checker_ptr_->validatePosSurround(pos))
            {
              is_occ = true;
              break;
            }
          }
        }

        if (is_occ || vel_violate)
        {
          // cout << "collision" << endl;
          continue;
        }

        /* ---------- compute cost ---------- */
        double time_to_goal, tmp_g_score, tmp_f_score;
        tmp_g_score = (um.squaredNorm() + w_time_) * tau + cur_node->g_score;
        tmp_f_score = tmp_g_score + lambda_heu_ * estimateHeuristic(pro_state, end_state, time_to_goal);

        /* ---------- compare expanded node in this loop ---------- */

        bool prune = false;
        for (int j = 0; j < tmp_expand_nodes.size(); ++j)
        {
          PathNodePtr expand_node = tmp_expand_nodes[j];
          if ((pro_id - expand_node->index).norm() == 0 && ((!dynamic) || pro_t_id == expand_node->time_idx))
          {
            prune = true;
            if (tmp_f_score < expand_node->f_score)
            {
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

        if (!prune)
        {
          if (pro_node == NULL)
          {
            pro_node = path_node_pool_[use_node_num_];
            pro_node->index = pro_id;
            pro_node->state = pro_state;
            pro_node->f_score = tmp_f_score;
            pro_node->g_score = tmp_g_score;
            pro_node->input = um;
            pro_node->duration = tau;
            pro_node->parent = cur_node;
            pro_node->node_state = IN_OPEN_SET;
            if (dynamic)
            {
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
            if (use_node_num_ == allocate_num_)
            {
              cout << "run out of memory." << endl;
              return NO_PATH;
            }
          }
          else if (pro_node->node_state == IN_OPEN_SET)
          {
            if (tmp_g_score < pro_node->g_score)
            {
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
          }
          else
          {
            cout << "error type in searching: " << pro_node->node_state << endl;
          }
        }

        /* ----------  ---------- */
      }
  }
  ROS_INFO_STREAM("k a use: " << (ros::Time::now() - t_ka_s).toSec());
  /* ---------- open set empty, no path ---------- */
  cout << "open set empty, no path!" << endl;
  cout << "use node num: " << use_node_num_ << endl;
  cout << "iter num: " << iter_num_ << endl;
  return NO_PATH;
}

void KinodynamicAstar::setParam(const ros::NodeHandle& nh)
{
  nh.param("search/max_tau", max_tau_, -1.0);
  nh.param("search/init_max_tau", init_max_tau_, -1.0);
  nh.param("search/max_vel", max_vel_, -1.0);
  nh.param("search/max_acc", max_acc_, -1.0);
  nh.param("search/w_time", w_time_, -1.0);
  nh.param("search/horizon", horizon_, -1.0);
  nh.param("search/resolution_astar", resolution_, -1.0);
  nh.param("search/time_resolution", time_resolution_, -1.0);
  nh.param("search/lambda_heu", lambda_heu_, -1.0);
  nh.param("search/allocate_num", allocate_num_, -1);
}

void KinodynamicAstar::retrievePath(PathNodePtr end_node)
{
  PathNodePtr cur_node = end_node;
  path_nodes_.push_back(cur_node);

  while (cur_node->parent != NULL)
  {
    cur_node = cur_node->parent;
    path_nodes_.push_back(cur_node);
  }

  reverse(path_nodes_.begin(), path_nodes_.end());
}
double KinodynamicAstar::estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2, double& optimal_time)
{
  const Vector3d dp = x2.head(3) - x1.head(3);
  const Vector3d v0 = x1.segment(3, 3);
  const Vector3d v1 = x2.segment(3, 3);

  double c1 = -36 * dp.dot(dp);
  double c2 = 24 * (v0 + v1).dot(dp);
  double c3 = -4 * (v0.dot(v0) + v0.dot(v1) + v1.dot(v1));
  double c4 = 0;
  double c5 = w_time_;

  std::vector<double> ts = quartic(c5, c4, c3, c2, c1);

  double v_max = max_vel_;
  double t_bar = (x1.head(3) - x2.head(3)).lpNorm<Infinity>() / v_max;
  ts.push_back(t_bar);

  double cost = 100000000;
  double t_d = t_bar;

  for (auto t : ts)
  {
    if (t < t_bar)
      continue;
    double c = -c1 / (3 * t * t * t) - c2 / (2 * t * t) - c3 / t + w_time_ * t;
    if (c < cost)
    {
      cost = c;
      t_d = t;
    }
  }

  optimal_time = t_d;

  return 1.0 * (1 + tie_breaker_) * cost;
}

bool KinodynamicAstar::computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2, double time_to_goal)
{
  /* ---------- get coefficient ---------- */
  const Vector3d p0 = state1.head(3);
  const Vector3d dp = state2.head(3) - p0;
  const Vector3d v0 = state1.segment(3, 3);
  const Vector3d v1 = state2.segment(3, 3);
  const Vector3d dv = v1 - v0;
  double t_d = time_to_goal;
  MatrixXd coef(3, 4);
  end_vel_ = v1;

  Vector3d a = 1.0 / 6.0 * (-12.0 / (t_d * t_d * t_d) * (dp - v0 * t_d) + 6 / (t_d * t_d) * dv);
  Vector3d b = 0.5 * (6.0 / (t_d * t_d) * (dp - v0 * t_d) - 2 / t_d * dv);
  Vector3d c = v0;
  Vector3d d = p0;

  // 1/6 * alpha * t^3 + 1/2 * beta * t^2 + v0
  // a*t^3 + b*t^2 + v0*t + p0
  coef.col(3) = a, coef.col(2) = b, coef.col(1) = c, coef.col(0) = d;

  Vector3d coord, vel, acc;
  VectorXd poly1d, t, polyv, polya;
  Vector3i index;

  Eigen::MatrixXd Tm(4, 4);
  Tm << 0, 1, 0, 0, 0, 0, 2, 0, 0, 0, 0, 3, 0, 0, 0, 0;

  /* ---------- forward checking of trajectory ---------- */
  // double t_delta = t_d / 10;
  double t_delta = 0.02;
  for (double time = t_delta; time <= t_d; time += t_delta)
  {
    t = VectorXd::Zero(4);
    for (int j = 0; j < 4; j++)
      t(j) = pow(time, j);

    for (int dim = 0; dim < 3; dim++)
    {
      poly1d = coef.row(dim);
      coord(dim) = poly1d.dot(t);
      vel(dim) = (Tm * poly1d).dot(t);
      acc(dim) = (Tm * Tm * poly1d).dot(t);
    }
    if (vel.norm() > max_vel_ || vel.norm() > max_acc_)
    {
      // cout << "vel:" << vel(dim) << ", acc:" << acc(dim) << endl;
      return false;
    }

    if (!pos_checker_ptr_->validatePosSurround(coord))
    {
      return false;
    }

  }
  coef_shot_ = coef;
  t_shot_ = t_d;
  is_shot_succ_ = true;
  return true;
}

vector<double> KinodynamicAstar::cubic(double a, double b, double c, double d)
{
  vector<double> dts;

  double a2 = b / a;
  double a1 = c / a;
  double a0 = d / a;

  double Q = (3 * a1 - a2 * a2) / 9;
  double R = (9 * a1 * a2 - 27 * a0 - 2 * a2 * a2 * a2) / 54;
  double D = Q * Q * Q + R * R;
  if (D > 0)
  {
    double S = std::cbrt(R + sqrt(D));
    double T = std::cbrt(R - sqrt(D));
    dts.push_back(-a2 / 3 + (S + T));
    return dts;
  }
  else if (D == 0)
  {
    double S = std::cbrt(R);
    dts.push_back(-a2 / 3 + S + S);
    dts.push_back(-a2 / 3 - S);
    return dts;
  }
  else
  {
    double theta = acos(R / sqrt(-Q * Q * Q));
    dts.push_back(2 * sqrt(-Q) * cos(theta / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 2 * M_PI) / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 4 * M_PI) / 3) - a2 / 3);
    return dts;
  }
}

vector<double> KinodynamicAstar::quartic(double a, double b, double c, double d, double e)
{
  vector<double> dts;

  double a3 = b / a;
  double a2 = c / a;
  double a1 = d / a;
  double a0 = e / a;

  vector<double> ys = cubic(1, -a2, a1 * a3 - 4 * a0, 4 * a2 * a0 - a1 * a1 - a3 * a3 * a0);
  double y1 = ys.front();
  double r = a3 * a3 / 4 - a2 + y1;
  if (r < 0)
    return dts;

  double R = sqrt(r);
  double D, E;
  if (R != 0)
  {
    D = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 + 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
    E = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 - 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
  }
  else
  {
    D = sqrt(0.75 * a3 * a3 - 2 * a2 + 2 * sqrt(y1 * y1 - 4 * a0));
    E = sqrt(0.75 * a3 * a3 - 2 * a2 - 2 * sqrt(y1 * y1 - 4 * a0));
  }

  if (!std::isnan(D))
  {
    dts.push_back(-a3 / 4 + R / 2 + D / 2);
    dts.push_back(-a3 / 4 + R / 2 - D / 2);
  }
  if (!std::isnan(E))
  {
    dts.push_back(-a3 / 4 - R / 2 + E / 2);
    dts.push_back(-a3 / 4 - R / 2 - E / 2);
  }

  return dts;
}

void KinodynamicAstar::init()
{
  /* ---------- map params ---------- */
  this->inv_resolution_ = 1.0 / resolution_;
  inv_time_resolution_ = 1.0 / time_resolution_;

  /* ---------- pre-allocated node ---------- */
  path_node_pool_.resize(allocate_num_);
  for (int i = 0; i < allocate_num_; i++)
  {
    path_node_pool_[i] = new PathNode;
  }

  phi_ = Eigen::MatrixXd::Identity(6, 6);
  use_node_num_ = 0;
  iter_num_ = 0;
}

void KinodynamicAstar::setPosChecker(const PosChecker::Ptr &checker)
{
  this->pos_checker_ptr_ = checker;
}

void KinodynamicAstar::reset()
{
  expanded_nodes_.clear();
  path_nodes_.clear();

  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> empty_queue;
  open_set_.swap(empty_queue);

  for (int i = 0; i < use_node_num_; i++)
  {
    PathNodePtr node = path_node_pool_[i];
    node->parent = NULL;
    node->node_state = NOT_EXPAND;
  }

  use_node_num_ = 0;
  iter_num_ = 0;
  is_shot_succ_ = false;
}

std::vector<Eigen::Vector3d> KinodynamicAstar::getKinoTraj(double delta_t)
{
  vector<Vector3d> state_list;

  /* ---------- get traj of searching ---------- */
  PathNodePtr node = path_nodes_.back();
  Matrix<double, 6, 1> x0, xt;

  while (node->parent != NULL)
  {
    Vector3d ut = node->input;
    double duration = node->duration;
    x0 = node->parent->state;

    for (double t = duration; t >= -1e-5; t -= delta_t)
    {
      stateTransit(x0, xt, ut, t);
      state_list.push_back(xt.head(3));
    }
    node = node->parent;
  }
  reverse(state_list.begin(), state_list.end());

  /* ---------- get traj of one shot ---------- */
  if (is_shot_succ_)
  {
    Vector3d coord;
    VectorXd poly1d, time(4);

    for (double t = delta_t; t <= t_shot_; t += delta_t)
    {
      for (int j = 0; j < 4; j++)
        time(j) = pow(t, j);

      for (int dim = 0; dim < 3; dim++)
      {
        poly1d = coef_shot_.row(dim);
        coord(dim) = poly1d.dot(time);
      }
      state_list.push_back(coord);
    }
  }

  return state_list;
}

void KinodynamicAstar::getVisTraj(double delta_t, vector<Eigen::Matrix<double,9,1>>& states)
{
  /* ---------- get traj of searching ---------- */
  PathNodePtr node = path_nodes_.back();
  Matrix<double, 6, 1> x0, xt;
  Matrix<double, 9, 1> pva;

  while (node->parent != NULL)
  {
    Vector3d ut = node->input;
    double duration = node->duration;
    x0 = node->parent->state;

    for (double t = duration; t >= -1e-5; t -= delta_t)
    {
      stateTransit(x0, xt, ut, t);
      pva.head(6) = xt;
      pva.tail(3) = ut;
      states.push_back(pva);
    }
    node = node->parent;
  }
  reverse(states.begin(), states.end());

  /* ---------- get traj of one shot ---------- */
  if (is_shot_succ_)
  {
    Eigen::Matrix<double,6,1> coord;
    Eigen::Vector3d control;
    VectorXd poly1d, time_p(4), time_v(4), time_a(4);
    time_p.setZero();
    time_v.setZero();
    time_a.setZero();

    for (double t = delta_t; t <= t_shot_; t += delta_t)
    {
      for (int j = 0; j < 4; j++)
        time_p(j) = pow(t, j);
      
      for (int j = 1; j < 4; j++)
        time_v(j) = j * pow(t, j - 1);
      
      for (int j = 2; j < 4; j++)
        time_a(j) = (j - 1) * j * pow(t, j - 2); 

      for (int dim = 0; dim < 3; dim++)
      {
        poly1d = coef_shot_.row(dim);
        coord(dim, 0) = poly1d.dot(time_p);
        coord(dim + 3, 0) = poly1d.dot(time_v);
        control(dim) = poly1d.dot(time_a);
      }
      pva.head(6) = coord;
      pva.tail(3) = control;
      states.push_back(pva);
    }
  }

}

void KinodynamicAstar::getTrajAttributes(double& traj_duration, double& ctrl_cost, double& traj_length, int& seg_nums)
{
  ctrl_cost = 0.0;
  traj_duration = 0.0;
  traj_length = 0.0;
  seg_nums = 0;
  
  /* ---------- get traj of searching ---------- */
  PathNodePtr node = path_nodes_.back();
  Matrix<double, 6, 1> x0, xt;
  double delta_t = 0.03;

  while (node->parent != NULL)
  {
    Vector3d ut = node->input;
    double duration = node->duration;
    x0 = node->parent->state;
    
    seg_nums ++;
    traj_duration += duration;
    ctrl_cost += duration * ut.dot(ut);
    for (double t = duration; t >= -1e-5; t -= delta_t)
    {
      stateTransit(x0, xt, ut, t);
      double speed = sqrt(xt(3,0)*xt(3,0) + xt(4,0)*xt(4,0) + xt(5,0)*xt(5,0));
      traj_length += speed * delta_t;
    }
    node = node->parent;
  }

  /* ---------- get traj of one shot ---------- */
  if (is_shot_succ_)
  {
    seg_nums ++;
    Vector3d vel, control;
    VectorXd poly1d, time_v(4), time_a(4);
    time_v.setZero();
    time_a.setZero();

    for (double t = delta_t; t <= t_shot_; t += delta_t)
    {
      for (int j = 1; j < 4; j++)
        time_v(j) = j * pow(t, j - 1);

      for (int j = 2; j < 4; j++)
        time_a(j) = (j - 1) * j * pow(t_shot_, j - 2); 
      
      for (int dim = 0; dim < 3; dim++)
      {
        poly1d = coef_shot_.row(dim);
        vel(dim) = poly1d.dot(time_v);
        control(dim) = poly1d.dot(time_a);
      }
      ctrl_cost += delta_t * control.dot(control);
      traj_length += vel.norm() * delta_t;
    }
    traj_duration += t_shot_;
    cout << "shot duration: " << t_shot_ << endl;
  }
}

Eigen::MatrixXd KinodynamicAstar::getSamples(double& ts, int& K)
{
  /* ---------- final trajectory time ---------- */
  double T_sum = 0.0;
  if (is_shot_succ_)
    T_sum += t_shot_;

  PathNodePtr node = path_nodes_.back();
  while (node->parent != NULL)
  {
    T_sum += node->duration;
    node = node->parent;
  }
  // cout << "final time:" << T_sum << endl;

  /* ---------- init for sampling ---------- */
  K = floor(T_sum / ts);
  ts = T_sum / (K + 1);
  // cout << "K:" << K << ", ts:" << ts << endl;

  bool sample_shot_traj = is_shot_succ_;

  Eigen::VectorXd sx(K + 2), sy(K + 2), sz(K + 2);
  int sample_num = 0;
  node = path_nodes_.back();

  double t;
  if (sample_shot_traj)
    t = t_shot_;
  else
  {
    t = node->duration;
    end_vel_ = node->state.tail(3);
  }

  for (double ti = T_sum; ti > -1e-5; ti -= ts)
  {
    /* ---------- sample shot traj---------- */
    if (sample_shot_traj)
    {
      Vector3d coord;
      VectorXd poly1d, time(4);
      for (int j = 0; j < 4; j++)
        time(j) = pow(t, j);

      for (int dim = 0; dim < 3; dim++)
      {
        poly1d = coef_shot_.row(dim);
        coord(dim) = poly1d.dot(time);
      }

      sx(sample_num) = coord(0), sy(sample_num) = coord(1), sz(sample_num) = coord(2);
      ++sample_num;
      t -= ts;

      /* end of segment */
      if (t < -1e-5)
      {
        sample_shot_traj = false;
        if (node->parent != NULL)
          t += node->duration;
      }
    }
    /* ---------- sample search traj---------- */
    else
    {
      Eigen::Matrix<double, 6, 1> x0 = node->parent->state;
      Eigen::Matrix<double, 6, 1> xt;
      Vector3d ut = node->input;

      stateTransit(x0, xt, ut, t);
      sx(sample_num) = xt(0), sy(sample_num) = xt(1), sz(sample_num) = xt(2);
      ++sample_num;

      t -= ts;
      // cout << "t: " << t << ", t acc: " << T_accumulate << endl;
      if (t < -1e-5 && node->parent->parent != NULL)
      {
        node = node->parent;
        t += node->duration;
      }
    }
  }
  /* ---------- return samples ---------- */
  Eigen::MatrixXd samples(3, K + 5);
  samples.block(0, 0, 1, K + 2) = sx.reverse().transpose();
  samples.block(1, 0, 1, K + 2) = sy.reverse().transpose();
  samples.block(2, 0, 1, K + 2) = sz.reverse().transpose();
  samples.col(K + 2) = start_vel_;
  samples.col(K + 3) = end_vel_;
  samples.col(K + 4) = node->input;

  return samples;
}

std::vector<PathNodePtr> KinodynamicAstar::getVisitedNodes()
{
  vector<PathNodePtr> visited;
  visited.assign(path_node_pool_.begin(), path_node_pool_.begin() + use_node_num_ - 1);
  return visited;
}

int KinodynamicAstar::timeToIndex(double time)
{
  int idx = floor((time - time_origin_) * inv_time_resolution_);
}

void KinodynamicAstar::stateTransit(Eigen::Matrix<double, 6, 1>& state0, Eigen::Matrix<double, 6, 1>& state1,
                                    Eigen::Vector3d um, double tau)
{
  for (int i = 0; i < 3; ++i)
    phi_(i, i + 3) = tau;

  Eigen::Matrix<double, 6, 1> integral;
  integral.head(3) = 0.5 * pow(tau, 2) * um;
  integral.tail(3) = tau * um;

  state1 = phi_ * state0 + integral;
}

}  // namespace dyn_planner