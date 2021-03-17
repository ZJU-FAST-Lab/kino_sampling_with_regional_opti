#include "r3_plan/r3_planner.h"

namespace tgk_planner
{
  size_t R3Planner::sampling(const Eigen::Vector3d &start, const Eigen::Vector3d &goal)
  {
    const double x_len = (goal - start).norm() / 2.0 + sampling_space_inflate_;
    int x_sample_num = floor(x_len / lattice_step_);
    int y_sample_num = floor(sampling_space_inflate_ / lattice_step_);
    int z_sample_num(y_sample_num);

    Eigen::Vector3d translation = 0.5 * (start + goal);
    Eigen::Vector3d xtf, ytf, ztf, downward(0, 0, -1);
    xtf = (goal - translation).normalized();
    ytf = xtf.cross(downward).normalized();
    ztf = xtf.cross(ytf);
    Eigen::Matrix3d R;
    R.col(0) = xtf;
    R.col(1) = ytf;
    R.col(2) = ztf;

    size_t num = x_sample_num * 2 * y_sample_num * 2 * z_sample_num * 2 + 2;
    graph_.vertices.clear();
    graph_.vertices.resize(num);
    size_t vertice_num(0); 

    Vector3d lattice_pt_ortho(-x_len, -sampling_space_inflate_, -sampling_space_inflate_), lattice_pt;
    for (int x = -x_sample_num; x < x_sample_num; ++x)
    {
      lattice_pt_ortho[0] += lattice_step_;
      lattice_pt_ortho[1] = -sampling_space_inflate_;
      for (int y = -y_sample_num; y < y_sample_num; ++y)
      {
        lattice_pt_ortho[1] += lattice_step_;
        lattice_pt_ortho[2] = -sampling_space_inflate_;
        for (int z = -z_sample_num; z < z_sample_num; ++z)
        {
          lattice_pt_ortho[2] += lattice_step_;
          lattice_pt = R * lattice_pt_ortho + translation;
          // cout << "lattice_pt_ortho: " << lattice_pt_ortho.transpose() << ", lattice_pt: " << lattice_pt.transpose() << endl;
          if (pos_checker_ptr_->validatePosSurround(lattice_pt))
          {
            graph_.vertices[vertice_num].pos = lattice_pt;
            graph_.vertices[vertice_num].id = vertice_num;
            graph_.vertices[vertice_num].neighbour_idx_dist.clear();
            vertice_num++;
          }
        }
      }
    }

    // std::uniform_real_distribution<double> dbl_rand = std::uniform_real_distribution<double>(-1.0, 1.0);
    // std::random_device rd;
    // std::mt19937_64 gen = std::mt19937_64(rd());
    // Vector3d rand_pt_otho, rand_pt;
    // for (size_t i = 0; i < num - 2; ++i)
    // {
    //   double x = dbl_rand(gen);
    //   rand_pt_otho[0] = x * x_len;
    //   double y = dbl_rand(gen);
    //   rand_pt_otho[1] = y * sampling_space_inflate_;
    //   double z = dbl_rand(gen);
    //   rand_pt_otho[2] = z * sampling_space_inflate_;
    //   rand_pt = R * rand_pt_otho + translation;
    //   if (pos_checker_ptr_->validatePosSurround(rand_pt))
    //   {
    //     graph_.vertices[vertice_num].pos = rand_pt;
    //     graph_.vertices[vertice_num].id = vertice_num;
    //     graph_.vertices[vertice_num].neighbour_idx_dist.clear();
    //     vertice_num++;
    //   }
    // }

    graph_.vertices[vertice_num].pos = start;
    start_id_ = vertice_num;
    vertice_num++;
    graph_.vertices[vertice_num].pos = goal;
    goal_id_ = vertice_num;
    vertice_num++;
    vertice_number_ = vertice_num;
    graph_.vertices.resize(vertice_number_);
    ROS_INFO_STREAM("vertices num: " << vertice_number_);
    return vertice_num;
  }

  void R3Planner::constructPRMGraph(double search_radius)
  {
    my_kd_tree_t index(3 /*dim*/, graph_, nanoflann::KDTreeSingleIndexAdaptorParams(10 /* max leaf */) );
	  index.buildIndex();

    for (size_t i = 0; i < vertice_number_; ++i)
    {
      Eigen::Vector3d curr_center_pt(graph_.vertices[i].pos), nearest_obs;
      float nearest_obs_squared_dis = pos_checker_ptr_->nearestObs(curr_center_pt, nearest_obs);
      double query_pt[3] = {curr_center_pt[0], curr_center_pt[1], curr_center_pt[2]};
      std::vector<std::pair<size_t, double>> ret_matches;
      ret_matches.reserve(50);
      nanoflann::SearchParams params;
      params.sorted = true;
      const size_t nMatches = index.radiusSearch(&query_pt[0], search_radius, ret_matches, params);
      //check collision free connection
      int curr_nb_num(0);
      for (const std::pair<size_t, double> &match : ret_matches)
      {
        // only connect to 10 furthest nodes
        if (curr_nb_num >= 61)
          break;
        if (match.second < nearest_obs_squared_dis)
          continue;
        // Eigen::Vector3d curr_neighbour_pt(graph_.vertices[match.first].pos);
        // Eigen::Vector3d collide_pos;
        // if (pos_checker_ptr_->checkLine(curr_center_pt, curr_neighbour_pt, collide_pos, false))
        // {
          // curr_nb_num++;
          graph_.vertices[i].neighbour_idx_dist.push_back(match);
        // }
      }
    }
  }

  inline bool R3Planner::searchShortestPath(std::vector<Eigen::Vector3d> &p, double &cost)
  {
    std::priority_queue<R3Node*, std::vector<R3Node*>, R3NodeComparator> queue;
    R3Node* start_node = &graph_.vertices[start_id_];
    R3Node* goal_node = &graph_.vertices[goal_id_];
    start_node->g_value = 0.0;
    start_node->f_value = getEuclideanHeuristic(graph_.vertices[start_id_].pos, graph_.vertices[goal_id_].pos);
    queue.push(start_node);
    R3Node* curr_node;
    bool goal_found(false);
    int iter_times(0);
    while (!queue.empty())
    {
      curr_node = queue.top();
      queue.pop();
      iter_times++;
      if (curr_node == goal_node)
      {
        goal_found = true;
        break;
      }
      for (const auto &nb : curr_node->neighbour_idx_dist)
      {
        double g_hat = curr_node->g_value + sqrt(nb.second);
        size_t nb_id = nb.first;
        if (graph_.vertices[nb_id].g_value > g_hat)
        {
          Eigen::Vector3d curr_neighbour_pt(graph_.vertices[nb_id].pos);
          Eigen::Vector3d collide_pos;


          Vector3d dir = curr_neighbour_pt - curr_node->pos;
          double len = dir.norm();
          Vector3d dir_norm = dir / len;
          double step = pos_checker_ptr_->getResolution() * 1.5;
          Vector3d step_vec = dir_norm * step;
          int check_n = floor(len / step);
          Vector3d check_pt(curr_node->pos);
          bool valid(true);
          for (int i = 1; i <= check_n; ++i)
          {
            check_pt += step_vec;
            if (!pos_checker_ptr_->validatePosSurround(check_pt))
            {
              valid = false; 
              break;
            }
          }
          if (!valid)
            continue;
          // if (!pos_checker_ptr_->checkLine(curr_node->pos, curr_neighbour_pt, collide_pos, true))
          //   continue;

          double h_value = getEuclideanHeuristic(graph_.vertices[nb_id].pos, graph_.vertices[goal_id_].pos);
          R3Node* nb_node = &graph_.vertices[nb_id];
          nb_node->g_value = g_hat;
          nb_node->f_value = nb_node->g_value + h_value;
          nb_node->parent = curr_node;
          queue.push(nb_node);
        }
      }
    }
    ROS_INFO_STREAM("search iter_times: " << iter_times);
    if (goal_found)
    {
      R3Node* curr_node = goal_node;
      cost = goal_node->g_value;
      while (curr_node)
      {
        p.push_back(curr_node->pos);
        curr_node = curr_node->parent;
      }
      std::reverse(std::begin(p), std::end(p));
      return true;
    }
    else
    {
      return false;
    }
  }

  inline double R3Planner::getEuclideanHeuristic(const Eigen::Vector3d &s, const Eigen::Vector3d &g) const 
  {
    return (g - s).norm();
  }

  void R3Planner::postProcessPath(std::vector<Eigen::Vector3d> &path)
  {
    size_t v_num = path.size();
    if (v_num < 2)
      return;
    std::vector<Eigen::Vector3d> path_origin = path;
    path.clear();
    for (size_t i = 0; i < v_num - 1; ++i)
    {
      double seg_dis = (path_origin[i] - path_origin[i + 1]).norm();
      Eigen::Vector3d nearest_obs;
      float nearest_obs_dis = sqrt(pos_checker_ptr_->nearestObs(path_origin[i], nearest_obs));

    }
  }

  bool R3Planner::planPRM(const Eigen::Vector3d &s, const Eigen::Vector3d &g, std::vector<Eigen::Vector3d> &p, double &cost, double radius)
  {
    ros::Time t1 = ros::Time::now();
    size_t v_n = sampling(s, g);
    ros::Time t2 = ros::Time::now();
    ROS_WARN_STREAM("sampling time: " << (t2 - t1).toSec());
    constructPRMGraph(radius);
    ros::Time t3 = ros::Time::now();
    ROS_WARN_STREAM("construct graph time: " << (t3 - t2).toSec());
    bool re = searchShortestPath(p, cost);
    ros::Time t4 = ros::Time::now();
    ROS_WARN_STREAM("search time: " << (t4 - t3).toSec());
    // if (re)
    // {
    //   postProcessPath(p);
    // }
    
    return re;
  }

  inline std::vector<Eigen::Vector3d> R3Planner::simplifyRoute(const std::vector<Eigen::Vector3d> &route, double resolution) const
  {
    std::vector<Eigen::Vector3d> subRoute;
    if (route.size() == 1 || route.size() == 2)
    {
      subRoute = route;
    }
    else if (route.size() >= 3)
    {
      std::vector<Eigen::Vector3d>::const_iterator maxIt;
      double maxDist = -INFINITY, tempDist;
      Eigen::Vector3d vec((route.back() - route.front()).normalized());

      for (auto it = route.begin() + 1; it != (route.end() - 1); it++)
      {
        tempDist = (*it - route.front() - vec.dot(*it - route.front()) * vec).norm();
        if (maxDist < tempDist)
        {
          maxDist = tempDist;
          maxIt = it;
        }
      }

      if (maxDist > resolution)
      {
        std::vector<Eigen::Vector3d>::const_iterator maxItNext = maxIt;
        maxItNext++;
        subRoute.insert(subRoute.end(), route.begin(), maxItNext);
        subRoute = simplifyRoute(subRoute, resolution);
        std::vector<Eigen::Vector3d> tempRoute(maxIt, route.end());
        tempRoute = simplifyRoute(tempRoute, resolution);
        subRoute.insert(subRoute.end(), tempRoute.begin() + 1, tempRoute.end());
      }
      else
      {
        subRoute.push_back(route.front());
        subRoute.push_back(route.back());
      }
    }

    return subRoute;
  }

  // bool R3Planner::plan(const Eigen::Vector3d &s, const Eigen::Vector3d &g, std::vector<Eigen::Vector3d> &p, double &cost, double timePerUnit) const
  // {
  //   // create and set bounds
  //   const double deltaX = g(0) - s(0), deltaY = g(1) - s(1);
  //   const double boxL = sqrt(deltaX * deltaX + deltaY * deltaY);
  //   const double theta = atan2(deltaY, deltaX);
  //   const double st = sin(theta), ct = cos(theta);
  //   ompl::base::RealVectorBounds bounds(3);
  //   bounds.setLow(0, -sampling_space_inflate_);
  //   bounds.setHigh(0, sampling_space_inflate_ + boxL);
  //   bounds.setLow(1, -sampling_space_inflate_);
  //   bounds.setHigh(1, sampling_space_inflate_);
  //   bounds.setLow(2, ground_height_);
  //   bounds.setHigh(2, ceiling_height_);
  //   // create and set state space
  //   auto space(std::make_shared<ompl::base::RealVectorStateSpace>(3));
  //   space->setBounds(bounds);
  //   // create and set space information
  //   auto si(std::make_shared<ompl::base::SpaceInformation>(space));
  //   si->setStateValidityChecker(
  //       [&](const ompl::base::State *state) {
  //         const auto *pos = state->as<ompl::base::RealVectorStateSpace::StateType>();
  //         Eigen::Vector3d position(s(0) + (*pos)[0] * ct - (*pos)[1] * st, s(1) + (*pos)[0] * st + (*pos)[1] * ct, (*pos)[2]);
  //         return pos_checker_ptr_->validatePosSurround(position);
  //       });
  //   si->setup();
  //   // create and set start & goal state
  //   ompl::base::ScopedState<> start(space), goal(space);
  //   start[0] = 0.0;
  //   start[1] = 0.0;
  //   start[2] = s(2);
  //   goal[0] = boxL;
  //   goal[1] = 0.0;
  //   goal[2] = g(2);
  //   // create and set problem definition
  //   auto pdef(std::make_shared<ompl::base::ProblemDefinition>(si));
  //   pdef->setStartAndGoalStates(start, goal);
  //   pdef->setOptimizationObjective(std::make_shared<ompl::base::PathLengthOptimizationObjective>(si));
  //   auto planner(std::make_shared<ompl::geometric::InformedRRTstar>(si));
  //   // auto planner(std::make_shared<ompl::geometric::LazyPRMstar>(si));
  //   planner->setProblemDefinition(pdef);
  //   planner->setup();
  //   // solve
  //   ompl::base::PlannerStatus solved;
  //   solved = planner->ompl::base::Planner::solve(timePerUnit * boxL);
  //   // solved = planner->ompl::base::Planner::solve(0.005);

  //   cost = -1.0;
  //   if (solved)
  //   {
  //     const ompl::geometric::PathGeometric path = ompl::geometric::PathGeometric(
  //         dynamic_cast<const ompl::geometric::PathGeometric &>(*pdef->getSolutionPath()));
  //     size_t path_state_num = path.getStateCount(); // include start & goal
  //     if (path_state_num < 2)
  //     {
  //       return false;
  //     }
  //     p.clear();
  //     p.reserve(path_state_num);
  //     for (size_t i = 0; i < path_state_num; i++)
  //     {
  //       const auto state = path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  //       p.emplace_back(s(0) + state[0] * ct - state[1] * st, s(1) + state[0] * st + state[1] * ct, state[2]);
  //     }
  //     cost = pdef->getSolutionPath()->cost(pdef->getOptimizationObjective()).value();
  //     return true;
  //   }

  //   return false;
  // }

} // namespace tgk_planner