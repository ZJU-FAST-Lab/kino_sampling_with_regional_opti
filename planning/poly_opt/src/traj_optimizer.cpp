#include "poly_opt/traj_optimizer.h"
#include <chrono>

namespace tgk_planner 
{
TrajOptimizer::TrajOptimizer(const ros::NodeHandle &node)
{
  node.param("optimization/vel_limit", vel_limit_, 0.0);
  node.param("optimization/acc_limit", acc_limit_, 0.0);
  node.param("optimization/jerk_limit", jerk_limit_, 0.0);
  node.param("optimization/max_iter_times", max_iter_times_, 0);
  ROS_WARN_STREAM("[opt] param: vel_limit: " << vel_limit_);
  ROS_WARN_STREAM("[opt] param: acc_limit: " << acc_limit_);
  ROS_WARN_STREAM("[opt] param: jerk_limit: " << jerk_limit_);
  ROS_WARN_STREAM("[opt] param: max_iter_times: " << max_iter_times_);
  ite_times_ = 0;
}

bool TrajOptimizer::solve_S_H()
{
  bool result(false);
  
  Trajectory last_valid_traj;
  double per_close = 90.0;
  ite_times_ = 0;
  for (; per_close <= 100; per_close += 1)
  {
    ros::Time t1 = ros::Time::now();
    tryQPCloseForm(per_close);
    ite_times_++;
    
    bool valid = checkTrajectoryConstraints(optimized_traj_);
    // std::vector<int> segs; 
    // std::vector<double> t_s, t_e; 
    // std::vector<Eigen::Vector3d> collisions;
    // bool valid = pos_checker_ptr_->getPolyTrajCollision(optimized_traj_, segs, collisions, t_s, t_e);
    // ros::Time t2 = ros::Time::now();
    // for (int i=0; i<segs.size(); ++i)
      // ROS_WARN_STREAM("collision: " << segs[i] << ", pos: " << collisions[i].transpose());
    // ROS_INFO("valid: %d, optimize for %d time(s), per_close: %lf, per_smooth: %lf", valid, ite_times_, per_close, 100.0-per_close);
    // ROS_WARN_STREAM("optimization iterate once used: " << (t2 - t1).toSec() << " s");
    if (valid) 
    {
      result = reTiming(optimized_traj_);
      if (!result)
        ROS_WARN("[opt-s-h] collision feasible but retiming false");
      return result;
    }
    // else
    // {
    //   vector<StatePVA> vis_x;
    //   optimized_traj_.sampleWholeTrajectory(&vis_x);
    //   vis_ptr_->visualizeStates(vis_x, OptimizedTraj, pos_checker_ptr_->getLocalTime());
    //   getchar();
    // }
    
  }

  ROS_WARN_STREAM("solve QP times: " << ite_times_);
  ROS_WARN("optimization fail");
  return result;
}

bool TrajOptimizer::solve_S_H_O()
{
  double weight_smooth = 3;
  double weight_close = 97;
  double weight_obs = 200;

  bool result(false);
  
  Q_all_ = weight_smooth * Q_smooth_ + weight_close * Q_close_;
  Z_all_ = weight_close * Q_close_ * coeff0_;

  // std::vector<std::vector<Eigen::Vector3d>> drag_lines;
  for (ite_times_ = 1; ite_times_ <= max_iter_times_; ++ite_times_)
  {
    auto start = std::chrono::high_resolution_clock::now();
    tryQP(Q_all_, Z_all_);
    // auto t1 = std::chrono::high_resolution_clock::now();
    std::vector<int> segs;
    std::vector<pair<int, int>> seg_num_obs_size;  //first: # of segment in a traj; second: number of attract_pt in this segment. 
    std::vector<double> t_s, t_e; // each attract_pt's timestamp of start and end in its corresponding segment. Size should be the same as the sum of seg_num_obs_size.second.
    std::vector<Eigen::Vector3d> attract_pts;
    bool valid = pos_checker_ptr_->getPolyTrajAttractPts(front_end_traj_, optimized_traj_, seg_num_obs_size, attract_pts, t_s, t_e);
    // auto t2 = std::chrono::high_resolution_clock::now();
    if (valid) 
    { 
      result = reTiming(optimized_traj_);
      if (!result)
        ROS_WARN("[opt-s-h-o] collision feasible but retiming false");
      return result;
    }
    else
    {
      weight_smooth = 1.0;
      weight_close = (100 - weight_smooth) / 2.0;
      weight_obs = weight_close * 2;

      // vector<StatePVA> vis_x;
      // optimized_traj_.sampleWholeTrajectory(&vis_x);
      // vis_ptr_->visualizeStates(vis_x, OptimizedTraj, pos_checker_ptr_->getLocalTime());
      // vector<StatePVA> traj_wpts;
      // optimized_traj_.getWpts(&traj_wpts);
      // vis_ptr_->visualizeSampledState(traj_wpts, pos_checker_ptr_->getLocalTime());
      // getchar();
      // auto t3 = std::chrono::high_resolution_clock::now();
      calMatrixQobsAndCoeff(seg_num_obs_size, attract_pts, t_s, t_e);
      Q_all_ = weight_smooth * Q_smooth_ + weight_close * Q_close_ + weight_obs * Q_obs_;
      Z_all_ = weight_close * Q_close_ * coeff0_ + weight_obs * Q_obs_ * coeff_obs_;
      // auto t4 = std::chrono::high_resolution_clock::now();
      // std::chrono::duration<double> diff1 = t1 - start;
      // std::cout << "try QP: " << diff1.count() * 1e6 << " us\n";
      // std::chrono::duration<double> diff2 = t2 - t1;
      // std::cout << "check traj: " << diff2.count() * 1e6 << " us\n";
      // std::chrono::duration<double> diff3 = t4 - t3;
      // std::cout << "adjust Q: " << diff3.count() * 1e6 << " us\n";
      // int k = 0;
      // for (int i=0; i<seg_num_obs_size.size(); ++i)
      // {
      //   for (int j=0; j<seg_num_obs_size[i].second; ++j)
      //   {
      //     ROS_WARN_STREAM("collision: " << seg_num_obs_size[i].first << ", pos: " << attract_pts[k].transpose() << ",dura: " << t_s[k] << " to " << t_e[k]);
      //     std::vector<Eigen::Vector3d> drag_line;
      //     drag_line.push_back(attract_pts[k]);
      //     drag_line.push_back(front_end_traj_[seg_num_obs_size[i].first].getPos(t_s[k]));
      //     drag_line.push_back(attract_pts[k]);
      //     drag_line.push_back(front_end_traj_[seg_num_obs_size[i].first].getPos(t_e[k]));
      //     drag_lines.push_back(drag_line);
      //     k++;
      //   }
      // }
      // vis_ptr_->visualizePRM(drag_lines, Color::SteelBlue(), pos_checker_ptr_->getLocalTime());
      // getchar();
    }
  }

  ROS_WARN_STREAM("optimization fail after " << max_iter_times_ << " times iteration");
  // if (!optimized_traj_.checkMaxVelRate(vel_limit_))
  // if (!optimized_traj_.checkMaxAccRate(acc_limit_))
  // if (!optimized_traj_.checkMaxJerkRate(jerk_limit_))

  return result;
}

bool TrajOptimizer::solveRegionalOpt(std::vector<pair<int, int>> &seg_num_obs_size, 
                                     std::vector<Eigen::Vector3d> &attract_pts, 
                                     std::vector<double> &t_s, std::vector<double> &t_e)
{
  bool result(false);

  double weight_smooth = 0.5;
  double weight_close = 50;
  double weight_obs = 300;

  // vector<StatePVA> vis_x;
  // front_end_traj_.sampleWholeTrajectory(&vis_x);
  // vis_ptr_->visualizeStates(vis_x, OptimizedTraj, pos_checker_ptr_->getLocalTime());
  // vector<StatePVA> traj_wpts;
  // front_end_traj_.getWpts(&traj_wpts);
  // vis_ptr_->visualizeSampledState(traj_wpts, pos_checker_ptr_->getLocalTime());
  // cout << "front end traj max v: " << front_end_traj_.getMaxVelRate() << "max a: " << front_end_traj_.getMaxAccRate() << endl;
  // getchar();

  std::vector<std::vector<Eigen::Vector3d>> drag_lines;
  for (ite_times_ = 1; ite_times_ < max_iter_times_; ++ite_times_)
  {
    auto t3 = std::chrono::high_resolution_clock::now();
    calMatrixQobsAndCoeff(seg_num_obs_size, attract_pts, t_s, t_e);
    auto t4 = std::chrono::high_resolution_clock::now();
    Q_all_ = weight_smooth * Q_smooth_ + weight_close * Q_close_ + weight_obs * Q_obs_;
    Z_all_ = weight_close * Q_close_ * coeff0_ + weight_obs * Q_obs_ * coeff_obs_;
    auto start = std::chrono::high_resolution_clock::now();
    tryQP(Q_all_, Z_all_);

    // vector<StatePVA> vis_x;
    // optimized_traj_.sampleWholeTrajectory(&vis_x);
    // vis_ptr_->visualizeStates(vis_x, OptimizedTraj, pos_checker_ptr_->getLocalTime());
    // vector<StatePVA> traj_wpts;
    // optimized_traj_.getWpts(&traj_wpts);
    // vis_ptr_->visualizeSampledState(traj_wpts, pos_checker_ptr_->getLocalTime());
    // int k = 0;
    // for (int i=0; i<seg_num_obs_size.size(); ++i)
    // {
    //   for (int j=0; j<seg_num_obs_size[i].second; ++j)
    //   {
    //     ROS_WARN_STREAM("collision: " << seg_num_obs_size[i].first << ", pos: " << attract_pts[k].transpose() << ",dura: " << t_s[k] << " to " << t_e[k]);
    //     std::vector<Eigen::Vector3d> drag_line;
    //     drag_line.push_back(attract_pts[k]);
    //     drag_line.push_back(front_end_traj_[seg_num_obs_size[i].first].getPos(t_s[k]));
    //     drag_line.push_back(attract_pts[k]);
    //     drag_line.push_back(front_end_traj_[seg_num_obs_size[i].first].getPos(t_e[k]));
    //     drag_lines.push_back(drag_line);
    //     k++;
    //   }
    // }
    // vis_ptr_->visualizePRM(drag_lines, Color::SteelBlue(), pos_checker_ptr_->getLocalTime());
    // getchar();

    auto t1 = std::chrono::high_resolution_clock::now();
    vector<pair<Vector3d, Vector3d>> collide_pts;
    vector<pair<double, double>> collide_timestamp;
    bool valid = pos_checker_ptr_->checkPolyTraj(optimized_traj_, collide_pts, collide_timestamp);
    // bool valid = pos_checker_ptr_->getRegionalAttractPts(front_end_traj_, optimized_traj_, seg_num_obs_size, attract_pts, t_s, t_e); // TODO
    auto t2 = std::chrono::high_resolution_clock::now();
    if (valid) 
    { 
      result = reTiming(optimized_traj_);
      if (!result)
        ROS_WARN("[regional opt] collision feasible but retiming false");
      return result;
    }
    else
    {
      vector<Eigen::Vector3d> grid_paths;
      seg_num_obs_size.clear();
      attract_pts.clear();
      t_s.clear();
      t_e.clear();
      for (int i = 0; i < collide_pts.size(); ++i)
      {
        if (searcher_->AstarSearch(pos_checker_ptr_->getResolution(), collide_pts[i].first, collide_pts[i].second))
        {
          vector<Eigen::Vector3d> grid_path = searcher_->getPath();
          grid_paths.insert(grid_paths.begin(), grid_path.begin(), grid_path.end());
          pos_checker_ptr_->getRegionalAttractPts(optimized_traj_, grid_path, collide_timestamp[i], seg_num_obs_size, attract_pts, t_s, t_e);
        }
      }
      vis_ptr_->visualizeKnots(grid_paths, pos_checker_ptr_->getLocalTime());
      //re-organize seg_num_obs_size
      int seg_num = optimized_traj_.getPieceNum();
      int obs_size[seg_num] = {0};
      for (int i = 0; i < seg_num_obs_size.size(); ++i)
      {
        int seg_id = seg_num_obs_size[i].first;
        obs_size[seg_id] += seg_num_obs_size[i].second;
      }
      seg_num_obs_size.clear();
      for (int i = 0; i < seg_num; ++i)
      {
        if (obs_size[i] != 0)
        {
          seg_num_obs_size.push_back(std::make_pair(i, obs_size[i]));
        }
      }
      
      // vector<StatePVA> vis_x;
      // optimized_traj_.sampleWholeTrajectory(&vis_x);
      // vis_ptr_->visualizeStates(vis_x, OptimizedTraj, pos_checker_ptr_->getLocalTime());
      // vector<StatePVA> traj_wpts;
      // optimized_traj_.getWpts(&traj_wpts);
      // vis_ptr_->visualizeSampledState(traj_wpts, pos_checker_ptr_->getLocalTime());
      // std::chrono::duration<double> diff1 = t1 - start;
      // std::cout << "try QP: " << diff1.count() * 1e6 << " us\n";
      // std::chrono::duration<double> diff2 = t2 - t1;
      // std::cout << "check traj: " << diff2.count() * 1e6 << " us\n";
      // std::chrono::duration<double> diff3 = t4 - t3;
      // std::cout << "adjust Q: " << diff3.count() * 1e6 << " us\n";
      // getchar();
    }
  }

  // ROS_WARN_STREAM("optimization fail after " << max_iter_times_ << " times iteration");
  // if (!optimized_traj_.checkMaxVelRate(vel_limit_))
  // if (!optimized_traj_.checkMaxAccRate(acc_limit_))
  // if (!optimized_traj_.checkMaxJerkRate(jerk_limit_))
  // cout << "result: " << result << "max v: " << optimized_traj_.getMaxVelRate() << "max a: " << optimized_traj_.getMaxAccRate() << endl;
  return result;
}

bool TrajOptimizer::initialize_smooth_close_obs()
{
  ros::Time t1 = ros::Time::now();
  int num_piece = front_end_traj_.getPieceNum();
  // no free variable left for one segment 
  if (num_piece <= 1)
    return false;
  int n_wp = num_piece + 1;
  this->path_ = Eigen::MatrixXd::Zero(n_wp, 3);
  this->vel_way_points_ = Eigen::MatrixXd::Zero(n_wp, 3);
  this->acc_way_points_ = Eigen::MatrixXd::Zero(n_wp, 3);
  this->coeff0_ = Eigen::MatrixXd::Zero(num_piece * 6, 3);
  for(int i = 0; i < num_piece; ++i)
  {
    path_.row(i) = front_end_traj_.getJuncPos(i).transpose();
    vel_way_points_.row(i) = front_end_traj_.getJuncVel(i).transpose();
    acc_way_points_.row(i) = front_end_traj_.getJuncAcc(i).transpose();
    for (int j = 0; j < 6; ++j)
    {
      CoefficientMat mat = front_end_traj_[i].getCoeffMat();
      coeff0_(6 * i + j, 0) = mat(0, 5 - j);
      coeff0_(6 * i + j, 1) = mat(1, 5 - j);
      coeff0_(6 * i + j, 2) = mat(2, 5 - j);
    }
  }
  // ROS_INFO_STREAM("coeff0:\n" << coeff0_);
  path_.row(num_piece) = front_end_traj_.getJuncPos(num_piece).transpose();
  vel_way_points_.row(num_piece) = front_end_traj_.getJuncVel(num_piece).transpose();
  acc_way_points_.row(num_piece) = front_end_traj_.getJuncAcc(num_piece).transpose();

  this->time_ = front_end_traj_.getDurations();
  this->m_ = num_piece;
  this->calMatrixA();
  this->calMatrixQ_smooth(MINIMUM_JERK);
  this->calMatrixQ_close();
  this->calMatrixCandMatrixZ(MIDDLE_P_V_A_CONSISTANT);

  D_ = Eigen::MatrixXd::Zero(3 * m_ + 3, 3); // 3 axes, x, y, z
  D_(0, 0) = path_(0, 0);  D_(1, 0) = vel_way_points_(0, 0);  D_(2, 0) =  acc_way_points_(0, 0); 
  D_(3, 0) = path_(m_, 0); D_(4, 0) = vel_way_points_(m_, 0); D_(5, 0) =  acc_way_points_(m_, 0); 
  D_(0, 1) = path_(0, 1);  D_(1, 1) = vel_way_points_(0, 1);  D_(2, 1) =  acc_way_points_(0, 1);
  D_(3, 1) = path_(m_, 1); D_(4, 1) = vel_way_points_(m_, 1); D_(5, 1) =  acc_way_points_(m_, 1); 
  D_(0, 2) = path_(0, 2);  D_(1, 2) = vel_way_points_(0, 2);  D_(2, 2) =  acc_way_points_(0, 2);
  D_(3, 2) = path_(m_, 2); D_(4, 2) = vel_way_points_(m_, 2); D_(5, 2) =  acc_way_points_(m_, 2); 

  vQo_i_.resize(m_);
  vcoeffo_i_.resize(m_);
  for (int i = 0; i < m_; ++i)
  {
    vQo_i_[i] = MatrixXd::Zero(6, 6);
    vcoeffo_i_[i] = MatrixXd::Zero(6, 3);
  }

  Q_obs_ = Eigen::MatrixXd::Zero(m_ * 6, m_ * 6);
  coeff_obs_ = Eigen::MatrixXd::Zero(m_ * 6, 3);

  // ros::Time t2 = ros::Time::now();
  // ROS_WARN_STREAM("optimization initialization used: " << (t2 - t1).toSec() << " s");
  return true;
}

bool TrajOptimizer::initialize_smooth_close()
{
  int num_piece = front_end_traj_.getPieceNum();
  // no free variable left for one segment 
  if (num_piece <= 1)
    return false;
  int n_wp = num_piece + 1;
  this->path_ = Eigen::MatrixXd::Zero(n_wp, 3);
  this->vel_way_points_ = Eigen::MatrixXd::Zero(n_wp, 3);
  this->acc_way_points_ = Eigen::MatrixXd::Zero(n_wp, 3);
  this->coeff0_ = Eigen::MatrixXd::Zero(num_piece * 6, 3);
  for(int i = 0; i < num_piece; ++i)
  {
    path_.row(i) = front_end_traj_.getJuncPos(i).transpose();
    vel_way_points_.row(i) = front_end_traj_.getJuncVel(i).transpose();
    acc_way_points_.row(i) = front_end_traj_.getJuncAcc(i).transpose();
    for (int j = 0; j < 6; ++j)
    {
      CoefficientMat mat = front_end_traj_[i].getCoeffMat();
      coeff0_(6 * i + j, 0) = mat(0, 5 - j);
      coeff0_(6 * i + j, 1) = mat(1, 5 - j);
      coeff0_(6 * i + j, 2) = mat(2, 5 - j);
    }
  }
  path_.row(num_piece) = front_end_traj_.getJuncPos(num_piece).transpose();
  vel_way_points_.row(num_piece) = front_end_traj_.getJuncVel(num_piece).transpose();
  acc_way_points_.row(num_piece) = front_end_traj_.getJuncAcc(num_piece).transpose();

  this->time_ = front_end_traj_.getDurations();
  this->m_ = num_piece;
  this->calMatrixA();
  this->calMatrixQ_smooth(MINIMUM_JERK);
  this->calMatrixQ_close();
  this->coeff_.resize(m_, 6*3);
  this->calMatrixCandMatrixZ(MIDDLE_P_V_A_CONSISTANT);
  return true;
}

bool TrajOptimizer::initialize_smooth(const Trajectory &traj)
{
  std::vector<Eigen::Vector3d> way_points;
  std::vector<Eigen::Vector3d> vel; 
  std::vector<Eigen::Vector3d> acc;
  std::vector<double> time;
  
  way_points.push_back(traj.getPos(0.0));
  vel.push_back(traj.getVel(0.0));
  acc.push_back(traj.getAcc(0.0));

  const int seg_num = traj.getPieceNum();
  for (int i = 0; i < seg_num; ++i)
  {
    double t = traj[i].getDuration();
    time.push_back(t);
    way_points.push_back(traj[i].getPos(t));
    vel.push_back(traj[i].getVel(t));
    acc.push_back(traj[i].getAcc(t));
  }
  setWayPointsAndTime(way_points, vel, acc, time);
  return true;
}

void TrajOptimizer::setWayPointsAndTime(std::vector<Eigen::Vector3d>& way_points,
                                        std::vector<Eigen::Vector3d>& vel,
                                        std::vector<Eigen::Vector3d>& acc,
                                        std::vector<double>& time)
{
  //TODO add assert to check size equavalence
  this->path_ = Eigen::MatrixXd::Zero(way_points.size(), 3);
  this->vel_way_points_ = Eigen::MatrixXd::Zero(vel.size(), 3);
  this->acc_way_points_ = Eigen::MatrixXd::Zero(acc.size(), 3);
  for(size_t i = 0; i < way_points.size(); ++i)
  {
    path_.row(i) = way_points[i].transpose();
    vel_way_points_.row(i) = vel[i].transpose();
    acc_way_points_.row(i) = acc[i].transpose();
  }
  this->time_ = time;
  this->m_ = time.size();
  this->calMatrixA();
  this->calMatrixQ_smooth(MINIMUM_JERK);
  this->coeff_.resize(m_, 6*3);
  this->calMatrixC();
}

bool TrajOptimizer::solve_S()
{
  for (ite_times_ = 1; ite_times_ < max_iter_times_; ++ite_times_)
  {
    tryQPCloseForm();
    // vector<StatePVA> vis_x;
    // optimized_traj_.sampleWholeTrajectory(&vis_x);
    // vis_ptr_->visualizeStates(vis_x, FMTTraj, pos_checker_ptr_->getLocalTime());
    bool violate(true);
    std::set<int> indices;
    violate = getCollideSegIdx(optimized_traj_, indices);
    if (!violate) 
    { 
      bool result = reTiming(optimized_traj_);
      if (!result)
        ROS_WARN("[opt-smooth] collision feasible but retiming false");
      return result;
    }
    else
    {
      splitDurations(time_, indices);
      vector<Vector3d> vertices, vels, accs;
      if (!getVerticesFromTraj(vertices, vels, accs, front_end_traj_, time_))
      {
        ROS_ERROR("[Opt] Can not get vertices from front traj!");
        return false;
      }
      setWayPointsAndTime(vertices, vels, accs, time_);
    }
  }

  return false;
}

// middle p\v\a consistent, middle p fixed, middle v\a free
void TrajOptimizer::tryQPCloseForm()
{
  Eigen::VectorXd Dx1 = Eigen::VectorXd::Zero(3 * m_ + 3);
  Eigen::VectorXd Dy1 = Eigen::VectorXd::Zero(3 * m_ + 3);
  Eigen::VectorXd Dz1 = Eigen::VectorXd::Zero(3 * m_ + 3);
  Dx1(0) = path_(0, 0);  Dx1(1) = vel_way_points_(0, 0);  Dx1(2) =  acc_way_points_(0, 0); 
  Dx1(3) = path_(m_, 0); Dx1(4) = vel_way_points_(m_, 0); Dx1(5) =  acc_way_points_(m_, 0); 
  Dy1(0) = path_(0, 1);  Dy1(1) = vel_way_points_(0, 1);  Dy1(2) =  acc_way_points_(0, 1);
  Dy1(3) = path_(m_, 1); Dy1(4) = vel_way_points_(m_, 1); Dy1(5) =  acc_way_points_(m_, 1); 
  Dz1(0) = path_(0, 2);  Dz1(1) = vel_way_points_(0, 2);  Dz1(2) =  acc_way_points_(0, 2);
  Dz1(3) = path_(m_, 2); Dz1(4) = vel_way_points_(m_, 2); Dz1(5) =  acc_way_points_(m_, 2); 
  for (int i = 0; i < m_ - 1; ++i)
  {
    Dx1(6 + i) = path_(i + 1, 0);
    Dy1(6 + i) = path_(i + 1, 1);
    Dz1(6 + i) = path_(i + 1, 2);
  }
  Eigen::MatrixXd R = A_inv_multiply_Ct_.transpose() * Q_smooth_ * A_inv_multiply_Ct_;
  Eigen::MatrixXd Rpf(2 * m_ - 2, 5 + m_);
  Eigen::MatrixXd Rpp(2 * m_ - 2, 2 * m_ - 2);
  Rpf = R.block(5 + m_, 0, 2 * m_ - 2, 5 + m_);
  Rpp = R.block(5 + m_, 5 + m_, 2 * m_ - 2, 2 * m_ - 2);
  
  Eigen::MatrixXd Rpp_inv = Rpp.inverse();
  Eigen::MatrixXd neg_Rpp_inv_multiply_Rfp_tran = - Rpp_inv * Rpf;
  Dx1.segment(5 + m_, 2 * m_ - 2) = neg_Rpp_inv_multiply_Rfp_tran * Dx1.segment( 0, 5 + m_ );
  Dy1.segment(5 + m_, 2 * m_ - 2) = neg_Rpp_inv_multiply_Rfp_tran * Dy1.segment( 0, 5 + m_ );
  Dz1.segment(5 + m_, 2 * m_ - 2) = neg_Rpp_inv_multiply_Rfp_tran * Dz1.segment( 0, 5 + m_ );
  Eigen::VectorXd Px = A_inv_multiply_Ct_ * Dx1;
  Eigen::VectorXd Py = A_inv_multiply_Ct_ * Dy1;
  Eigen::VectorXd Pz = A_inv_multiply_Ct_ * Dz1;

  std::vector<CoefficientMat> coeffMats;
  CoefficientMat coeffMat;
  for(int i = 0; i < m_; i ++) 
  {
    for (int j = 0; j < 6; ++j)
    {
      coeffMat(0, j) = Px[i * 6 + 5 - j];
      coeffMat(1, j) = Py[i * 6 + 5 - j];
      coeffMat(2, j) = Pz[i * 6 + 5 - j];
    }
    coeffMats.push_back(coeffMat);
  }
  optimized_traj_ = Trajectory(time_, coeffMats);
}

// middle p\v\a consistent
void TrajOptimizer::tryQPCloseForm(double percent_of_close)
{
  double weight_smooth = 100.0 - percent_of_close;
  double weight_close = percent_of_close;
  
  Eigen::VectorXd Dx1 = Eigen::VectorXd::Zero(3 * m_ + 3);
  Eigen::VectorXd Dy1 = Eigen::VectorXd::Zero(3 * m_ + 3);
  Eigen::VectorXd Dz1 = Eigen::VectorXd::Zero(3 * m_ + 3);
  Dx1(0) = path_(0, 0);  Dx1(1) = vel_way_points_(0, 0);  Dx1(2) =  acc_way_points_(0, 0); 
  Dx1(3) = path_(m_, 0); Dx1(4) = vel_way_points_(m_, 0); Dx1(5) =  acc_way_points_(m_, 0); 
  Dy1(0) = path_(0, 1);  Dy1(1) = vel_way_points_(0, 1);  Dy1(2) =  acc_way_points_(0, 1);
  Dy1(3) = path_(m_, 1); Dy1(4) = vel_way_points_(m_, 1); Dy1(5) =  acc_way_points_(m_, 1); 
  Dz1(0) = path_(0, 2);  Dz1(1) = vel_way_points_(0, 2);  Dz1(2) =  acc_way_points_(0, 2);
  Dz1(3) = path_(m_, 2); Dz1(4) = vel_way_points_(m_, 2); Dz1(5) =  acc_way_points_(m_, 2); 
  int num = 0;
  for (std::set<int>::iterator it = fixed_pos_.begin(); it != fixed_pos_.end(); ++it)
  {
    Dx1(6 + num) = path_((*it) + 1, 0);
    Dy1(6 + num) = path_((*it) + 1, 1);
    Dz1(6 + num) = path_((*it) + 1, 2);
    ++num;
  }
  
//   Eigen::MatrixXd A_inv_multiply_Ct = A_inv_ * Ct_;
  Eigen::MatrixXd R = A_inv_multiply_Ct_.transpose() * (weight_smooth*Q_smooth_ + weight_close*Q_close_) * A_inv_multiply_Ct_;
  Eigen::MatrixXd Rpf(3 * m_ - 3 - n_, 6 + n_);
  Eigen::MatrixXd Rpp(3 * m_ - 3 - n_, 3 * m_ - 3 - n_);
  Rpf = R.block(6 + n_, 0, 3 * m_ - 3 - n_, 6 + n_);
  Rpp = R.block(6 + n_, 6 + n_, 3 * m_ - 3 - n_, 3 * m_ - 3 - n_);
  
  Eigen::MatrixXd Rpp_inv = Rpp.inverse();
  Eigen::MatrixXd neg_Rpp_inv_multiply_Rfp_tran = - Rpp_inv * Rpf;
  Eigen::MatrixXd Rpp_inv_multiply_Zp = weight_close * Rpp_inv * Zp_;
  Dx1.segment(6 + n_, 3 * m_ - 3 - n_) = Rpp_inv_multiply_Zp.col(0) + neg_Rpp_inv_multiply_Rfp_tran * Dx1.segment( 0, 6 + n_ );
  Dy1.segment(6 + n_, 3 * m_ - 3 - n_) = Rpp_inv_multiply_Zp.col(1) + neg_Rpp_inv_multiply_Rfp_tran * Dy1.segment( 0, 6 + n_ );
  Dz1.segment(6 + n_, 3 * m_ - 3 - n_) = Rpp_inv_multiply_Zp.col(2) + neg_Rpp_inv_multiply_Rfp_tran * Dz1.segment( 0, 6 + n_ );
  Eigen::VectorXd Px = A_inv_multiply_Ct_ * Dx1;
  Eigen::VectorXd Py = A_inv_multiply_Ct_ * Dy1;
  Eigen::VectorXd Pz = A_inv_multiply_Ct_ * Dz1;
  
  std::vector<CoefficientMat> coeffMats;
  CoefficientMat coeffMat;
  for(int i = 0; i < m_; i ++) 
  {
    for (int j = 0; j < 6; ++j)
    {
      coeffMat(0, j) = Px[i * 6 + 5 - j];
      coeffMat(1, j) = Py[i * 6 + 5 - j];
      coeffMat(2, j) = Pz[i * 6 + 5 - j];
    }
    coeffMats.push_back(coeffMat);
  }
  optimized_traj_ = Trajectory(time_, coeffMats);
}

void TrajOptimizer::tryQP(const MatrixXd &Q_all, const MatrixXd &Z_all)
{
  // auto start = std::chrono::high_resolution_clock::now();
  Eigen::MatrixXd R = A_inv_multiply_Ct_.transpose() * Q_all * A_inv_multiply_Ct_;
  Eigen::MatrixXd Rpf( R.block(6, 0, 3 * m_ - 3, 6) );
  Eigen::MatrixXd Rpp( R.block(6, 6, 3 * m_ - 3, 3 * m_ - 3) );

  Z_ = A_inv_multiply_Ct_.transpose() * Z_all;
  Zp_ = Z_.block(6, 0, 3 * m_ - 3, 3);

  Eigen::MatrixXd Zp_minus_Rpf_multiply_Df = Zp_ - Rpf * D_.block(0, 0, 6, 3);
  // D_.block(6, 0, 3 * m_ - 3, 3) = Rpp.inverse() * Zp_minus_Rpf_multiply_Df;
  D_.block(6, 0, 3 * m_ - 3, 3) = Rpp.llt().solve(Zp_minus_Rpf_multiply_Df); //Rpp is PD, so use LLT to solve LP
  Eigen::MatrixXd P = A_inv_multiply_Ct_ * D_;

  // auto end1 = std::chrono::high_resolution_clock::now();
  std::vector<CoefficientMat> coeffMats;
  CoefficientMat coeffMat;
  for(int i = 0; i < m_; i ++) 
  {
    for (int j = 0; j < 6; ++j)
    {
      coeffMat(0, j) = P(i * 6 + 5 - j, 0);
      coeffMat(1, j) = P(i * 6 + 5 - j, 1);
      coeffMat(2, j) = P(i * 6 + 5 - j, 2);
    }
    coeffMats.push_back(coeffMat);
  }
  // auto end2 = std::chrono::high_resolution_clock::now();
  optimized_traj_ = Trajectory(time_, coeffMats);

  // auto end3 = std::chrono::high_resolution_clock::now();
  // std::chrono::duration<double> diff1 = end1-start;
  // std::cout << "Matrix multiply: " << diff1.count() * 1e6 << " us\n";
  // std::chrono::duration<double> diff2 = end2-end1;
  // std::cout << "fuzhi coeff: " << diff2.count() * 1e6 << " us\n";
  // std::chrono::duration<double> diff3 = end3-end2;
  // std::cout << "traj coeff: " << diff3.count() * 1e6 << " us\n";
}

void TrajOptimizer::calMatrixCandMatrixZ(int type)
{
  if (type == MIDDLE_P_V_CONSISTANT) /* a inconsistant */
  {
    int num_f = 6;          // 3 + 3 : only start and target has fixed derivatives   
    int num_p = 4 * m_ - 4;  // 4 * (m - 1)
    int num_d = 6 * m_;
  
    Ct_ = MatrixXd::Zero(num_d, num_f + num_p); 
    Ct_( 0, 0 ) = 1; Ct_( 2, 1 ) = 1; Ct_( 4, 2 ) = 1;  // Stack the start point
    Ct_( 1, 6 ) = 1; Ct_( 3, 7 ) = 1; Ct_( 5, 8 ) = 1; 

    Ct_(6 * (m_ - 1) + 0, 4 * (m_ - 2) + 6 + 0 ) = 1; 
    Ct_(6 * (m_ - 1) + 2, 4 * (m_ - 2) + 6 + 1 ) = 1;
    Ct_(6 * (m_ - 1) + 4, 4 * (m_ - 2) + 6 + 3 ) = 1;

    Ct_(6 * (m_ - 1) + 1, 3) = 1; // Stack the end point
    Ct_(6 * (m_ - 1) + 3, 4) = 1;
    Ct_(6 * (m_ - 1) + 5, 5) = 1;

    for(int j = 2; j < m_; j ++ ){
      Ct_( 6 * (j - 1) + 0, 6 + 4 * (j - 2) + 0 ) = 1;
      Ct_( 6 * (j - 1) + 1, 6 + 4 * (j - 1) + 0 ) = 1;
      Ct_( 6 * (j - 1) + 2, 6 + 4 * (j - 2) + 1 ) = 1;
      Ct_( 6 * (j - 1) + 3, 6 + 4 * (j - 1) + 1 ) = 1;
      Ct_( 6 * (j - 1) + 4, 6 + 4 * (j - 2) + 3 ) = 1;
      Ct_( 6 * (j - 1) + 5, 6 + 4 * (j - 1) + 2 ) = 1;
    }
    
    A_inv_multiply_Ct_ = A_inv_ * Ct_;
    Z_ = A_inv_multiply_Ct_.transpose() * Q_close_ * coeff0_;
    Zp_ = Z_.block(6, 0, 4 * m_ - 4, 3);
  }
  else if (type == MIDDLE_P_V_A_CONSISTANT) /* p\v\a consistant */
  {
    int num_f = 6;          // 3 + 3 : only start and target has fixed derivatives   
    int num_p = 3 * m_ - 3;  // 3 * (m - 1)
    int num_d = 6 * m_;
  
    Ct_ = MatrixXd::Zero(num_d, num_f + num_p); 
    Ct_( 0, 0 ) = 1; Ct_( 2, 1 ) = 1; Ct_( 4, 2 ) = 1;  // Stack the start point
    Ct_( 1, 6 ) = 1; Ct_( 3, 7 ) = 1; Ct_( 5, 8 ) = 1; 

    Ct_(6 * (m_ - 1) + 0, 3 * m_ + 0 ) = 1; // Stack the end point
    Ct_(6 * (m_ - 1) + 2, 3 * m_ + 1 ) = 1;
    Ct_(6 * (m_ - 1) + 4, 3 * m_ + 2 ) = 1;
    Ct_(6 * (m_ - 1) + 1, 3) = 1;
    Ct_(6 * (m_ - 1) + 3, 4) = 1;
    Ct_(6 * (m_ - 1) + 5, 5) = 1;

    for(int j = 2; j < m_; j ++ ){
      Ct_( 6 * (j - 1) + 0, 6 + 3 * (j - 2) + 0 ) = 1;
      Ct_( 6 * (j - 1) + 1, 6 + 3 * (j - 1) + 0 ) = 1;
      Ct_( 6 * (j - 1) + 2, 6 + 3 * (j - 2) + 1 ) = 1;
      Ct_( 6 * (j - 1) + 3, 6 + 3 * (j - 1) + 1 ) = 1;
      Ct_( 6 * (j - 1) + 4, 6 + 3 * (j - 2) + 2 ) = 1;
      Ct_( 6 * (j - 1) + 5, 6 + 3 * (j - 1) + 2 ) = 1;
    }

    A_inv_multiply_Ct_ = A_inv_ * Ct_;
    Z_ = A_inv_multiply_Ct_.transpose() * Q_close_ * coeff0_;
    Zp_ = Z_.block(6, 0, 3 * m_ - 3, 3);
  }
}

// middle p fixed, p\v\a consistant
void TrajOptimizer::calMatrixC()
{
  int num_f = 5 + m_;          // 3 + 3 + m_-1: start and target has fixed derivatives, middle p fixed   
  int num_p = 2 * m_ - 2;  // 2 * (m - 1)
  int num_d = 6 * m_;

  Ct_ = MatrixXd::Zero(num_d, num_f + num_p); 
  
  Ct_(0, 0) = 1; Ct_(2, 1) = 1; Ct_(4, 2) = 1;
  Ct_(1, 6) = 1; Ct_(3, 6 + (m_ - 1)) = 1; Ct_(5, 6 + 2 * (m_ - 1)) = 1;
  Ct_(6*(m_-1), 6 + (m_ - 1) - 1) = 1; 
  Ct_(6*(m_-1)+2, 6 + 2 * (m_ - 1) - 1) = 1; 
  Ct_(6*(m_-1)+4, 6 + 3 * (m_ - 1) - 1) = 1; 
  Ct_(6*(m_-1)+1, 3) = 1; Ct_(6*(m_-1)+3, 4) = 1; Ct_(6*(m_-1)+5, 5) = 1; 
  for (int i = 1; i < m_ - 1; ++i)
  {
    Ct_(6 * i, 6 + i - 1) = 1; 
    Ct_(6 * i + 1, 6 + i) = 1; 
    Ct_(6 * i + 2, 6 + (m_ - 1) + i - 1) = 1; 
    Ct_(6 * i + 3, 6 + (m_ - 1) + i) = 1;
    Ct_(6 * i + 4, 6 + 2 * (m_ - 1) + i - 1) = 1;
    Ct_(6 * i + 5, 6 + 2 * (m_ - 1) + i) = 1;
  }  
  A_inv_multiply_Ct_ = A_inv_ * Ct_;
}

/* Produce Mapping Matrix A_ and its inverse A_inv_ */
void TrajOptimizer::calMatrixA()
{
  A_ = Eigen::MatrixXd::Zero(m_ * 6, m_ * 6);
  const static auto Factorial = [](int x)
  {
    int fac = 1;
    for(int i = x; i > 0; i--)
      fac = fac * i;
    return fac;
  };

  Eigen::MatrixXd Ab;
  for(int k = 0; k < m_; k++)
  {
    Ab = Eigen::MatrixXd::Zero(6, 6);
    for(int i = 0; i < 3; i++)
    {
      Ab(2 * i, i) = Factorial(i);
      for(int j = i; j < 6; j++)
        Ab(2 * i + 1, j ) = Factorial(j) / Factorial( j - i ) * pow(time_[k], j - i );
    }
    A_.block(k * 6, k * 6, 6, 6) = Ab;    
  }
  A_inv_ = A_.inverse();
}

/* Produce the smoothness cost Hessian */
void TrajOptimizer::calMatrixQ_smooth(int type)
{
  Q_smooth_ = Eigen::MatrixXd::Zero(m_ * 6, m_ * 6);
  
  if (type == MINIMUM_ACC)
  {
    for(int k = 0; k < m_; k ++)
      for(int i = 2; i < 6; i ++)
        for(int j = i; j < 6; j ++) 
        {
          Q_smooth_( k*6 + i, k*6 + j ) = i * (i - 1) * j * (j - 1) / (i + j - 3) * pow(time_[k], (i + j - 3) );
          Q_smooth_( k*6 + j, k*6 + i ) = Q_smooth_( k*6 + i, k*6 + j );
        }
  }
  else if (type == MINIMUM_JERK)
  {
    for(int k = 0; k < m_; k ++)
      for(int i = 3; i < 6; i ++)
        for(int j = i; j < 6; j ++) 
        {
          Q_smooth_( k*6 + i, k*6 + j ) = i * (i - 1) * (i - 2) * j * (j - 1) * (j - 2) / (i + j - 5) * pow(time_[k], (i + j - 5) );
          Q_smooth_( k*6 + j, k*6 + i ) = Q_smooth_( k*6 + i, k*6 + j );
        }
  }
  else if (type == MINIMUM_SNAP)
  {
    for(int k = 0; k < m_; k ++)
      for(int i = 4; i < 6; i ++)
        for(int j = i; j < 6; j ++) 
        {
          Q_smooth_( k*6 + i, k*6 + j ) = i * (i - 1) * (i - 2) * (i - 3) * j * (j - 1) * (j - 2) * (i - 3) / (i + j - 7) * pow(time_[k], (i + j - 7) );
          Q_smooth_( k*6 + j, k*6 + i ) = Q_smooth_( k*6 + i, k*6 + j );
        }
  }
  else 
  {
    cout << "[OPT]: unfedined Q_smooth type" << endl;
  }
}

/* Produce the closeness cost Hessian matrix */
void TrajOptimizer::calMatrixQ_close()
{
  Q_close_ = Eigen::MatrixXd::Zero(m_ * 6, m_ * 6);
  
  for(int k = 0; k < m_; k ++)
    for(int i = 0; i < 6; i ++)
      for(int j = i; j < 6; j ++) 
      {
        Q_close_( k*6 + i, k*6 + j ) = pow(time_[k], (i + j + 1)) / (i + j + 1);
        Q_close_( k*6 + j, k*6 + i ) = Q_close_( k*6 + i, k*6 + j );
      }
}

void TrajOptimizer::calMatrixQ_obs(const vector<int> &segs, const vector<double> &t_s, const vector<double> &t_e)
{
  Q_obs_ = Eigen::MatrixXd::Zero(m_ * 6, m_ * 6);
  size_t n_obs = segs.size();
  for (size_t k = 0; k < n_obs; ++k)
  {
    int seg_number = segs[k];
    for(int i = 0; i < 6; i ++)
      for(int j = i; j < 6; j ++) 
      {
        Q_obs_( seg_number*6 + i, seg_number*6 + j ) += (pow(t_e[k], (i + j + 1)) - pow(t_s[k], (i + j + 1))) / (i + j + 1);
        Q_obs_( seg_number*6 + j, seg_number*6 + i ) = Q_obs_( seg_number*6 + i, seg_number*6 + j );
      }
  }
  ROS_INFO_STREAM("Q_obs:\n " << Q_obs_);

}

/*
Q_oi: the i^th obs
*/
void TrajOptimizer::calMatrixQAndCoeffPerObs(const Vector3d &attract_pts, const double &t_s, const double &t_e, 
                                            Eigen::Matrix<double, 6, 6> &Q_oi, Eigen::Matrix<double, 6, 3> &coeff_oi)
{
  for(int i = 0; i < 6; i ++)
    for(int j = i; j < 6; j ++) 
    {
      Q_oi(i, j) = (pow(t_e, (i + j + 1)) - pow(t_s, (i + j + 1))) / (i + j + 1);
      Q_oi(j, i) = Q_oi(i, j);
    }
  coeff_oi(0, 0) = attract_pts[0];
  coeff_oi(0, 1) = attract_pts[1];
  coeff_oi(0, 2) = attract_pts[2];
}

void TrajOptimizer::calMatrixQobsAndCoeff(const vector<pair<int, int>> &seg_num_obs_size, const vector<Vector3d> &attract_pts, 
                                  const vector<double> &t_s, const vector<double> &t_e)
{
  int curr_obs = 0;
  for (const pair<int, int> &seon : seg_num_obs_size)
  {
    int seg_num = seon.first;
    int obs_size = seon.second;
    vector<Eigen::Matrix<double, 6, 6>> vQo_i(obs_size); //vector of obs Q of the i^th segment
    vector<Eigen::Matrix<double, 6, 3>> vcoeffo_i(obs_size); 
    // for obs_size obstacles in one seg, incrementally update the corresponding Q and coeff
    for (int i = 0; i < obs_size; ++i)
    {
      vcoeffo_i[i].setZero();
      calMatrixQAndCoeffPerObs(attract_pts[curr_obs], t_s[curr_obs], t_e[curr_obs], vQo_i[i], vcoeffo_i[i]);
      // ROS_INFO_STREAM("vQo_i[i]:\n " << vQo_i[i]);
      // ROS_INFO_STREAM("vcoeffo_i[i]:\n " << vcoeffo_i[i]);
      vQo_i_[seg_num] += vQo_i[i];
      vcoeffo_i_[seg_num] += vQo_i[i] * vcoeffo_i[i];
      curr_obs++;
    }
    //calculate Q_obs & coeff_obs
    Q_obs_.block(seg_num * 6, seg_num * 6, 6, 6) = vQo_i_[seg_num];
    // coeff_obs_.block(seg_num * 6, 0, 6, 3) = vQo_i_[seg_num].inverse() * vcoeffo_i_[seg_num];
    coeff_obs_.block(seg_num * 6, 0, 6, 3) = vQo_i_[seg_num].llt().solve(vcoeffo_i_[seg_num]); //vQo_i_[i] is PD, use LLT to solve LP
  }
  // ROS_INFO_STREAM("Q_obs:\n " << Q_obs_);
  // ROS_INFO_STREAM("coeff_obs_:\n " << coeff_obs_);
}

inline bool TrajOptimizer::checkTrajectoryConstraints(const Trajectory &traj)
{
  if (!traj.checkMaxVelRate(vel_limit_))
  {
    // ROS_WARN("vel constraints violate!");
    return false;
  }
  if (!traj.checkMaxAccRate(acc_limit_))
  {
    // ROS_WARN("acc constraints violate!");
    return false;
  }
  if (!traj.checkMaxJerkRate(jerk_limit_))
  {
    // ROS_WARN("jerk constraints violate!");
    return false;
  }
  if (!pos_checker_ptr_->checkPolyTraj(traj))
  {
    // ROS_WARN("pos constraints violate!");
    return false;
  }
  return true;
}

bool TrajOptimizer::getVerticesFromTraj(vector<Vector3d> &vertices, 
                                        vector<Vector3d> &vels, 
                                        vector<Vector3d> &accs, 
                                        const Trajectory &trajectory, std::vector<double> durs)
{
  size_t n_segment = durs.size();
  if (n_segment <= 0)
  {
    ROS_ERROR("zero segment!");
    return false;
  }

  vertices.reserve(n_segment + 1);
  vels.reserve(n_segment + 1);
  accs.reserve(n_segment + 1);
  double tau(0.0);
  Eigen::Vector3d last_pos, last_vel, last_acc;
  last_pos = trajectory.getPos(tau);
  last_vel = trajectory.getVel(tau);
  last_acc = trajectory.getAcc(tau); 
  vertices.push_back(last_pos);
  vels.push_back(last_vel);
  accs.push_back(last_acc);
  for (int i = 0; i < n_segment; ++i) 
  {
    tau += durs[i];
    last_pos = trajectory.getPos(tau); 
    last_vel = trajectory.getVel(tau);
    last_acc = trajectory.getAcc(tau); 
    vertices.push_back(last_pos);
    vels.push_back(last_vel);
    accs.push_back(last_acc);
  }
  return true;
}

bool TrajOptimizer::getCollideSegIdx(const Trajectory& traj, std::set<int> &indices)
{
  bool result(false);
  double total_dura = traj.getTotalDuration();

  const double kDefaultSamplingTime = 0.02;  // In seconds.
  for (double t = 0.0; t < total_dura; t += kDefaultSamplingTime)
  {
    Vector3d pos = traj.getPos(t);
    if (!pos_checker_ptr_->validatePosSurround(pos)) 
    {
      double t_in_piece = t;
      int idx = traj.locatePieceIdx(t_in_piece);
      indices.insert(idx);
      result = true;
    }
  }
  return result;
}

void TrajOptimizer::splitDurations(std::vector<double>& durs, const std::set<int> &indices)
{
  std::vector<double> durs_ori = durs;
  int step = 0;
  for (const int& idx : indices)
  {
    double dur = durs_ori[idx];
    auto it = durs.begin();
    durs.erase(it + idx + step);
    durs.insert(it + idx + step, 2, dur/2.0);
    step++;
  }
}

bool TrajOptimizer::reTiming(Trajectory& traj)
{
  if (!traj.checkMaxAccRate(acc_limit_))
  {
    const int ite_time(4);
    int n(0);
    while (n < ite_time)
    {
      // ROS_WARN_STREAM("max acc: " << optimized_traj_.getMaxAccRate());
      double k(0.90);
      // double k = min(0.98, acc_limit_ / optimized_traj_.getMaxAccRate());
      traj.scaleTime(k);
      if (traj.checkMaxAccRate(acc_limit_))
        break;
      n++;
    }
    if (n >= ite_time)
    {
      // ROS_ERROR_STREAM("max acc: " << optimized_traj_.getMaxAccRate());
      return false;
    }
  }

  if (!traj.checkMaxVelRate(vel_limit_))
  {
    const int ite_time(3);
    int n(0);
    while (n < ite_time)
    {
      // ROS_WARN_STREAM("max vel: " << optimized_traj_.getMaxVelRate());
      double k(0.95);
      // double k = min(0.98, acc_limit_ / optimized_traj_.getMaxVelRate());
      traj.scaleTime(k);
      if (traj.checkMaxVelRate(vel_limit_))
        break;
      n++;
    }
    if (n >= ite_time)
    {
      // ROS_ERROR_STREAM("max vel: " << optimized_traj_.getMaxVelRate());
      return false;
    }
  }

  if (!traj.checkMaxJerkRate(jerk_limit_))
  {
    const int ite_time(4);
    int n(0);
    while (n < ite_time)
    {
      // ROS_WARN_STREAM("max jerk: " << optimized_traj_.getMaxJerkRate());
      double k(0.90);
      // double k = min(0.98, acc_limit_ / optimized_traj_.getMaxJerkRate());
      traj.scaleTime(k);
      if (traj.checkMaxJerkRate(jerk_limit_))
        break;
      n++;
    }
    if (n >= ite_time)
    {
      // ROS_ERROR_STREAM("max jerk: " << optimized_traj_.getMaxJerkRate());
      return false;
    }
  }

  return true;
}

}
