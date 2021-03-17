#include "visualization_utils/visualization_utils.h"
#include <ros/ros.h>
#include <queue>
#include <string>
#include <iostream>

VisualRviz::VisualRviz(const ros::NodeHandle &nh) : nh_(nh)
{
    rand_sample_pos_point_pub_ = nh_.advertise<visualization_msgs::Marker>("rand_sample_pos_points", 1);
    rand_sample_vel_vec_pub_ = nh_.advertise<visualization_msgs::Marker>("rand_sample_vel_vecs", 1);
    rand_sample_acc_vec_pub_ = nh_.advertise<visualization_msgs::Marker>("rand_sample_acc_vecs", 1);
    tree_traj_pos_point_pub_ = nh_.advertise<visualization_msgs::Marker>("tree_traj_pos", 1);
    tree_traj_vel_vec_pub_ = nh_.advertise<visualization_msgs::Marker>("tree_traj_vel", 1);
    tree_traj_acc_vec_pub_ = nh_.advertise<visualization_msgs::Marker>("tree_traj_acc", 1);
    final_traj_pos_point_pub_ = nh_.advertise<visualization_msgs::Marker>("a_traj_pos", 1);
    final_traj_vel_vec_pub_ = nh_.advertise<visualization_msgs::Marker>("a_traj_vel", 1);
    final_traj_acc_vec_pub_ = nh_.advertise<visualization_msgs::Marker>("a_traj_acc", 1);
    first_traj_pos_point_pub_ = nh_.advertise<visualization_msgs::Marker>("first_traj_pos", 1);
    first_traj_vel_vec_pub_ = nh_.advertise<visualization_msgs::Marker>("first_traj_vel", 1);
    first_traj_acc_vec_pub_ = nh_.advertise<visualization_msgs::Marker>("first_traj_acc", 1);
    best_traj_pos_point_pub_ = nh_.advertise<visualization_msgs::Marker>("best_traj_pos", 1);
    best_traj_vel_vec_pub_ = nh_.advertise<visualization_msgs::Marker>("best_traj_vel", 1);
    best_traj_acc_vec_pub_ = nh_.advertise<visualization_msgs::Marker>("best_traj_acc", 1);
    tracked_traj_pos_point_pub_ = nh_.advertise<visualization_msgs::Marker>("tracked_traj_pos", 1);
    optimized_traj_pos_point_pub_ = nh_.advertise<visualization_msgs::Marker>("optimized_traj_pos", 1);
    optimized_traj_vel_vec_pub_ = nh_.advertise<visualization_msgs::Marker>("optimized_traj_vel", 1);
    optimized_traj_acc_vec_pub_ = nh_.advertise<visualization_msgs::Marker>("optimized_traj_acc", 1);
    fmt_traj_pos_point_pub_ = nh_.advertise<visualization_msgs::Marker>("fmt_traj_pos", 1);
    fmt_traj_vel_vec_pub_ = nh_.advertise<visualization_msgs::Marker>("fmt_traj_vel", 1);
    fmt_traj_acc_vec_pub_ = nh_.advertise<visualization_msgs::Marker>("fmt_traj_acc", 1);
    fmt_wo_traj_pos_point_pub_ = nh_.advertise<visualization_msgs::Marker>("fmt_wo_traj_pos", 1);
    fmt_wo_traj_vel_vec_pub_ = nh_.advertise<visualization_msgs::Marker>("fmt_wo_traj_vel", 1);
    fmt_wo_traj_acc_vec_pub_ = nh_.advertise<visualization_msgs::Marker>("fmt_wo_traj_acc", 1);
    start_and_goal_pub_ = nh_.advertise<visualization_msgs::Marker>("start_and_goal", 1);
    topo_pub_ = nh_.advertise<visualization_msgs::Marker>("topo", 1);
    orphans_pos_pub_ = nh_.advertise<visualization_msgs::Marker>("orphan_pos", 1);
    orphans_vel_vec_pub_ = nh_.advertise<visualization_msgs::Marker>("orphan_vel", 1);
    fwd_reachable_pos_pub_ = nh_.advertise<visualization_msgs::Marker>("fwd_reachable_pos", 1);
    bwd_reachable_pos_pub_ = nh_.advertise<visualization_msgs::Marker>("bwd_reachable_pos", 1);
    knots_pub_ = nh_.advertise<visualization_msgs::Marker>("knots", 1);
    collision_pub_ = nh_.advertise<visualization_msgs::Marker>("collision", 1);
    replan_direction_pub_ = nh_.advertise<visualization_msgs::Marker>("replan_direction", 1);
    traj_list_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/traj_list", 1);

    prm_pub_ = nh_.advertise<visualization_msgs::Marker>("prm", 1);
    multi_traj_pos_point_pub_ = nh_.advertise<visualization_msgs::Marker>("multi_traj", 1);
    prm_vertex_pub_ = nh_.advertise<visualization_msgs::Marker>("prm_vertex", 1);
    topo_pt_pub_ = nh_.advertise<visualization_msgs::Marker>("/topo_pt", 1);
    poly_pt_pub_ = nh_.advertise<visualization_msgs::Marker>("/poly_pt", 1);
    lines_pub_ = nh_.advertise<visualization_msgs::Marker>("/line", 1);
    balls_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/nearest_obs_balls", 1);
    texts_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/texts", 1);
    points_pub_ = nh_.advertise<visualization_msgs::Marker>("/points", 1);
}

void VisualRviz::visualizeStates(const std::vector<StatePVA> &x, int trajectory_type, ros::Time local_time)
{
    visualization_msgs::Marker pos_point, vel_vec, acc_vec;
    if (x.empty())
    {
        pos_point.action = visualization_msgs::Marker::DELETEALL;
        vel_vec.action = visualization_msgs::Marker::DELETEALL;
        acc_vec.action = visualization_msgs::Marker::DELETEALL;
    }
    geometry_msgs::Point p, a;
    for (size_t i = 0; i < x.size(); ++i)
    {
        p.x = x[i](0, 0);
        p.y = x[i](1, 0);
        p.z = x[i](2, 0);
        a.x = x[i](0, 0);
        a.y = x[i](1, 0);
        a.z = x[i](2, 0);
        pos_point.points.push_back(p);

        vel_vec.points.push_back(p);
        p.x += x[i](3, 0);
        p.y += x[i](4, 0);
        p.z += x[i](5, 0);
        vel_vec.points.push_back(p);

        acc_vec.points.push_back(a);
        a.x += x[i](6, 0);
        a.y += x[i](7, 0);
        a.z += x[i](8, 0);
        acc_vec.points.push_back(a);
    }
    pos_point.header.frame_id = "map";
    pos_point.header.stamp = local_time;
    pos_point.ns = "traj";
    pos_point.action = visualization_msgs::Marker::ADD;
    pos_point.lifetime = ros::Duration(0);
    pos_point.pose.orientation.w = 1.0;
    pos_point.id = 100;
    pos_point.type = visualization_msgs::Marker::LINE_STRIP;
    pos_point.scale.x = 0.17;
    pos_point.scale.y = 0.17;
    pos_point.scale.z = 0.17;

    vel_vec.header.frame_id = "map";
    vel_vec.header.stamp = local_time;
    vel_vec.ns = "traj";
    vel_vec.action = visualization_msgs::Marker::ADD;
    vel_vec.lifetime = ros::Duration(0);
    vel_vec.pose.orientation.w = 1.0;
    vel_vec.id = 200;
    vel_vec.type = visualization_msgs::Marker::LINE_LIST;
    vel_vec.scale.x = 0.03;

    acc_vec.header.frame_id = "map";
    acc_vec.header.stamp = local_time;
    acc_vec.ns = "traj";
    acc_vec.action = visualization_msgs::Marker::ADD;
    acc_vec.lifetime = ros::Duration(0);
    acc_vec.pose.orientation.w = 1.0;
    acc_vec.id = 300;
    acc_vec.type = visualization_msgs::Marker::LINE_LIST;
    acc_vec.scale.x = 0.03;
    
    switch (trajectory_type) 
    {
    case FirstTraj:
        pos_point.color = Color::Red();
        pos_point.color.a = 1.0;
        vel_vec.color = Color::Red();
        vel_vec.color.a = 1.0;
        acc_vec.color = Color::Red();
        acc_vec.color.a = 0.05;
        first_traj_pos_point_pub_.publish(pos_point);
        first_traj_vel_vec_pub_.publish(vel_vec);
        first_traj_acc_vec_pub_.publish(acc_vec);
        break;

    case BestTraj:
        pos_point.color = Color::Blue();
        pos_point.color.a = 1.0;
        vel_vec.color = Color::Blue();
        vel_vec.color.a = 1.0;
        acc_vec.color = Color::Blue();
        acc_vec.color.a = 0.15;
        best_traj_pos_point_pub_.publish(pos_point);
        best_traj_vel_vec_pub_.publish(vel_vec);
        best_traj_acc_vec_pub_.publish(acc_vec);
        break;

    case FinalTraj:
        pos_point.color = Color::Blue();
        pos_point.color.a = 1.0;
        vel_vec.color = Color::Blue();
        vel_vec.color.a = 1.0;
        acc_vec.color = Color::Blue();
        acc_vec.color.a = 0.10;
        final_traj_pos_point_pub_.publish(pos_point);
        final_traj_vel_vec_pub_.publish(vel_vec);
        final_traj_acc_vec_pub_.publish(acc_vec);
        break;

    case TrackedTraj:
        pos_point.color = Color::Yellow();
        pos_point.color.a = 1.0;
        vel_vec.color = Color::Yellow();
        vel_vec.color.a = 1.0;
        acc_vec.color = Color::Yellow();
        acc_vec.color.a = 0.05;
        tracked_traj_pos_point_pub_.publish(pos_point);
        break;

    case OptimizedTraj:
        pos_point.color = Color::Red();
        pos_point.color.a = 1.0;
        vel_vec.color = Color::Red();
        vel_vec.color.a = 1.0;
        acc_vec.color = Color::Red();
        acc_vec.color.a = 0.15;
        optimized_traj_pos_point_pub_.publish(pos_point);
        optimized_traj_vel_vec_pub_.publish(vel_vec);
        optimized_traj_acc_vec_pub_.publish(acc_vec);
        break;

    case FMTTraj:
        pos_point.color = Color::Chartreuse();
        pos_point.color.a = 1.0;
        vel_vec.color = Color::Chartreuse();
        vel_vec.color.a = 1.0;
        acc_vec.color = Color::Chartreuse();
        acc_vec.color.a = 0.05;
        fmt_traj_pos_point_pub_.publish(pos_point);
        fmt_traj_vel_vec_pub_.publish(vel_vec);
        fmt_traj_acc_vec_pub_.publish(acc_vec);
        break;

    case FMTTrajWithout:
        pos_point.color = Color::Orange();
        pos_point.color.a = 1.0;
        vel_vec.color = Color::Orange();
        vel_vec.color.a = 1.0;
        acc_vec.color = Color::Orange();
        acc_vec.color.a = 0.12;
        fmt_wo_traj_pos_point_pub_.publish(pos_point);
        fmt_wo_traj_vel_vec_pub_.publish(vel_vec);
        fmt_wo_traj_acc_vec_pub_.publish(acc_vec);
        break;

    case TreeTraj:
        pos_point.color = Color::SteelBlue();
        pos_point.color.a = 1.0;
        pos_point.scale.x = 0.03;
        pos_point.scale.y = 0.03;
        pos_point.scale.z = 0.03;
        pos_point.type = visualization_msgs::Marker::POINTS;
        vel_vec.color = Color::SteelBlue();
        vel_vec.color.a = 1.0;
        acc_vec.color = Color::SteelBlue();
        acc_vec.color.a = 0.1;
        tree_traj_pos_point_pub_.publish(pos_point);
        tree_traj_vel_vec_pub_.publish(vel_vec);
        tree_traj_acc_vec_pub_.publish(acc_vec);
        break;
    }
}

void VisualRviz::visualizePoints(const std::vector<StatePVA> &x, ros::Time local_time)
{
    if (x.empty()) return;
    visualization_msgs::Marker pos_point;
    geometry_msgs::Point p;
    for (size_t i = 0; i < x.size(); ++i)
    {
        p.x = x[i](0, 0);
        p.y = x[i](1, 0);
        p.z = x[i](2, 0);
        pos_point.points.push_back(p);
    }
    pos_point.header.frame_id = "map";
    pos_point.header.stamp = local_time;
    pos_point.ns = "traj";
    pos_point.action = visualization_msgs::Marker::ADD;
    pos_point.lifetime = ros::Duration(0);
    pos_point.pose.orientation.w = 1.0;
    pos_point.id = 100;
    pos_point.type = visualization_msgs::Marker::POINTS;
    pos_point.scale.x = 0.05;
    pos_point.scale.y = 0.05;
    pos_point.scale.z = 0.05;
    pos_point.color = Color::Black();
    pos_point.color.a = 0.8;
    points_pub_.publish(pos_point);
}

void VisualRviz::visualizeKnots(const std::vector<Eigen::Vector3d> &knots, ros::Time local_time)
{
    if (knots.empty()) return;
    visualization_msgs::Marker pos_point;
    geometry_msgs::Point p;
    for (size_t i = 0; i < knots.size(); ++i)
    {
        p.x = knots[i](0);
        p.y = knots[i](1);
        p.z = knots[i](2);
        pos_point.points.push_back(p);
    }
    pos_point.header.frame_id = "map";
    pos_point.header.stamp = local_time;
    pos_point.ns = "traj";
    pos_point.action = visualization_msgs::Marker::ADD;
    pos_point.lifetime = ros::Duration(0);
    pos_point.pose.orientation.w = 1.0;
    pos_point.id = 400;
    pos_point.type = visualization_msgs::Marker::SPHERE_LIST;
    pos_point.scale.x = 0.2;
    pos_point.scale.y = 0.2;
    pos_point.scale.z = 0.2;
    pos_point.color = Color::Pink();
    pos_point.color.a = 1.0;
    knots_pub_.publish(pos_point);
}

void VisualRviz::visualizeCollision(const Eigen::Vector3d &collision, ros::Time local_time)
{
    visualization_msgs::Marker pos_point;
    geometry_msgs::Point p;
    p.x = collision(0);
    p.y = collision(1);
    p.z = collision(2);
    pos_point.points.push_back(p);
    pos_point.header.frame_id = "map";
    pos_point.header.stamp = local_time;
    pos_point.ns = "colllision";
    pos_point.action = visualization_msgs::Marker::ADD;
    pos_point.lifetime = ros::Duration(0);
    pos_point.pose.orientation.w = 1.0;
    pos_point.id = 400;
    pos_point.type = visualization_msgs::Marker::SPHERE_LIST;
    pos_point.scale.x = 0.2;
    pos_point.scale.y = 0.2;
    pos_point.scale.z = 0.2;
    pos_point.color = Color::Red();
    pos_point.color.a = 1.0;
    collision_pub_.publish(pos_point);
}

void VisualRviz::visualizeSampledState(const std::vector<StatePVA> &nodes, ros::Time local_time)
{
    visualization_msgs::Marker pos_point, vel_vec, acc_vec;

    geometry_msgs::Point p;
    for (const auto &node : nodes)
    {
        p.x = node[0];
        p.y = node[1];
        p.z = node[2];
        pos_point.points.push_back(p);
        vel_vec.points.push_back(p);
        p.x += node[3] / 2.0;
        p.y += node[4] / 2.0;
        p.z += node[5] / 2.0;
        vel_vec.points.push_back(p);
    }
    
    pos_point.header.frame_id = "map";
    pos_point.header.stamp = local_time;
    pos_point.ns = "sample";
    pos_point.action = visualization_msgs::Marker::ADD;
    pos_point.lifetime = ros::Duration(0);
    pos_point.pose.orientation.w = 1.0;
    pos_point.id = 10;
    pos_point.type = visualization_msgs::Marker::POINTS;
    pos_point.scale.x = 0.07;
    pos_point.scale.y = 0.07;
    pos_point.scale.z = 0.07;
    pos_point.color = Color::Green();
    pos_point.color.a = 1.0;

    vel_vec.header.frame_id = "map";
    vel_vec.header.stamp = local_time;
    vel_vec.ns = "sample";
    vel_vec.action = visualization_msgs::Marker::ADD;
    vel_vec.lifetime = ros::Duration(0);
    vel_vec.pose.orientation.w = 1.0;
    vel_vec.id = 20;
    vel_vec.type = visualization_msgs::Marker::LINE_LIST;
    vel_vec.scale.x = 0.03;
    vel_vec.color = Color::Red();
    vel_vec.color.a = 1.0;

    rand_sample_pos_point_pub_.publish(pos_point);
    rand_sample_vel_vec_pub_.publish(vel_vec);
}

void VisualRviz::visualizeValidSampledState(const std::vector<StatePVA> &nodes, ros::Time local_time)
{
    visualization_msgs::Marker pos_point, vel_vec, acc_vec;

    geometry_msgs::Point p;
    for (const auto &node : nodes)
    {
        p.x = node[0];
        p.y = node[1];
        p.z = node[2];
        pos_point.points.push_back(p);
    }
    
    pos_point.header.frame_id = "map";
    pos_point.header.stamp = local_time;
    pos_point.ns = "sample";
    pos_point.action = visualization_msgs::Marker::ADD;
    pos_point.lifetime = ros::Duration(0);
    pos_point.pose.orientation.w = 1.0;
    pos_point.id = 10;
    pos_point.type = visualization_msgs::Marker::POINTS;
    pos_point.scale.x = 0.07;
    pos_point.scale.y = 0.07;
    pos_point.scale.z = 0.07;
    pos_point.color = Color::Red();
    pos_point.color.a = 1.0;

    rand_sample_vel_vec_pub_.publish(pos_point);
}

void VisualRviz::visualizeStartAndGoal(Eigen::Vector3d start, Eigen::Vector3d goal, ros::Time local_time)
{
    visualization_msgs::Marker pos_point;

    geometry_msgs::Point p;
    p.x = start[0];
    p.y = start[1];
    p.z = start[2];
    pos_point.points.push_back(p);
    p.x = goal[0];
    p.y = goal[1];
    p.z = goal[2];
    pos_point.points.push_back(p);

    pos_point.header.frame_id = "map";
    pos_point.header.stamp = local_time;
    pos_point.ns = "s_g";
    pos_point.action = visualization_msgs::Marker::ADD;
    pos_point.lifetime = ros::Duration(0);
    pos_point.pose.orientation.w = 1.0;
    pos_point.id = 11;
    pos_point.type = visualization_msgs::Marker::POINTS;
    pos_point.scale.x = 0.4;
    pos_point.scale.y = 0.4;
    pos_point.scale.z = 0.4;
    pos_point.color = Color::Purple();
    pos_point.color.a = 1.0;

    start_and_goal_pub_.publish(pos_point);
}

void VisualRviz::visualizeTopo(const std::vector<Eigen::Vector3d> &p_head,
                               const std::vector<Eigen::Vector3d> &tracks,
                               ros::Time local_time)
{
    if (tracks.empty() || p_head.empty() || tracks.size() != p_head.size())
        return;

    visualization_msgs::Marker topo;
    geometry_msgs::Point p;

    for (int i = 0; i < tracks.size(); i++)
    {
        p.x = p_head[i][0];
        p.y = p_head[i][1];
        p.z = p_head[i][2];
        topo.points.push_back(p);
        p.x += tracks[i][0];
        p.y += tracks[i][1];
        p.z += tracks[i][2];
        topo.points.push_back(p);
    }

    topo.header.frame_id = "map";
    topo.header.stamp = local_time;
    topo.ns = "topo";
    topo.action = visualization_msgs::Marker::ADD;
    topo.lifetime = ros::Duration(0);
    topo.pose.orientation.w = 1.0;
    topo.id = 117;
    topo.type = visualization_msgs::Marker::LINE_LIST;
    topo.scale.x = 0.15;
    topo.color = Color::Green();
    topo.color.a = 1.0;

    topo_pub_.publish(topo);
}

void VisualRviz::visualizeReplanDire(const Eigen::Vector3d &pos, const Eigen::Vector3d &dire, ros::Time local_time)
{
    visualization_msgs::Marker pos_point;

    geometry_msgs::Point p;
    p.x = pos[0];
    p.y = pos[1];
    p.z = pos[2];
    pos_point.points.push_back(p);
    p.x += dire[0];
    p.y += dire[1];
    p.z += dire[2];
    pos_point.points.push_back(p);

    pos_point.header.frame_id = "map";
    pos_point.header.stamp = local_time;
    pos_point.ns = "traj";
    pos_point.action = visualization_msgs::Marker::ADD;
    pos_point.lifetime = ros::Duration(0);
    pos_point.pose.orientation.w = 1.0;
    pos_point.id = 11;
    pos_point.type = visualization_msgs::Marker::ARROW;
    pos_point.scale.x = 0.1;
    pos_point.scale.y = 0.1;
    pos_point.scale.z = 0.1;
    pos_point.color = Color::Purple();
    pos_point.color.a = 1.0;

    replan_direction_pub_.publish(pos_point);
}

void VisualRviz::visualizeReachPos(int type, const Eigen::Vector3d& center, const double& diam, ros::Time local_time)
{
    visualization_msgs::Marker pos;
    pos.header.frame_id = "map";
    pos.header.stamp = local_time;
    pos.ns = "reachable_pos";
    pos.type = visualization_msgs::Marker::SPHERE;
    pos.action = visualization_msgs::Marker::ADD;
    pos.pose.position.x = center[0];
    pos.pose.position.y = center[1];
    pos.pose.position.z = center[2];
    pos.scale.x = diam;
    pos.scale.y = diam;
    pos.scale.z = diam;
    if (type == FORWARD_REACHABLE_POS)
    {
        pos.id = 1;
        pos.color.a = 0.3; 
        pos.color = Color::Green();
        fwd_reachable_pos_pub_.publish(pos);
    }
    else if(type == BACKWARD_REACHABLE_POS)
    {
        pos.id = 2;
        pos.color.a = 0.3; 
        pos.color = Color::Red();
        bwd_reachable_pos_pub_.publish(pos);
    }
    
}

void VisualRviz::visualizeBalls(const std::vector<Eigen::Vector3d> &centers, const std::vector<double> &radii, ros::Time local_time)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    std::string spherePath = "package://visualization_utils/sphere.stl";
    marker.mesh_resource = spherePath;
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = 0;
    marker.color = Color::Yellow();
    marker.color.a = 0.3; 

    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.reserve(centers.size() + 1);
    marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(marker);
    marker.action = visualization_msgs::Marker::ADD;
    for (size_t i = 0; i < centers.size(); ++i) 
    {
        marker.pose.position.x = centers[i][0];
        marker.pose.position.y = centers[i][1];
        marker.pose.position.z = centers[i][2];
        double diam = 2.0 * radii[i];
        marker.scale.x = diam;
        marker.scale.y = diam;
        marker.scale.z = diam;
        marker_array.markers.push_back(marker);
        marker.id++;
    }
    balls_pub_.publish(marker_array);
}


void VisualRviz::visualizeOrphans(const std::vector<StatePVA> &ophs, ros::Time local_time)
{
    visualization_msgs::Marker pos_point, vel_vec;
    geometry_msgs::Point p;

    for (int i = 0; i < ophs.size(); i++)
    {
        p.x = ophs[i](0, 0);
        p.y = ophs[i](1, 0);
        p.z = ophs[i](2, 0);
        pos_point.points.push_back(p);

        vel_vec.points.push_back(p);
        p.x += ophs[i](3, 0) / 10.0;
        p.y += ophs[i](4, 0) / 10.0;
        p.z += ophs[i](5, 0) / 10.0;
        vel_vec.points.push_back(p);
    }

    pos_point.header.frame_id = "map";
    pos_point.header.stamp = local_time;
    pos_point.ns = "orphan";
    pos_point.action = visualization_msgs::Marker::ADD;
    pos_point.lifetime = ros::Duration(0);
    pos_point.pose.orientation.w = 1.0;
    pos_point.id = 43;
    pos_point.type = visualization_msgs::Marker::POINTS;
    pos_point.scale.x = 0.1;
    pos_point.scale.y = 0.1;
    pos_point.scale.z = 0.1;
    pos_point.color = Color::Green();
    pos_point.color.a = 1.0;

    vel_vec.header.frame_id = "map";
    vel_vec.header.stamp = local_time;
    vel_vec.ns = "orphan";
    vel_vec.action = visualization_msgs::Marker::ADD;
    vel_vec.lifetime = ros::Duration(0);
    vel_vec.pose.orientation.w = 1.0;
    vel_vec.id = 244;
    vel_vec.type = visualization_msgs::Marker::LINE_LIST;
    vel_vec.scale.x = 0.1;
    vel_vec.scale.y = 0.1;
    vel_vec.scale.z = 0.1;
    vel_vec.color = Color::Yellow();
    vel_vec.color.a = 1.0;

    orphans_pos_pub_.publish(pos_point);
    orphans_vel_vec_pub_.publish(vel_vec);
}

void VisualRviz::visualizePRM(const std::vector<std::vector<Eigen::Vector3d>> &paths, Color color, ros::Time local_time)
{
    if (paths.empty())
        return;

    visualization_msgs::Marker topo, vertice;
    geometry_msgs::Point p;

    for (const auto &path : paths)
    {
        for (int j = 0; j < path.size() - 1; ++j)
        {
            p.x = path[j][0];
            p.y = path[j][1];
            p.z = path[j][2];
            topo.points.push_back(p);
            vertice.points.push_back(p);
            p.x = path[j + 1][0];
            p.y = path[j + 1][1];
            p.z = path[j + 1][2];
            topo.points.push_back(p);
            vertice.points.push_back(p);
        }
    }

    topo.header.frame_id = "map";
    topo.header.stamp = local_time;
    topo.ns = "prm";
    topo.action = visualization_msgs::Marker::ADD;
    topo.lifetime = ros::Duration(0);
    topo.pose.orientation.w = 1.0;
    topo.id = 118;
    topo.type = visualization_msgs::Marker::LINE_LIST;
    topo.scale.x = 0.05;
    topo.scale.y = 0.05;
    topo.scale.z = 0.05;
    topo.color = color;
    topo.color.a = 1.0;

    vertice.header.frame_id = "map";
    vertice.header.stamp = local_time;
    vertice.ns = "prm";
    vertice.action = visualization_msgs::Marker::ADD;
    vertice.lifetime = ros::Duration(0);
    vertice.pose.orientation.w = 1.0;
    vertice.id = 119;
    vertice.type = visualization_msgs::Marker::POINTS;
    vertice.scale.x = 0.08;
    vertice.scale.y = 0.08;
    vertice.scale.z = 0.08;
    vertice.color = Color::Blue();
    vertice.color.a = 1.0;

    prm_pub_.publish(topo);
    prm_vertex_pub_.publish(vertice);
}

void VisualRviz::visualizeLearning(const std::vector<Eigen::Vector3d> &topo_samples, const std::vector<Eigen::Vector3d> &poly_samples, ros::Time local_time)
{
  visualization_msgs::Marker topo_pts, poly_pts, lines;
  geometry_msgs::Point p;
  
  int n = topo_samples.size();
  for (int i = 0; i < n; i++)
  {
    p.x = topo_samples[i](0);
    p.y = topo_samples[i](1);
    p.z = topo_samples[i](2);
    topo_pts.points.push_back(p);
    lines.points.push_back(p);

    p.x = poly_samples[i](0);
    p.y = poly_samples[i](1);
    p.z = poly_samples[i](2);
    poly_pts.points.push_back(p);
    lines.points.push_back(p);
  }

  topo_pts.header.frame_id = "map";
  topo_pts.header.stamp = ros::Time::now();
  topo_pts.ns = "orphan";
  topo_pts.action = visualization_msgs::Marker::ADD;
  topo_pts.lifetime = ros::Duration(0);
  topo_pts.pose.orientation.w = 1.0;
  topo_pts.id = 43;
  topo_pts.type = visualization_msgs::Marker::POINTS;
  topo_pts.scale.x = 0.1;
  topo_pts.scale.y = 0.1;
  topo_pts.scale.z = 0.1;
  topo_pts.color.r = 1.0;
  topo_pts.color.a = 1.0;

  poly_pts.header.frame_id = "map";
  poly_pts.header.stamp = ros::Time::now();
  poly_pts.ns = "orphan";
  poly_pts.action = visualization_msgs::Marker::ADD;
  poly_pts.lifetime = ros::Duration(0);
  poly_pts.pose.orientation.w = 1.0;
  poly_pts.id = 4;
  poly_pts.type = visualization_msgs::Marker::POINTS;
  poly_pts.scale.x = 0.1;
  poly_pts.scale.y = 0.1;
  poly_pts.scale.z = 0.1;
  poly_pts.color.g = 1.0;
  poly_pts.color.a = 1.0;

  lines.header.frame_id = "map";
  lines.header.stamp = ros::Time::now();
  lines.ns = "orphan";
  lines.action = visualization_msgs::Marker::ADD;
  lines.lifetime = ros::Duration(0);
  lines.pose.orientation.w = 1.0;
  lines.id = 244;
  lines.type = visualization_msgs::Marker::LINE_LIST;
  lines.scale.x = 0.03;
  lines.scale.y = 0.03;
  lines.scale.z = 0.03;
  lines.color.b = 1.0;
  lines.color.a = 1.0;

  topo_pt_pub_.publish(topo_pts);
  poly_pt_pub_.publish(poly_pts);
  lines_pub_.publish(lines);
}

void VisualRviz::visualizeMultiTraj(const std::vector<std::vector<StatePVA>> &x, ros::Time local_time)
{
    if (x.empty()) return;
    visualization_msgs::Marker pos_point;
    geometry_msgs::Point p;
    for (const auto &traj : x)
    {
        for (size_t i = 0; i < traj.size()-1; ++i)
        {
            p.x = traj[i](0, 0);
            p.y = traj[i](1, 0);
            p.z = traj[i](2, 0);
            pos_point.points.push_back(p);

            p.x = traj[i+1](0, 0);
            p.y = traj[i+1](1, 0);
            p.z = traj[i+1](2, 0);
            pos_point.points.push_back(p);

        }
    }
    pos_point.header.frame_id = "map";
    pos_point.header.stamp = local_time;
    pos_point.ns = "traj";
    pos_point.action = visualization_msgs::Marker::ADD;
    pos_point.lifetime = ros::Duration(0);
    pos_point.pose.orientation.w = 1.0;
    pos_point.id = 156;
    pos_point.type = visualization_msgs::Marker::LINE_LIST;
    pos_point.scale.x = 0.01;
    pos_point.scale.y = 0.01;
    pos_point.scale.z = 0.01;
    pos_point.color = Color::SteelBlue();
    pos_point.color.a = 1.0;
    multi_traj_pos_point_pub_.publish(pos_point);
}

void VisualRviz::visualizeTrajList(const std::vector<std::vector<StatePVA>> &x, ros::Time local_time)
{
    if (x.empty()) return;
    visualization_msgs::MarkerArray traj_list_point;
    visualization_msgs::Marker clear_previous_msg;
    clear_previous_msg.action = visualization_msgs::Marker::DELETEALL;
    traj_list_point.markers.push_back(clear_previous_msg);
    
    geometry_msgs::Point p;
    size_t traj_num = x.size();
    for (size_t i = 0; i < traj_num; ++i)
    {
        visualization_msgs::Marker traj_point;
        for (size_t j = 0; j < x[i].size(); ++j)
        {
            p.x = x[i][j](0, 0);
            p.y = x[i][j](1, 0);
            p.z = x[i][j](2, 0);
            traj_point.points.push_back(p);
        }
        traj_point.header.frame_id = "map";
        traj_point.header.stamp = local_time;
        traj_point.ns = "traj_list";
        traj_point.action = visualization_msgs::Marker::ADD;
        traj_point.lifetime = ros::Duration(0);
        traj_point.pose.orientation.w = 1.0;
        traj_point.id = i;
        traj_point.type = visualization_msgs::Marker::LINE_STRIP;
        traj_point.scale.x = 0.25;
        traj_point.scale.y = 0.25;
        traj_point.scale.z = 0.25;
        traj_point.color.r = 0;
        traj_point.color.g = 0.5;
        traj_point.color.b = 1 - (double)(i+1 + 3) / (traj_num + 10);
        traj_point.color.a = (double)(i+1 + 3) / (traj_num + 3);

        traj_list_point.markers.push_back(traj_point);
    }
    
    traj_list_pub_.publish(traj_list_point);
}

void VisualRviz::visualizeText(const std::vector<std::string> &texts, const std::vector<Eigen::Vector3d> &positions, ros::Time local_time)
{
    // visualization_msgs::MarkerArray mk_ary;
    // visualization_msgs::Marker text_mk;
    // text_mk.action = visualization_msgs::Marker::DELETEALL;
    // mk_ary.markers.push_back(text_mk);
    // text_mk.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    // text_mk.header.frame_id = "map";
    // text_mk.header.stamp = local_time;
    // text_mk.ns = "texts";
    // text_mk.action = visualization_msgs::Marker::ADD;
    // text_mk.lifetime = ros::Duration(0);
    // text_mk.color.a = 1.0;
    // text_mk.scale.z = 4;
    // text_mk.color = Color::Blue();
    // for (size_t i = 0; i < texts.size(); ++i)
    // {
    //     text_mk.text = texts[i];
    //     text_mk.pose.position.x = positions[i][0];
    //     text_mk.pose.position.y = positions[i][1];
    //     text_mk.pose.position.z = positions[i][2];
    //     text_mk.id = i;
    //     mk_ary.markers.push_back(text_mk);
    // }
    // texts_pub_.publish(mk_ary);

    visualization_msgs::MarkerArray mk_ary;
    visualization_msgs::Marker text_mk;
    text_mk.action = visualization_msgs::Marker::DELETEALL;
    mk_ary.markers.push_back(text_mk);
    text_mk.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_mk.header.frame_id = "map";
    text_mk.header.stamp = local_time;
    text_mk.ns = "texts";
    text_mk.action = visualization_msgs::Marker::ADD;
    text_mk.lifetime = ros::Duration(0);
    text_mk.color.a = 1.0;
    text_mk.scale.z = 3;

    double y_diff(- 22);
    double vertical_diff(2);
    double x_base(33), z_base(4);
    /*line 1*/
    int line_num = 3;
    text_mk.color = Color::Black();
    text_mk.text = "Time (ms)";
    text_mk.pose.position.x = x_base + vertical_diff * line_num;
    text_mk.pose.position.y = 0;
    text_mk.pose.position.z = z_base + vertical_diff * line_num;
    text_mk.id = 0;
    mk_ary.markers.push_back(text_mk);

    text_mk.color = Color::Black();
    text_mk.text = "Jerk Cost";
    text_mk.pose.position.x = x_base + vertical_diff * line_num;
    text_mk.pose.position.y = y_diff;
    text_mk.pose.position.z = z_base + vertical_diff * line_num;
    text_mk.id = 1;
    mk_ary.markers.push_back(text_mk);
    /*line 1*/

    /*line 2*/
    line_num = 2;
    text_mk.color = Color::Blue();
    text_mk.text = "[Front-end w/]";
    text_mk.pose.position.x = x_base + vertical_diff * line_num;
    text_mk.pose.position.y = - y_diff;
    text_mk.pose.position.z = z_base + vertical_diff * line_num;
    text_mk.id = 2;
    mk_ary.markers.push_back(text_mk);

    text_mk.color = Color::Blue();
    text_mk.text = texts[0];
    text_mk.pose.position.x = x_base + vertical_diff * line_num;
    text_mk.pose.position.y = 0;
    text_mk.pose.position.z = z_base + vertical_diff * line_num;
    text_mk.id = 3;
    mk_ary.markers.push_back(text_mk);

    text_mk.color = Color::Blue();
    text_mk.text = texts[1];
    text_mk.pose.position.x = x_base + vertical_diff * line_num;
    text_mk.pose.position.y = y_diff;
    text_mk.pose.position.z = z_base + vertical_diff * line_num;
    text_mk.id = 4;
    mk_ary.markers.push_back(text_mk);
    /*line 2*/

    /*line 3*/
    line_num = 1;
    text_mk.color = Color::Orange();
    text_mk.text = "[Front-end w/o]";
    text_mk.pose.position.x = x_base + vertical_diff * line_num;
    text_mk.pose.position.y = - y_diff;
    text_mk.pose.position.z = z_base + vertical_diff * line_num;
    text_mk.id = 5;
    mk_ary.markers.push_back(text_mk);

    text_mk.color = Color::Orange();
    text_mk.text = texts[2];
    text_mk.pose.position.x = x_base + vertical_diff * line_num;
    text_mk.pose.position.y = 0;
    text_mk.pose.position.z = z_base + vertical_diff * line_num;
    text_mk.id = 6;
    mk_ary.markers.push_back(text_mk);

    text_mk.color = Color::Orange();
    text_mk.text = texts[3];
    text_mk.pose.position.x = x_base + vertical_diff * line_num;
    text_mk.pose.position.y = y_diff;
    text_mk.pose.position.z = z_base + vertical_diff * line_num;
    text_mk.id = 7;
    mk_ary.markers.push_back(text_mk);
    /*line 3*/

    /*line 4*/
    line_num = 0;
    text_mk.color = Color::Red();
    text_mk.text = "[Back-end]";
    text_mk.pose.position.x = x_base + vertical_diff * line_num;
    text_mk.pose.position.y = - y_diff;
    text_mk.pose.position.z = z_base + vertical_diff * line_num;
    text_mk.id = 8;
    mk_ary.markers.push_back(text_mk);

    text_mk.color = Color::Red();
    text_mk.text = texts[4];
    text_mk.pose.position.x = x_base + vertical_diff * line_num;
    text_mk.pose.position.y = 0;
    text_mk.pose.position.z = z_base + vertical_diff * line_num;
    text_mk.id = 9;
    mk_ary.markers.push_back(text_mk);

    text_mk.color = Color::Red();
    text_mk.text = texts[5];
    text_mk.pose.position.x = x_base + vertical_diff * line_num;
    text_mk.pose.position.y = y_diff;
    text_mk.pose.position.z = z_base + vertical_diff * line_num;
    text_mk.id = 10;
    mk_ary.markers.push_back(text_mk);
    /*line 4*/
    texts_pub_.publish(mk_ary);
}