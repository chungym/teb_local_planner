/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017.
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann
 *********************************************************************/

#include <teb_local_planner/teb_local_planner_ros.h>
//#include <teb_local_planner/TrajectoryPointSE2.h>

#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

using namespace teb_local_planner; // it is ok here to import everything for testing purposes

// ============= Global Variables ================
// Ok global variables are bad, but here we only have a simple testing node.
PlannerInterfacePtr planner;
TebVisualizationPtr visual;
std::vector<ObstaclePtr> obst_vector;
std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> dyn_obst_viapts;
ViaPointContainer via_points;
TebConfig config;
boost::shared_ptr< dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig> > dynamic_recfg;
ros::Subscriber custom_obst_sub;
ros::Subscriber via_points_sub;
ros::Subscriber clicked_points_sub;
std::vector<ros::Subscriber> obst_vel_subs;
unsigned int no_fixed_obstacles;

ros::Publisher obst_plan_pub;
ros::Publisher voronoi_pub;

interactive_markers::InteractiveMarkerServer* marker_server_ptr;

// =========== Function declarations =============
void CB_mainCycle(const ros::TimerEvent& e);
void CB_publishCycle(const ros::TimerEvent& e);
void CB_reconfigure(TebLocalPlannerReconfigureConfig& reconfig, uint32_t level);
void CB_customObstacle(const costmap_converter::ObstacleArrayMsg::ConstPtr& obst_msg);
void CreateInteractiveMarker(const double& init_x, const double& init_y, unsigned int id, std::string frame, interactive_markers::InteractiveMarkerServer* marker_server, interactive_markers::InteractiveMarkerServer::FeedbackCallback feedback_cb);
void CreateInteractiveMarker_Traj(const double& init_x, const double& init_y, const double& init_z, unsigned int id, std::string frame, interactive_markers::InteractiveMarkerServer* marker_server, interactive_markers::InteractiveMarkerServer::FeedbackCallback feedback_cb);
void CB_obstacle_marker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
void CB_obstacle_viapoint_marker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
void CB_clicked_points(const geometry_msgs::PointStampedConstPtr& point_msg);
void CB_via_points(const nav_msgs::Path::ConstPtr& via_points_msg);
void CB_setObstacleVelocity(const geometry_msgs::TwistConstPtr& twist_msg, const unsigned int id);

void generateTrajectroyFromViaPoints(std::vector<TrajectoryPointSE2>& trajectroy, std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>& dyn_obst_viapts);
void publish_obstacle_path(const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> >& points);

// =============== Main function =================
int main( int argc, char** argv )
{
  ros::init(argc, argv, "test_optim_node");
  ros::NodeHandle n("~");
 
  // load ros parameters from node handle
  config.loadRosParamFromNodeHandle(n);
 
  ros::Timer cycle_timer = n.createTimer(ros::Duration(0.025), CB_mainCycle);
  ros::Timer publish_timer = n.createTimer(ros::Duration(0.1), CB_publishCycle);
  
  // setup dynamic reconfigure
  dynamic_recfg = boost::make_shared< dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig> >(n);
  dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig>::CallbackType cb = boost::bind(CB_reconfigure, _1, _2);
  dynamic_recfg->setCallback(cb);
  
  // setup callback for custom obstacles
  custom_obst_sub = n.subscribe("obstacles", 1, CB_customObstacle);
  
  // setup callback for clicked points (in rviz) that are considered as via-points
  clicked_points_sub = n.subscribe("/clicked_point", 5, CB_clicked_points);
  
  // setup callback for via-points (callback overwrites previously set via-points)
  via_points_sub = n.subscribe("via_points", 1, CB_via_points);

  // setup publisher for obstacle via points
  obst_plan_pub = n.advertise<nav_msgs::Path>("obst_plan",1);

  voronoi_pub = n.advertise<nav_msgs::OccupancyGrid>("voronoi",1);

  // interactive marker server for simulated dynamic obstacles
  interactive_markers::InteractiveMarkerServer marker_server("marker_obstacles");
  marker_server_ptr = &marker_server;

  obst_vector.push_back( boost::make_shared<PointObstacle>(-3,1) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(6,2) );
  obst_vector.push_back( boost::make_shared<PointObstacle>(0,-1) );
//  obst_vector.push_back( boost::make_shared<LineObstacle>(1,1.5,1,-1.5) ); //90 deg
//  obst_vector.push_back( boost::make_shared<LineObstacle>(1,0,-1,0) ); //180 deg
//  obst_vector.push_back( boost::make_shared<PointObstacle>(-1.5,-0.5) );

  // Dynamic obstacles
  Eigen::Vector2d vel (0.1, -0.3);
  obst_vector.at(0)->setCentroidVelocity(vel);
  vel = Eigen::Vector2d(-0.3, -0.2);
  obst_vector.at(1)->setCentroidVelocity(vel);

  if (!config.obstacles.include_dynamic_obstacles)
  {
    obst_vector.at(0)->disableDynamic();
    obst_vector.at(1)->disableDynamic();
  }     


  /*
  PolygonObstacle* polyobst = new PolygonObstacle;
  polyobst->pushBackVertex(1, -1);
  polyobst->pushBackVertex(0, 1);
  polyobst->pushBackVertex(1, 1);
  polyobst->pushBackVertex(2, 1);
 
  polyobst->finalizePolygon();
  obst_vector.emplace_back(polyobst);
  */
  
  // Dynamic obstacle with trajectory defined by via points
  // defined as (x, y, theta,time)
  dyn_obst_viapts.push_back(Eigen::Vector4d(1, 1, M_PI_2, 0));
  dyn_obst_viapts.push_back(Eigen::Vector4d(1, 2, M_PI_2, 5));
  dyn_obst_viapts.push_back(Eigen::Vector4d(1, 3, M_PI_2, 10));
  dyn_obst_viapts.push_back(Eigen::Vector4d(1, 4, M_PI_2, 15));

  for (unsigned int i=0; i<obst_vector.size(); ++i)
  {
    // setup callbacks for setting obstacle velocities
    std::string topic = "/test_optim_node/obstacle_" + std::to_string(i) + "/cmd_vel";
    obst_vel_subs.push_back(n.subscribe<geometry_msgs::Twist>(topic, 1, boost::bind(&CB_setObstacleVelocity, _1, i)));

    //CreateInteractiveMarker(obst_vector.at(i)[0],obst_vector.at(i)[1],i,&marker_server, &CB_obstacle_marker);  
    // Add interactive markers for all point obstacles
    boost::shared_ptr<PointObstacle> pobst = boost::dynamic_pointer_cast<PointObstacle>(obst_vector.at(i));
    if (pobst)
    {
      CreateInteractiveMarker(pobst->x(),pobst->y(),i, config.map_frame, &marker_server, &CB_obstacle_marker);  
    }
  }

  for (unsigned int i=0; i<dyn_obst_viapts.size(); ++i)
  {
    // Add interactive markers for obstacle trajectory via points
    Eigen::Vector4d& pt = dyn_obst_viapts.at(i);
    if (&pt)
    {
      CreateInteractiveMarker_Traj(pt.x(),pt.y(),pt.z(), i, config.map_frame, &marker_server, &CB_obstacle_viapoint_marker);
    }
  }
  
  CircularObstacle* obstacle_withTrajectory = new CircularObstacle;

  std::vector<TrajectoryPointSE2> trajectory;
  generateTrajectroyFromViaPoints(trajectory, dyn_obst_viapts);

  obstacle_withTrajectory->setTrajectory(trajectory, !config.trajectory.exact_arc_length);
  obstacle_withTrajectory->radius() = 0.5;
  obstacle_withTrajectory->position() = obstacle_withTrajectory->getInitPose().position();
  if (!config.obstacles.include_obstacle_trajectory){obstacle_withTrajectory->disableTrajectory();}
  if (!config.obstacles.include_dynamic_obstacles){obstacle_withTrajectory->disableDynamic();}     

  obst_vector.push_back(ObstaclePtr(obstacle_withTrajectory));

  marker_server.applyChanges();
  
  // Setup visualization
  visual = TebVisualizationPtr(new TebVisualization(n, config));
  
  // Setup robot shape model
  RobotFootprintModelPtr robot_model = TebLocalPlannerROS::getRobotFootprintFromParamServer(n);
  
  // setup voronoi graph
  boost::shared_ptr<dynamicvoronoi::BoostVoronoi> voronoiPtr(boost::make_shared<dynamicvoronoi::BoostVoronoi>());

  // Setup planner (homotopy class planning or just the local teb planner)
  if (config.hcp.enable_homotopy_class_planning)
  {
    planner = PlannerInterfacePtr(new HomotopyClassPlanner(config, &obst_vector, robot_model, visual, &via_points));
    boost::dynamic_pointer_cast<HomotopyClassPlanner> (planner)->setVoronoi(voronoiPtr);
  }
  else
  {
    planner = PlannerInterfacePtr(new TebOptimalPlanner(config, &obst_vector, robot_model, visual, &via_points));
  }


  no_fixed_obstacles = obst_vector.size();
  ros::spin();


  return 0;
}

// Planning loop
void CB_mainCycle(const ros::TimerEvent& e)
{
  nav_msgs::OccupancyGrid::Ptr grid = boost::make_shared<nav_msgs::OccupancyGrid>();

  grid->header.frame_id = config.map_frame;
  grid->header.stamp = ros::Time::now();
  grid->info.resolution = 0.05;

  grid->info.width = 300;
  grid->info.height = 300;

  grid->info.origin.position.x = -7.5;
  grid->info.origin.position.y = -7.5;
  grid->info.origin.position.z = 0.0;
  grid->info.origin.orientation.w = 1.0;

  grid->data = std::vector<int8_t, std::allocator<int8_t>>(300*300);

  if (config.hcp.enable_homotopy_class_planning)
  {
    boost::dynamic_pointer_cast<HomotopyClassPlanner> (planner)->getVoronoi()->setOccupancyGrid(grid);
  }

  planner->plan(PoseSE2(-4,0,0), PoseSE2(4,0,0)); // hardcoded start and goal for testing purposes

  grid->data.clear();
}

// Visualization loop
void CB_publishCycle(const ros::TimerEvent& e)
{
  planner->visualize(ros::Time::now());
  visual->publishObstacles(obst_vector);
  visual->publishViaPoints(via_points);

  publish_obstacle_path(dyn_obst_viapts);

  if (config.hcp.enable_homotopy_class_planning)
  {
    boost::mutex::scoped_lock l(*boost::dynamic_pointer_cast<HomotopyClassPlanner> (planner)->getVoronoi()->getGridMutex());
    voronoi_pub.publish(boost::dynamic_pointer_cast<HomotopyClassPlanner> (planner)->getVoronoi()->getVisualisation());
  }

}

void CB_reconfigure(TebLocalPlannerReconfigureConfig& reconfig, uint32_t level)
{
  config.reconfigure(reconfig);
}

void CreateInteractiveMarker(const double& init_x, const double& init_y, unsigned int id, std::string frame, interactive_markers::InteractiveMarkerServer* marker_server, interactive_markers::InteractiveMarkerServer::FeedbackCallback feedback_cb)
{
  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker i_marker;
  i_marker.header.frame_id = frame;
  i_marker.header.stamp = ros::Time::now();
  std::ostringstream oss;
  //oss << "obstacle" << id;
  oss << id;
  i_marker.name = oss.str();
  i_marker.description = "Obstacle";
  i_marker.pose.position.x = init_x;
  i_marker.pose.position.y = init_y;
  i_marker.pose.orientation.w = 1.0f; // make quaternion normalized

  // create a grey box marker
  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::CUBE;
  box_marker.id = id;
  box_marker.scale.x = 0.2;
  box_marker.scale.y = 0.2;
  box_marker.scale.z = 0.2;
  box_marker.color.r = 0.5;
  box_marker.color.g = 0.5;
  box_marker.color.b = 0.5;
  box_marker.color.a = 1.0;
  box_marker.pose.orientation.w = 1.0f; // make quaternion normalized

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  box_control.markers.push_back( box_marker );

  // add the control to the interactive marker
  i_marker.controls.push_back( box_control );

  // create a control which will move the box, rviz will insert 2 arrows
  visualization_msgs::InteractiveMarkerControl move_control;
  move_control.name = "move_x";
  move_control.orientation.w = 0.707107f;
  move_control.orientation.x = 0;
  move_control.orientation.y = 0.707107f;
  move_control.orientation.z = 0;
  move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;


  // add the control to the interactive marker
  i_marker.controls.push_back(move_control);

  // add the interactive marker to our collection
  marker_server->insert(i_marker);
  marker_server->setCallback(i_marker.name,feedback_cb);
}

void CreateInteractiveMarker_Traj(const double& init_x, const double& init_y, const double& init_theta, unsigned int id, std::string frame, interactive_markers::InteractiveMarkerServer* marker_server, interactive_markers::InteractiveMarkerServer::FeedbackCallback feedback_cb)
{
  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker i_marker;
  i_marker.header.frame_id = frame;
  i_marker.header.stamp = ros::Time::now();
  std::ostringstream oss;
  //oss << "obstacle" << id;
  oss << "p" << id;
  i_marker.name = oss.str();
  i_marker.description = "Obstacle via-point";
  i_marker.pose.position.x = init_x;
  i_marker.pose.position.y = init_y;
  i_marker.pose.orientation = tf::createQuaternionMsgFromYaw(init_theta);; // make quaternion normalized


  // create a grey arrow marker
  visualization_msgs::Marker arrow_marker;
  arrow_marker.type = visualization_msgs::Marker::ARROW;
  arrow_marker.id = id;
  arrow_marker.scale.x = 0.2;
  arrow_marker.scale.y = 0.2;
  arrow_marker.scale.z = 0.2;
  arrow_marker.color.r = 0.5;
  arrow_marker.color.g = 0.5;
  arrow_marker.color.b = 0.5;
  arrow_marker.color.a = 1.0;
  arrow_marker.pose.orientation.w = 1.0f; // make quaternion normalized

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl arrow_control;
  arrow_control.always_visible = true;
  arrow_control.markers.push_back( arrow_marker );

  // add the control to the interactive marker
  i_marker.controls.push_back( arrow_control );


  // create a control which will move the marker, rviz will insert 2 arrows
  visualization_msgs::InteractiveMarkerControl move_control;
  move_control.name = "move_x";
  move_control.orientation.w = 0.707107f;
  move_control.orientation.x = 0.707107f;
  move_control.orientation.y = 0;
  move_control.orientation.z = 0;
  move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  move_control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;

  // add the control to the interactive marker
  i_marker.controls.push_back(move_control);

  // add another direction
  move_control.name = "move_z";
  move_control.orientation.x = 0;
  move_control.orientation.z = 0.707107f;
  i_marker.controls.push_back(move_control);

  //add rotation
  if (id==0)
  {
    move_control.orientation.w = 0.707107f;
    move_control.orientation.x = 0;
    move_control.orientation.y = 0.707107f;
    move_control.orientation.z = 0;
    move_control.name = "rotate_y";
    move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    i_marker.controls.push_back(move_control);
  }

  // add the interactive marker to our collection
  marker_server->insert(i_marker);
  marker_server->setCallback(i_marker.name,feedback_cb);
}

void CB_obstacle_marker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  std::stringstream ss(feedback->marker_name);
  unsigned int index;
  ss >> index;
  
  if (index>=no_fixed_obstacles) 
    return;
  PointObstacle* pobst = static_cast<PointObstacle*>(obst_vector.at(index).get());
  pobst->position() = Eigen::Vector2d(feedback->pose.position.x,feedback->pose.position.y);	  
  if (!config.obstacles.include_dynamic_obstacles){pobst->disableDynamic();}     
}

void CB_obstacle_viapoint_marker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  std::stringstream ss(feedback->marker_name);
  unsigned int index;
  char ch;
  ss >> ch; // skip initial >
  ss >> index;
  Eigen::Vector4d& pt = dyn_obst_viapts.at(index);
  pt.x() = feedback->pose.position.x;
  pt.y() = feedback->pose.position.y;	  
  pt.z() = tf::getYaw(feedback->pose.orientation);

  for (unsigned int i=1; i<dyn_obst_viapts.size();i++)
  {
    Eigen::Vector2d deltaS = dyn_obst_viapts.at(i).head(2) - dyn_obst_viapts.at(i-1).head(2);
    double path_angle = atan2(deltaS(1),deltaS(0));
    dyn_obst_viapts.at(i).z() = 2 * path_angle - dyn_obst_viapts.at(i-1).z();
    std::string mark_name_i = 'p' + std::to_string(i);
    geometry_msgs::Pose pose;
    pose.position.x = dyn_obst_viapts.at(i).x();
    pose.position.y = dyn_obst_viapts.at(i).y();
    pose.position.z = 0;
    pose.orientation = tf::createQuaternionMsgFromYaw(dyn_obst_viapts.at(i).z());
    marker_server_ptr->setPose( mark_name_i, pose );
  }

  marker_server_ptr->applyChanges();

  CircularObstacle* pobst = static_cast<CircularObstacle*>(obst_vector.back().get());
  std::vector<TrajectoryPointSE2> trajectory;
  generateTrajectroyFromViaPoints(trajectory, dyn_obst_viapts);
  pobst->setTrajectory(trajectory, !config.trajectory.exact_arc_length);
  pobst->position() = pobst->getInitPose().position();
  if (!config.obstacles.include_obstacle_trajectory){pobst->disableTrajectory();}
  if (!config.obstacles.include_dynamic_obstacles){pobst->disableDynamic();}     

}

void CB_customObstacle(const costmap_converter::ObstacleArrayMsg::ConstPtr& obst_msg)
{
  // resize such that the vector contains only the fixed obstacles specified inside the main function
  obst_vector.resize(no_fixed_obstacles);
  
  // Add custom obstacles obtained via message (assume that all obstacles coordiantes are specified in the default planning frame)  
  for (size_t i = 0; i < obst_msg->obstacles.size(); ++i)
  {
    if (obst_msg->obstacles.at(i).polygon.points.size() == 1 )
    {
      if (obst_msg->obstacles.at(i).radius == 0) 
      {
        obst_vector.push_back(ObstaclePtr(new PointObstacle( obst_msg->obstacles.at(i).polygon.points.front().x,
                                                           obst_msg->obstacles.at(i).polygon.points.front().y )));
      }
      else
      {
        obst_vector.push_back(ObstaclePtr(new CircularObstacle( obst_msg->obstacles.at(i).polygon.points.front().x,
                                                            obst_msg->obstacles.at(i).polygon.points.front().y,
                                                            obst_msg->obstacles.at(i).radius )));
      }
    }
    else
    {
      PolygonObstacle* polyobst = new PolygonObstacle;
      for (size_t j=0; j<obst_msg->obstacles.at(i).polygon.points.size(); ++j)
      {
        polyobst->pushBackVertex( obst_msg->obstacles.at(i).polygon.points[j].x,
                                  obst_msg->obstacles.at(i).polygon.points[j].y );
      }
      polyobst->finalizePolygon();
      obst_vector.push_back(ObstaclePtr(polyobst));
    }

    if(!obst_vector.empty())
      obst_vector.back()->setCentroidVelocity(obst_msg->obstacles.at(i).velocities, obst_msg->obstacles.at(i).orientation);
  }
}


void CB_clicked_points(const geometry_msgs::PointStampedConstPtr& point_msg)
{
  // we assume for simplicity that the fixed frame is already the map/planning frame
  // consider clicked points as via-points
  via_points.push_back( Eigen::Vector2d(point_msg->point.x, point_msg->point.y) );
  ROS_INFO_STREAM("Via-point (" << point_msg->point.x << "," << point_msg->point.y << ") added.");
  if (config.optim.weight_viapoint<=0)
    ROS_WARN("Note, via-points are deactivated, since 'weight_via_point' <= 0");
}

void CB_via_points(const nav_msgs::Path::ConstPtr& via_points_msg)
{
  ROS_INFO_ONCE("Via-points received. This message is printed once.");
  via_points.clear();
  for (const geometry_msgs::PoseStamped& pose : via_points_msg->poses)
  {
    via_points.emplace_back(pose.pose.position.x, pose.pose.position.y);
  }
}

void CB_setObstacleVelocity(const geometry_msgs::TwistConstPtr& twist_msg, const unsigned int id)
{
  if (id >= obst_vector.size())
  {
    ROS_WARN("Cannot set velocity: unknown obstacle id.");
    return;
  }

  Eigen::Vector2d vel (twist_msg->linear.x, twist_msg->linear.y);
  obst_vector.at(id)->setCentroidVelocity(vel);
}

void generateTrajectroyFromViaPoints(std::vector<TrajectoryPointSE2>& trajectroy, std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>>& dyn_obst_viapts)
{
  for (unsigned int i=0; i<dyn_obst_viapts.size(); ++i)
  {
    Eigen::Vector4d pt = dyn_obst_viapts.at(i);
    TrajectoryPointSE2 ptSE2; 
    ptSE2.pose.x = pt.x();
    ptSE2.pose.y = pt.y();
    ptSE2.pose.theta = pt.z();
    ptSE2.time_from_start = ros::Duration(pt.w());
    
    double angle_diff, dist, deltaT;
    Eigen::Vector2d deltaS;

    if (i < dyn_obst_viapts.size()-1)
    {
      Eigen::Vector4d pt2 = dyn_obst_viapts.at(i+1);
      deltaT = pt2.w()-pt.w();
      deltaS = pt2.head(2) - pt.head(2);
      dist = deltaS.norm();
      angle_diff = g2o::normalize_theta(pt2.z() - pt.z());
    }
    else
    {
      deltaS = dyn_obst_viapts.at(i).head(2) - dyn_obst_viapts.at(i-1).head(2);
      deltaT = dyn_obst_viapts.at(i).w()-dyn_obst_viapts.at(i-1).w();
      dist = deltaS.norm();
      angle_diff = g2o::normalize_theta(dyn_obst_viapts.at(i).z() - dyn_obst_viapts.at(i-1).z());
    }
    if (angle_diff != 0 && config.trajectory.exact_arc_length)
    {
        double radius =  dist/(2*sin(angle_diff/2));
        dist = fabs( angle_diff * radius ); // actual arg length!
        ptSE2.velocity.linear.x = cos(dyn_obst_viapts.at(i).z()) * dist / deltaT;
        ptSE2.velocity.linear.y = sin(dyn_obst_viapts.at(i).z()) * dist / deltaT;
    }
    else
    {
      ptSE2.velocity.linear.x = deltaS.x() / deltaT;
      ptSE2.velocity.linear.y = deltaS.y() / deltaT;
    }
    ptSE2.velocity.angular.z = angle_diff / deltaT;
    
    trajectroy.push_back(ptSE2);
  }
}


void publish_obstacle_path(const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> >& points)
{
    // create path msg
    nav_msgs::Path obst_path;
    obst_path.header.frame_id = config.map_frame;
    obst_path.header.stamp = ros::Time::now();
    
    // fill path msgs with teb configurations
    for (int i=0; i < points.size(); i++)
    {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = obst_path.header.frame_id;
      pose.header.stamp = obst_path.header.stamp;
      pose.pose.position.x = points.at(i).x();
      pose.pose.position.y = points.at(i).y();
      obst_path.poses.push_back(pose);
    }
    obst_plan_pub.publish(obst_path);

}
