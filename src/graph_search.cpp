/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017,
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
 * Authors: Christoph Rösmann, Franz Albers
 *********************************************************************/

#include <teb_local_planner/graph_search.h>
#include <teb_local_planner/homotopy_class_planner.h>

namespace teb_local_planner
{

void GraphSearchInterface::DepthFirst(HcGraph& g, std::vector<HcGraphVertexType>& visited, bool*& visited_array, const HcGraphVertexType& goal, double start_orientation,
                                      double goal_orientation, const geometry_msgs::Twist* start_velocity)
{
  // see http://www.technical-recipes.com/2011/a-recursive-algorithm-to-find-all-paths-between-two-given-nodes/ for details on finding all simple paths

  if ((int)hcp_->getTrajectoryContainer().size() >= cfg_->hcp.max_number_classes)
    return; // We do not need to search for further possible alternative homotopy classes.

  HcGraphVertexType back = visited.back();

  /// Examine adjacent nodes
  HcGraphAdjecencyIterator it, end;
  for ( boost::tie(it,end) = boost::adjacent_vertices(back,g); it!=end; ++it)
  {
    if ( visited_array[*it] )
      continue; // already visited

    if ( *it == goal ) // goal reached
    {
      visited.push_back(*it);

      // Add new TEB, if this path belongs to a new homotopy class
      hcp_->addAndInitNewTeb(visited.begin(), visited.end(), boost::bind(getVector2dFromHcGraph, _1, boost::cref(graph_)),
                             start_orientation, goal_orientation, start_velocity);

      visited.pop_back();
      break;
    }
  }

  /// Recursion for all adjacent vertices
  for ( boost::tie(it,end) = boost::adjacent_vertices(back,g); it!=end; ++it)
  {
    if ( visited_array[*it] || *it == goal)
      continue; // already visited || goal reached


    visited.push_back(*it);
    visited_array[*it] = true;

    // recursion step
    DepthFirst(g, visited, visited_array, goal, start_orientation, goal_orientation, start_velocity);

    visited.pop_back();
    visited_array[*it] = false;
  }
}



void lrKeyPointGraph::createGraph(const PoseSE2& start, const PoseSE2& goal, double dist_to_obst, double obstacle_heading_threshold, const geometry_msgs::Twist* start_velocity)
{
  // Clear existing graph and paths
  clearGraph();
  if((int)hcp_->getTrajectoryContainer().size() >= cfg_->hcp.max_number_classes)
    return;
  // Direction-vector between start and goal and normal-vector:
  Eigen::Vector2d diff = goal.position()-start.position();

  if (diff.norm()<cfg_->goal_tolerance.xy_goal_tolerance)
  {
    ROS_DEBUG("HomotopyClassPlanner::createProbRoadmapGraph(): xy-goal-tolerance already reached.");
    if (hcp_->getTrajectoryContainer().empty())
    {
      ROS_INFO("HomotopyClassPlanner::createProbRoadmapGraph(): Initializing a small straight line to just correct orientation errors.");
      hcp_->addAndInitNewTeb(start, goal, start_velocity);
    }
    return;
  }

  Eigen::Vector2d normal(-diff[1],diff[0]); // normal-vector
  normal.normalize();
  normal = normal*dist_to_obst; // scale with obstacle_distance;

  // Insert Vertices
  HcGraphVertexType start_vtx = boost::add_vertex(graph_); // start vertex
  graph_[start_vtx].pos = start.position();
  diff.normalize();

  // store nearest obstacle keypoints -> only used if limit_obstacle_heading is enabled
  std::pair<HcGraphVertexType,HcGraphVertexType> nearest_obstacle; // both vertices are stored
  double min_dist = DBL_MAX;

  if (hcp_->obstacles()!=NULL)
  {
    for (ObstContainer::const_iterator it_obst = hcp_->obstacles()->begin(); it_obst != hcp_->obstacles()->end(); ++it_obst)
    {
      // check if obstacle is placed in front of start point
      Eigen::Vector2d start2obst = (*it_obst)->getCentroid() - start.position();
      double dist = start2obst.norm();
      if (start2obst.dot(diff)/dist<0.1)
        continue;

      // Add Keypoints
      HcGraphVertexType u = boost::add_vertex(graph_);
      graph_[u].pos = (*it_obst)->getCentroid() + normal;
      HcGraphVertexType v = boost::add_vertex(graph_);
      graph_[v].pos = (*it_obst)->getCentroid() - normal;

      // store nearest obstacle
      if (obstacle_heading_threshold && dist<min_dist)
      {
        min_dist = dist;
        nearest_obstacle.first = u;
        nearest_obstacle.second = v;
      }
    }
  }

  HcGraphVertexType goal_vtx = boost::add_vertex(graph_); // goal vertex
  graph_[goal_vtx].pos = goal.position();

  // Insert Edges
  HcGraphVertexIterator it_i, end_i, it_j, end_j;
  for (boost::tie(it_i,end_i) = boost::vertices(graph_); it_i!=end_i-1; ++it_i) // ignore goal in this loop
  {
    for (boost::tie(it_j,end_j) = boost::vertices(graph_); it_j!=end_j; ++it_j) // check all forward connections
    {
      if (it_i==it_j)
        continue;
      // TODO: make use of knowing in which order obstacles are inserted and that for each obstacle 2 vertices are added,
      // therefore we must only check one of them.
      Eigen::Vector2d distij = graph_[*it_j].pos-graph_[*it_i].pos;
      distij.normalize();
      // Check if the direction is backwards:
      if (distij.dot(diff)<=obstacle_heading_threshold)
        continue;


      // Check start angle to nearest obstacle
      if (obstacle_heading_threshold && *it_i==start_vtx && min_dist!=DBL_MAX)
      {
        if (*it_j == nearest_obstacle.first || *it_j == nearest_obstacle.second)
        {
          Eigen::Vector2d keypoint_dist = graph_[*it_j].pos-start.position();
          keypoint_dist.normalize();
          Eigen::Vector2d start_orient_vec( cos(start.theta()), sin(start.theta()) ); // already normalized
          // check angle
          if (start_orient_vec.dot(keypoint_dist) <= obstacle_heading_threshold)
          {
            ROS_DEBUG("createGraph() - deleted edge: limit_obstacle_heading");
            continue;
          }
        }
      }

      // Collision Check

      if (hcp_->obstacles()!=NULL)
      {
        bool collision = false;
        for (ObstContainer::const_iterator it_obst = hcp_->obstacles()->begin(); it_obst != hcp_->obstacles()->end(); ++it_obst)
        {
          if ( (*it_obst)->checkLineIntersection(graph_[*it_i].pos,graph_[*it_j].pos, 0.5*dist_to_obst) )
          {
            collision = true;
            break;
          }
        }
        if (collision)
          continue;
      }

      // Create Edge
      boost::add_edge(*it_i,*it_j,graph_);
    }
  }


  // Find all paths between start and goal!
  std::vector<HcGraphVertexType> visited;
  visited.push_back(start_vtx);

  bool* visited_array = new bool[boost::num_vertices(graph_)]();
  visited_array[start_vtx] = true;

  DepthFirst(graph_, visited, visited_array, goal_vtx, start.theta(), goal.theta(), start_velocity);

  delete[] visited_array;
  visited_array = NULL;
}



void ProbRoadmapGraph::createGraph(const PoseSE2& start, const PoseSE2& goal, double dist_to_obst, double obstacle_heading_threshold, const geometry_msgs::Twist* start_velocity)
{
  // Clear existing graph and paths
  clearGraph();
  if((int)hcp_->getTrajectoryContainer().size() >= cfg_->hcp.max_number_classes)
    return;
  // Direction-vector between start and goal and normal-vector:
  Eigen::Vector2d diff = goal.position()-start.position();
  double start_goal_dist = diff.norm();

  if (start_goal_dist<cfg_->goal_tolerance.xy_goal_tolerance)
  {
    ROS_DEBUG("HomotopyClassPlanner::createProbRoadmapGraph(): xy-goal-tolerance already reached.");
    if (hcp_->getTrajectoryContainer().empty())
    {
      ROS_INFO("HomotopyClassPlanner::createProbRoadmapGraph(): Initializing a small straight line to just correct orientation errors.");
      hcp_->addAndInitNewTeb(start, goal, start_velocity);
    }
    return;
  }
  Eigen::Vector2d normal(-diff.coeffRef(1),diff.coeffRef(0)); // normal-vector
  normal.normalize();

  // Now sample vertices between start, goal and a specified width between both sides
  // Let's start with a square area between start and goal (maybe change it later to something like a circle or whatever)

  double area_width = cfg_->hcp.roadmap_graph_area_width;

  boost::random::uniform_real_distribution<double> distribution_x(0, start_goal_dist * cfg_->hcp.roadmap_graph_area_length_scale);
  boost::random::uniform_real_distribution<double> distribution_y(0, area_width);

  double phi = atan2(diff.coeffRef(1),diff.coeffRef(0)); // rotate area by this angle
  Eigen::Rotation2D<double> rot_phi(phi);

  Eigen::Vector2d area_origin;
  if (cfg_->hcp.roadmap_graph_area_length_scale != 1.0)
    area_origin = start.position() + 0.5*(1.0-cfg_->hcp.roadmap_graph_area_length_scale)*start_goal_dist*diff.normalized() - 0.5*area_width*normal; // bottom left corner of the origin
  else
    area_origin = start.position() - 0.5*area_width*normal; // bottom left corner of the origin

  // Insert Vertices
  HcGraphVertexType start_vtx = boost::add_vertex(graph_); // start vertex
  graph_[start_vtx].pos = start.position();
  diff.normalize(); // normalize in place


  // Start sampling
  for (int i=0; i < cfg_->hcp.roadmap_graph_no_samples; ++i)
  {
    Eigen::Vector2d sample;
     bool coll_free;
//     do // sample as long as a collision free sample is found
//     {
      // Sample coordinates
      sample = area_origin + rot_phi*Eigen::Vector2d(distribution_x(rnd_generator_), distribution_y(rnd_generator_));

      // Test for collision
      // we do not care for collision checking here to improve efficiency, since we perform resampling repeatedly.
      // occupied vertices are ignored in the edge insertion state since they always violate the edge-obstacle collision check.
       coll_free = true;
       for (ObstContainer::const_iterator it_obst = hcp_->obstacles()->begin(); it_obst != hcp_->obstacles()->end(); ++it_obst)
       {
         // skip dynamic obstacles
         if ( (*it_obst)->isDynamic() )
          continue;

         if ( (*it_obst)->checkCollision(sample, std::max(cfg_->obstacles.min_obstacle_dist, hcp_->getInscribedRadius()))) // TODO really keep dist_to_obst here?
         {
           coll_free = false;
           break;
         }
       }
//
    if (!coll_free ){continue;}

    // Add new vertex
    HcGraphVertexType v = boost::add_vertex(graph_);
    graph_[v].pos = sample;
  }

  // Now add goal vertex
  HcGraphVertexType goal_vtx = boost::add_vertex(graph_); // goal vertex
  graph_[goal_vtx].pos = goal.position();


  // Insert Edges
  HcGraphVertexIterator it_i, end_i, it_j, end_j;
  for (boost::tie(it_i,end_i) = boost::vertices(graph_); it_i!=boost::prior(end_i); ++it_i) // ignore goal in this loop
  {
    for (boost::tie(it_j,end_j) = boost::vertices(graph_); it_j!=end_j; ++it_j) // check all forward connections
    {
      if (it_i==it_j) // same vertex found
        continue;

      Eigen::Vector2d distij = graph_[*it_j].pos-graph_[*it_i].pos;
      distij.normalize(); // normalize in place

      // Check if the direction is backwards:
      if (distij.dot(diff)<=obstacle_heading_threshold)
          continue; // diff is already normalized


      // Collision Check
      bool collision = false;
      for (ObstContainer::const_iterator it_obst = hcp_->obstacles()->begin(); it_obst != hcp_->obstacles()->end(); ++it_obst)
      {
        // skip dynamic obstacles
        if ( (*it_obst)->isDynamic() )
          continue;

        if ( (*it_obst)->checkLineIntersection(graph_[*it_i].pos,graph_[*it_j].pos, std::max(cfg_->obstacles.min_obstacle_dist, hcp_->getInscribedRadius()*0.5)) )
        {
          collision = true;
          break;
        }
      }
      if (collision)
        continue;

      // Create Edge
      boost::add_edge(*it_i,*it_j,graph_);
    }
  }

  /// Find all paths between start and goal!
  std::vector<HcGraphVertexType> visited;
  visited.push_back(start_vtx);

  bool* visited_array = new bool[boost::num_vertices(graph_)]();
  visited_array[start_vtx] = true;

  DepthFirst(graph_, visited, visited_array, goal_vtx, start.theta(), goal.theta(), start_velocity);

  delete[] visited_array;
  visited_array = NULL;


  // augment with Voronoi roadmap

  if (cfg_->hcp.voronoi_exploration && (int)hcp_->getTrajectoryContainer().size() < cfg_->hcp.max_number_classes)
  {

    hcp_->getVoronoi()->updateGridFromCostmap(); // there is internal checking whether a costmap is set
    

    for (auto it = hcp_->obstacles()->begin(); it != hcp_->obstacles()->end(); it++)
    {
      if (it->get()->isDynamic()){continue;}

      Eigen::Vector2d pos = it->get()->getCentroid();

      if (boost::dynamic_pointer_cast<PointObstacle>(*it) != nullptr)
      {
        hcp_->getVoronoi()->addCircularObstacle(pos.x(), pos.y(), cfg_->obstacles.min_obstacle_dist);
      }
      else if (boost::dynamic_pointer_cast<CircularObstacle>(*it) != nullptr)
      {
        hcp_->getVoronoi()->addCircularObstacle(pos.x(), pos.y(), boost::dynamic_pointer_cast<CircularObstacle>(*it)->radius());
      }
      else if (boost::dynamic_pointer_cast<LineObstacle>(*it) != nullptr)
      {
        boost::shared_ptr<LineObstacle> ptr = boost::dynamic_pointer_cast<LineObstacle>(*it);
        hcp_->getVoronoi()->addPillObstacle(ptr->start(), ptr->end(), cfg_->obstacles.min_obstacle_dist);
      }
      else if (boost::dynamic_pointer_cast<PillObstacle>(*it) != nullptr)
      {
        boost::shared_ptr<PillObstacle> ptr = boost::dynamic_pointer_cast<PillObstacle>(*it);
        hcp_->getVoronoi()->addPillObstacle(ptr->start(), ptr->end(), ptr->radius());
      }
      else if (boost::dynamic_pointer_cast<PolygonObstacle>(*it) != nullptr)
      {
        hcp_->getVoronoi()->addPolygonObstacle(boost::dynamic_pointer_cast<PolygonObstacle>(*it)->vertices());
      }
    
    }

    // robot footprint
    //hcp_->getVoronoi()->addCircularObstacle(start.position().x(), start.position().y(), std::max(hcp_->getInscribedRadius(), cfg_->obstacles.min_obstacle_dist)); 

    hcp_->getVoronoi()->updateGraph();

    
    boost::optional<std::pair<teb_local_planner::HcGraphVertexType, teb_local_planner::HcGraphVertexType>> start_goal
      = hcp_->getVoronoi()->addStartAndGoal(start.position(), goal.position()); 

    
    if (start_goal)
    {
      HcGraphVertexType start_vtx, goal_vtx;
      start_vtx = (*start_goal).first;
      goal_vtx = (*start_goal).second;
      /// Find all paths between start and goal!
      std::vector<HcGraphVertexType> visited;
      visited.push_back(start_vtx);

      // not neccessary to lock due to single threading, just in case someone creates a separate thread outside
      boost::mutex::scoped_lock l(*hcp_->getVoronoi()->getMutex());

      graph_ = *hcp_->getVoronoi()->getGraph();

      std::unordered_map<teb_local_planner::HcGraphVertexType, bool> visited_map;
      visited_map.emplace(start_vtx, true);

      //DepthFirst( graph_, visited, visited_array, goal_vtx, start.theta(), goal.theta(), start_velocity);

      // Breadth First search seems better 
      std::queue<std::pair<std::vector<HcGraphVertexType>, std::unordered_map<teb_local_planner::HcGraphVertexType, bool>>> queue;
      queue.push(std::make_pair(visited,visited_map));

      int count = 0;
      while (!queue.empty() && count < 2000)
      {
        count++;
        // get the first path from the queue
        std::vector<HcGraphVertexType> path = queue.front().first;
        std::unordered_map<teb_local_planner::HcGraphVertexType, bool> path_map = queue.front().second;
        queue.pop();

        HcGraphVertexType back = path.back();

        if ((int)hcp_->getTrajectoryContainer().size() >= cfg_->hcp.max_number_classes)
          break; // We do not need to search for further possible alternative homotopy classes.

        HcGraphAdjecencyIterator it, end;
        for ( boost::tie(it,end) = boost::adjacent_vertices(back, graph_); it!=end; ++it)
        {

          if (*it == goal_vtx)
          {
            std::vector<HcGraphVertexType> new_path = std::vector<HcGraphVertexType>(path);
            new_path.push_back(*it);

            double dist = 0;
            auto prev_it = new_path.begin();
            // the path should contains at least 2 elements, start & goal
            for (auto path_it = std::next(prev_it); path_it != new_path.end(); path_it++)
            {
              dist += (graph_[*path_it].pos - graph_[*prev_it].pos).norm();
              prev_it = path_it;
            }

            if (dist < cfg_->hcp.max_ratio_detours_duration_best_duration * cfg_->trajectory.max_global_plan_lookahead_dist)
            {
              // Add new TEB, if this path belongs to a new homotopy class
              hcp_->addAndInitNewTeb(new_path.begin(), new_path.end(), boost::bind(getVector2dFromHcGraph, _1, boost::cref(graph_)),
                                    start.theta(), goal.theta(), start_velocity);
            }

            continue;
          } 

          if ( path_map.find(*it) != path_map.end() )
            continue; // already visited || goal reached


          std::vector<HcGraphVertexType> new_path = std::vector<HcGraphVertexType>(path);
          new_path.push_back(*it);
          std::unordered_map<teb_local_planner::HcGraphVertexType, bool> new_map(path_map);
          new_map.emplace(*it, true);
          queue.push(std::make_pair(new_path, new_map));

        }
      }


      //delete[] visited_array;
      //visited_array = NULL; 
    }
    
  }

}


void VoronoiGraph::createGraph(const PoseSE2& start, const PoseSE2& goal, double dist_to_obst, double obstacle_heading_threshold, const geometry_msgs::Twist* start_velocity)
{
  // Clear existing graph and paths
  clearGraph();
  if((int)hcp_->getTrajectoryContainer().size() >= cfg_->hcp.max_number_classes)
    return;
  // Direction-vector between start and goal and normal-vector:
  Eigen::Vector2d diff = goal.position()-start.position();
  double start_goal_dist = diff.norm();

  if (start_goal_dist<cfg_->goal_tolerance.xy_goal_tolerance)
  {
    ROS_DEBUG("HomotopyClassPlanner::createVoronoiGraph(): xy-goal-tolerance already reached.");
    if (hcp_->getTrajectoryContainer().empty())
    {
      ROS_INFO("HomotopyClassPlanner::createVoronoiGraph(): Initializing a small straight line to just correct orientation errors.");
      hcp_->addAndInitNewTeb(start, goal, start_velocity);
    }
    return;
  }


  hcp_->getVoronoi()->updateGridFromCostmap(); // there is internal checking whether a costmap is set
  

  for (auto it = hcp_->obstacles()->begin(); it != hcp_->obstacles()->end(); it++)
  {
    Eigen::Vector2d pos = it->get()->getCentroid();

    if (boost::dynamic_pointer_cast<PointObstacle>(*it) != nullptr)
    {
      hcp_->getVoronoi()->addCircularObstacle(pos.x(), pos.y(), cfg_->obstacles.min_obstacle_dist);
    }
    else if (boost::dynamic_pointer_cast<CircularObstacle>(*it) != nullptr)
    {
      hcp_->getVoronoi()->addCircularObstacle(pos.x(), pos.y(), boost::dynamic_pointer_cast<CircularObstacle>(*it)->radius());
    }
    else if (boost::dynamic_pointer_cast<LineObstacle>(*it) != nullptr)
    {
      boost::shared_ptr<LineObstacle> ptr = boost::dynamic_pointer_cast<LineObstacle>(*it);
      hcp_->getVoronoi()->addPillObstacle(ptr->start(), ptr->end(), cfg_->obstacles.min_obstacle_dist);
    }
    else if (boost::dynamic_pointer_cast<PillObstacle>(*it) != nullptr)
    {
      boost::shared_ptr<PillObstacle> ptr = boost::dynamic_pointer_cast<PillObstacle>(*it);
      hcp_->getVoronoi()->addPillObstacle(ptr->start(), ptr->end(), ptr->radius());
    }
    else if (boost::dynamic_pointer_cast<PolygonObstacle>(*it) != nullptr)
    {
      hcp_->getVoronoi()->addPolygonObstacle(boost::dynamic_pointer_cast<PolygonObstacle>(*it)->vertices());
    }
  
  }

  // robot footprint
  //hcp_->getVoronoi()->addCircularObstacle(start.position().x(), start.position().y(), std::max(hcp_->getInscribedRadius(), cfg_->obstacles.min_obstacle_dist)); 

  hcp_->getVoronoi()->updateGraph();

  
  boost::optional<std::pair<teb_local_planner::HcGraphVertexType, teb_local_planner::HcGraphVertexType>> start_goal
    = hcp_->getVoronoi()->addStartAndGoal(start.position(), goal.position()); 

  
    if (start_goal)
    {
      HcGraphVertexType start_vtx, goal_vtx;
      start_vtx = (*start_goal).first;
      goal_vtx = (*start_goal).second;
      /// Find all paths between start and goal!
      std::vector<HcGraphVertexType> visited;
      visited.push_back(start_vtx);

      // not neccessary to lock due to single threading, just in case someone creates a separate thread outside
      boost::mutex::scoped_lock l(*hcp_->getVoronoi()->getMutex());

      graph_ = *hcp_->getVoronoi()->getGraph();

      std::unordered_map<teb_local_planner::HcGraphVertexType, bool> visited_map;
      visited_map.emplace(start_vtx, true);

      //DepthFirst( graph_, visited, visited_array, goal_vtx, start.theta(), goal.theta(), start_velocity);

      // Breadth First search seems better 
      std::queue<std::pair<std::vector<HcGraphVertexType>, std::unordered_map<teb_local_planner::HcGraphVertexType, bool>>> queue;
      queue.push(std::make_pair(visited,visited_map));

      int count = 0;
      while (!queue.empty() && count < 2000)
      {
        count++;
        // get the first path from the queue
        std::vector<HcGraphVertexType> path = queue.front().first;
        std::unordered_map<teb_local_planner::HcGraphVertexType, bool> path_map = queue.front().second;
        queue.pop();

        HcGraphVertexType back = path.back();

        if ((int)hcp_->getTrajectoryContainer().size() >= cfg_->hcp.max_number_classes)
          break; // We do not need to search for further possible alternative homotopy classes.

        HcGraphAdjecencyIterator it, end;
        for ( boost::tie(it,end) = boost::adjacent_vertices(back, graph_); it!=end; ++it)
        {

          if (*it == goal_vtx)
          {
            std::vector<HcGraphVertexType> new_path = std::vector<HcGraphVertexType>(path);
            new_path.push_back(*it);

            double dist = 0;
            auto prev_it = new_path.begin();
            // the path should contains at least 2 elements, start & goal
            for (auto path_it = std::next(prev_it); path_it != new_path.end(); path_it++)
            {
              dist += (graph_[*path_it].pos - graph_[*prev_it].pos).norm();
              prev_it = path_it;
            }

            if (dist < cfg_->hcp.max_ratio_detours_duration_best_duration * cfg_->trajectory.max_global_plan_lookahead_dist)
            {
              // Add new TEB, if this path belongs to a new homotopy class
              hcp_->addAndInitNewTeb(new_path.begin(), new_path.end(), boost::bind(getVector2dFromHcGraph, _1, boost::cref(graph_)),
                                    start.theta(), goal.theta(), start_velocity);
            }

            continue;
          } 

          if ( path_map.find(*it) != path_map.end() )
            continue; // already visited || goal reached


          std::vector<HcGraphVertexType> new_path = std::vector<HcGraphVertexType>(path);
          new_path.push_back(*it);
          std::unordered_map<teb_local_planner::HcGraphVertexType, bool> new_map(path_map);
          new_map.emplace(*it, true);
          queue.push(std::make_pair(new_path, new_map));

        }
      }


      //delete[] visited_array;
      //visited_array = NULL; 
    }
  


}

} // end namespace
