/**
 * Copyright (c) 2018, G.A. vd. Hoorn
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @author G.A. vd. Hoorn
 */

#ifndef MOVEIT_MPP_PLANNER_MANAGER_H_
#define MOVEIT_MPP_PLANNER_MANAGER_H_

#include <moveit/planning_interface/planning_interface.h>
#include <ros/node_handle.h>
#include <pluginlib/class_loader.h>

#include <map>
#include <string>
#include <vector>

namespace moveit_mpp
{

class MultiPlannerPluginManager : public planning_interface::PlannerManager
{
public:
  MultiPlannerPluginManager();
  virtual ~MultiPlannerPluginManager();

  bool initialize(const robot_model::RobotModelConstPtr &model, const std::string &ns) override;

  bool canServiceRequest(const moveit_msgs::MotionPlanRequest &req) const override;

  std::string getDescription() const override
  {
    return "Multi Planner Plugin";
  }

  void getPlanningAlgorithms(std::vector<std::string> &algs) const override;

  planning_interface::PlanningContextPtr getPlanningContext(
      const planning_scene::PlanningSceneConstPtr &planning_scene,
      const planning_interface::MotionPlanRequest &req,
      moveit_msgs::MoveItErrorCodes &error_code) const override;

protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::map<std::string, planning_interface::PlannerManagerPtr> planners_;
  boost::shared_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager> > planner_plugin_loader_;
};

}

#endif /* MOVEIT_MPP_PLANNER_MANAGER_H_ */
