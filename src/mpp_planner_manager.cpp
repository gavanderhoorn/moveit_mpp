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

#include <moveit_mpp/mpp_planner_manager.h>
#include <class_loader/class_loader.h>


// NOTE: in reality, the 'req.planner_id' is not an actual planner id, but
// a ROS param name in the form: group[PLANNER_CONFIG_NAME].
// 
// Example:
//   manipulator[RRTConnectkConfigDefault]
//   
// this deconstructs into:
//   group                = manipulator
//   planner config param = RRTConnectkConfigDefault
// 
// to get to the actual planner class, we need to look at parameter:
// 
//   /move_group/planner_configs/RRTConnectkConfigDefault/type
// 
// which has the value: geometric::RRTConnect
// 
// which is the actual 'planner id'

// example from MotionPlanRequest
//   planner_id: Default PTP
//   group_name: manipulator

// example from MotionPlanRequest (with planner key prefix)
//   planner_id: ompl/SBLkConfigDefault
//   group_name: manipulator





namespace moveit_mpp
{


static const std::string PARAMETER_NS = "mpp";

static const std::string SVC_LIST_PLUGINS = "list_plugins";

static const std::string DEFAULT_PLANNER = "ompl";



struct PlannerStruct
{
  std::string key;
  std::string cfg;
};

// TODO: deal with 'malformed' planner id strings properly
PlannerStruct resolve_planner(const std::string& planner_str)
{
  PlannerStruct ret;

  // default result: default planner, no specific config. Planner plugin
  // should be able to deal with that. If it can't, it'll complain to caller.
  ret.key = DEFAULT_PLANNER;
  ret.cfg = "";

  if (planner_str == "")
  {
    ROS_INFO_STREAM_NAMED("multi_planner_plugin_manager",
      "No planner specified, falling back to default (" << ret.key << ")");
    return ret;
  }

  // 'ompl/PRMstarkConfigDefault'
  auto idx = planner_str.find('/');
  if (idx == std::string::npos)
  {
    // assume this is a bw-compatibility case
    ROS_INFO_STREAM_NAMED("multi_planner_plugin_manager",
      "No planner prefix found, assuming default (" << ret.key << ")");
    ret.cfg = planner_str;
  }
  else
  {
    // split into key, planner config
    ret.key = planner_str.substr(0, idx);
    ret.cfg = planner_str.substr(idx + 1);
    ROS_INFO_STREAM_NAMED("multi_planner_plugin_manager",
      "Request for planner cfg '" << ret.cfg << "' from planner '"
      << ret.key << "'");
  }

  return ret;
}








MultiPlannerPluginManager::MultiPlannerPluginManager() :
  planning_interface::PlannerManager(), pnh_(PARAMETER_NS)
{
  ROS_INFO_NAMED("multi_planner_plugin_manager", "ctor");

  planner_plugin_loader_.reset(
    new pluginlib::ClassLoader<planning_interface::PlannerManager>(
      "moveit_core", "planning_interface::PlannerManager"));
}

MultiPlannerPluginManager::~MultiPlannerPluginManager()
{
  ROS_INFO_NAMED("multi_planner_plugin_manager", "dtor");

  // TODO: should all planner plugins be explicitly dtor-ed?
}

bool MultiPlannerPluginManager::initialize(const robot_model::RobotModelConstPtr &model, const std::string &ns)
{
  ROS_INFO_STREAM_NAMED("multi_planner_plugin_manager", "initialise");
  ROS_INFO_STREAM_NAMED("multi_planner_plugin_manager", "ns: '" << ns << "'");

  if (!ns.empty())
  {
    nh_ = ros::NodeHandle(ns);
    pnh_ = ros::NodeHandle(ns, PARAMETER_NS);
  }


  // TODO: get our own parameters, for now, use hard-coded hack
  std::map<std::string, std::string> planners_to_load =
  {
    {"ompl", "ompl_interface/OMPLPlanner"},
    {"ptp" , "moveit_ptp/PtpPlannerManager"},
    // {"clik", "constrained_ik/CLIKPlanner"}
  };


  for (auto const& planner_info : planners_to_load)
  {
    ROS_INFO_STREAM_NAMED("multi_planner_plugin_manager", "Attempting to load planner plugin '"
      << planner_info.second << "' for '" << planner_info.first << "'");

    auto planner_ns = planner_info.first;
    auto planner_id = planner_info.second;

    try
    {
      auto p = planner_plugin_loader_->createInstance(planner_id);
      p->initialize(model, ns + "/" + planner_ns);
      planners_[planner_ns] = p;
    }
    catch (pluginlib::PluginlibException& ex)
    {
      ROS_ERROR_STREAM("Exception while loading planner plugin '" << planner_id
        << "': " << ex.what());
    }
  }

  // if no planners could be loaded, all lookups later will fail, so we don't
  // need to do anything special here
  ROS_INFO_STREAM_NAMED("multi_planner_plugin_manager", "Loaded "
    << planners_.size() << " planner plugins.");


  // TODO: init reverse lookup hashmap here, using 'getPlanningAlgorithms()' output
  // as keys, and the PlannerManagerPtr as value. This would let us make a reverse
  // lookup in canServiceRequest() and getPlanningContext() without having to
  // deconstruct the 'planner_id' in the MotionPlanRequest


  // done
  return true;
}

bool MultiPlannerPluginManager::canServiceRequest(const moveit_msgs::MotionPlanRequest &req) const
{
  ROS_INFO_STREAM_NAMED("multi_planner_plugin_manager",
    "canServiceRequest for planner '" << req.planner_id << "'");

  // figure out which planner should be used
  PlannerStruct resolved_planner = resolve_planner(req.planner_id);

  // duplicate request and update planner cfg id (remove planner prefix)
  planning_interface::MotionPlanRequest nreq(req);
  nreq.planner_id = resolved_planner.cfg;

  // retrieve planner instance
  // TODO: do not assume planner is loaded
  return planners_.at(resolved_planner.key)->canServiceRequest(nreq);
}

void MultiPlannerPluginManager::getPlanningAlgorithms(std::vector<std::string>& algs) const
{
  algs.clear();

  // loop over all loaded planners
  for (auto&& planner_info : planners_)
  {
    auto planner_key = planner_info.first;
    auto planner = planner_info.second;

    // retrieve configured planner configs from planner
    std::vector<std::string> temp;
    planner->getPlanningAlgorithms(temp);

    ROS_INFO_STREAM_NAMED("multi_planner_plugin_manager", "Adding "
      << temp.size() << " alg(s) for key '" << planner_key << "':");

    // prefix all returned planner configs with the key (ns) for this planner
    for (auto&& alg : temp)
    {
      // manipulator  <<-- this points to default: with MPP that means the
      //                   'default planner' (ie: yaml setting)
      auto split_idx = alg.find('[');
      if (split_idx == std::string::npos)
      {
        // assume this is the default settings:
        algs.push_back(alg);
      }
      else
      {
        // convert: manipulator[BKPIECEkConfigDefault]
        // to     : manipulator[ompl/BKPIECEkConfigDefault]
        auto ns_alg = alg.insert(split_idx + 1, planner_key + "/");
        algs.push_back(ns_alg);
        ROS_INFO_STREAM_NAMED("multi_planner_plugin_manager", "  " << ns_alg);
      }
    }
  }
}

planning_interface::PlanningContextPtr MultiPlannerPluginManager::getPlanningContext(
  const planning_scene::PlanningSceneConstPtr &planning_scene,
  const planning_interface::MotionPlanRequest &req,
  moveit_msgs::MoveItErrorCodes &error_code) const
{
  ROS_INFO_STREAM_NAMED("multi_planner_plugin_manager",
    "getPlanningContext for planner: '" << req.planner_id << "'");

  // figure out which planner should be used
  PlannerStruct resolved_planner = resolve_planner(req.planner_id);

  // duplicate request and update planner cfg id (remove planner prefix)
  planning_interface::MotionPlanRequest nreq(req);
  nreq.planner_id = resolved_planner.cfg;

  // pass this on to requested planner
  return planners_.at(resolved_planner.key)->getPlanningContext(planning_scene, nreq, error_code);
}


} // moveit_mpp


CLASS_LOADER_REGISTER_CLASS(moveit_mpp::MultiPlannerPluginManager, planning_interface::PlannerManager);
