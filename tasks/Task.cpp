/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

#include <nav_graph_search/terrain_classes.hpp>
#include <envire/Orocos.hpp>
#include <envire/maps/MLSGrid.hpp>
#include <iterator>
#include <fstream>

using namespace simple_path_planner;

Task::Task(std::string const& name)
    : TaskBase(name),
    mMlsGrid(NULL),
    mPlanner(NULL),
    mStartPos(base::Vector3d::Zero()),
    mGoalPos(base::Vector3d::Zero()),
    mLastReplanTime(),
    mLastStartPosition(base::Vector3d::Zero()),
    mEnv(NULL),
    mFirstReceivedTravMap(NULL)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine),
    mMlsGrid(NULL),
    mPlanner(NULL),
    mStartPos(base::Vector3d::Zero()),
    mGoalPos(base::Vector3d::Zero()),
    mLastReplanTime(),
    mLastStartPosition(base::Vector3d::Zero()),
    mEnv(NULL),
    mFirstReceivedTravMap(NULL)
{
}

Task::~Task()
{
    delete mPlanner; mPlanner = NULL;
    delete mEnv; mEnv = NULL;
    delete mFirstReceivedTravMap; mFirstReceivedTravMap = NULL;
}

bool Task::configureHook()
{
    if (! TaskBase::configureHook()) {
        return false;
    }

    // Delete old mls grids.
    mMlsGrid.reset();
    
    // We did not get a map yet.
    mTraversabilityMapStatus = RTT::NoData;    
    mLastReplanTime = base::Time();

    // Try to load the terrain classes.
    std::list<nav_graph_search::TerrainClass> classList;
    try {
        classList = nav_graph_search::TerrainClass::load(_terrain_classes_path.get());
        if(classList.empty()) {
            throw std::runtime_error("terrain class list empty");
        }
    } catch (std::runtime_error& e) {
        RTT::log(RTT::Warning) << e.what() << ", default terrain classes will be used instead" << RTT::endlog();

        nav_graph_search::TerrainClass unknown;
        unknown.cost = 1.5;
        unknown.out = 0;
        unknown.name = "unknown";
        nav_graph_search::TerrainClass obstacle;
        obstacle.cost = -1;
        obstacle.out = 1;
        obstacle.name = "obstacle";

        classList.push_back(unknown);
        classList.push_back(obstacle);
        
        // Costs of 2 to 1.
        for(int i = 2; i < 13; i++)
        {
            nav_graph_search::TerrainClass c;
            c.cost = 2 - (i-2) / 10.0;  // 1 + i * 0.01 ;
            c.out = i;
            c.name = "slope";
            classList.push_back(c);
        }
    }
    
    RTT::log(RTT::Info) << nav_graph_search::TerrainClass::toString(classList) << RTT::endlog();
    
    delete mPlanner;
    mPlanner = new nav_graph_search::DStarLite(classList);
    
    mEnv = new envire::Environment();
    
    return true;
}

// bool Task::startHook()
// {
//     if (! TaskBase::startHook())
//         return false;
//     return true;
// }

void Task::updateHook()
{
    TaskBase::updateHook();

    bool needsReplan = false;
    

    // Receive map.
    RTT::FlowStatus ret = receiveEnvireData();
    if (ret == RTT::NoData) {
        RTT::log(RTT::Info) <<  "No envire data has been received yet, return" << RTT::endlog();
        return;
    }

    if (ret == RTT::NewData)
    {
        RTT::log(RTT::Info) <<  "Received new environment data" << RTT::endlog();
        needsReplan = true;
        
        // New map data received, send the current dstar lite trav map if create_debug_outputs is set to true.
        if(_create_debug_outputs.get()) {
            sendInternalDStarLiteMap();
        }
    }

    // Receive start position. Prioritizes robot pose.
    base::samples::RigidBodyState robotPose;
    if(_robot_pose_in.connected()) {
        if (_robot_pose_in.read(robotPose) == RTT::NewData)
        {
            mStartPos = robotPose.position;  
            // Recalculate the trajectory if the distance exceeds a threshold (if distance > 0)
            if(_recalculate_trajectory_distance_threshold.get() > 0) {
                base::Vector3d p1 = robotPose.position;
                base::Vector3d p2 = mLastStartPosition;
                p1.z() = p2.z() = 0;

                double distance = (p1 -p2).norm();
                if(distance > _recalculate_trajectory_distance_threshold.get() && 
                        _replanning_on_new_start_position.get()) {  
                    RTT::log(RTT::Info) << "Trajectory will be recalculated, distance to last position of recalculation (" <<
                            distance << ") exceeds threshold" << RTT::endlog();
                    needsReplan = true;
                }
            }
        }
    }
    else 
    {
        ret = _start_position_in.read(mStartPos);
        if (ret == RTT::NoData) {
            RTT::log(RTT::Info) <<  "Received no start position, return" << RTT::endlog();
            return;
        }

        if (ret == RTT::NewData  && _replanning_on_new_start_position.get())
        {
            RTT::log(RTT::Info) <<  "Received start position: " << mStartPos << RTT::endlog();
            needsReplan = true;
        }
    }

    // Receive goal position.
    ret = _target_position_in.read(mGoalPos);
    if (ret == RTT::NoData) {
        RTT::log(RTT::Info) <<  "Received no goal position yet" << RTT::endlog();
        return;
    }

    if (ret == RTT::NewData)
    {
        RTT::log(RTT::Info) <<  "Received goal position: " << mGoalPos << RTT::endlog();
        if(_replanning_on_new_goal_position.get()) {
            needsReplan = true;
        }
    }

    // Initiate replanning if the robot stucks 
    base::Time currentTime = base::Time::now();
    if(_replan_timeout_ms.get() > 0 && 
            (currentTime - mLastReplanTime).toMilliseconds() > _replan_timeout_ms.get()) {
        RTT::log(RTT::Info) << "Replanning initiated, robot did not change its position for " << _replan_timeout_ms.get() << " msec" << RTT::endlog();
        needsReplan = true;
    }    

    if(needsReplan) {
        
        _debug_start_pos.write(mStartPos);
        _debug_goal_pos.write(mGoalPos);
        
        RTT::log(RTT::Info) << "Planning" << RTT::endlog();
        // Check whether the last map update has placed an obstacle on the goal position.
        double goal_cost = 1.0;
        if(mPlanner->getCost(mGoalPos[0], mGoalPos[1], goal_cost) && goal_cost == -1) {
            RTT::log(RTT::Warning) << "An obstacle has been placed on the goal position (" <<
                    mGoalPos[0] << ", " << mGoalPos[1] << ")" << RTT::endlog(); 
            exception(OBSTACLE_SET_ON_GOAL);  
        }

        if(mPlanner->run(mStartPos, mGoalPos, &mPlanningError))
        {
            std::vector<base::Vector3d> trajectory_map = mPlanner->getTrajectoryMap();
            std::vector<envire::GridBase::Position> trajectory_mls_grid;
            envire::GridBase::Position grid_pos;
            
            // Creates a MLS grid trajectory.
            if(mMlsGrid) {
                std::vector<base::Vector3d>::iterator it_map = trajectory_map.begin();
                for(; it_map != trajectory_map.end(); ++it_map) {
                    if(mMlsGrid->toGrid(*it_map, grid_pos.x, grid_pos.y)) {
                        trajectory_mls_grid.push_back(grid_pos);
                    } else {
                        RTT::log(RTT::Warning) << "Trajectory is outside of the MLSGrid which is most probably a bug" << RTT::endlog();    
                    }
                }
   
                // Adds the height to the MLS grid trajectory and refills trajectory map.
                std::vector<Eigen::Vector3d> p_trajectory = 
                        mMlsGrid->projectPointsOnSurface(mStartPos.z(), 
                                trajectory_mls_grid, 
                                _trajectory_z_offset.get());
                trajectory_map.clear();
                
                double x = 0.0, y = 0.0;
                std::vector<Eigen::Vector3d>::const_iterator it = p_trajectory.begin();
                for(; it != p_trajectory.end(); ++it)
                {
                    mMlsGrid->fromGrid(it->x(), it->y(), x, y);
                    trajectory_map.push_back(base::Vector3d(x,y, it->z()));
                }
            }
                
            _trajectory_out.write(trajectory_map);
            
            std::stringstream oss;
            oss << "Calculated trajectory: " << std::endl;
            for(unsigned int i = 0; i < trajectory_map.size(); ++i) {
                oss << "(" << trajectory_map[i][0] << ", " << 
                        trajectory_map[i][1] << ", " <<  
                        trajectory_map[i][2] << ") ";
            }
            RTT::log(RTT::Info) << oss.str() << RTT::endlog();

            base::Trajectory base_trajectory;
            base_trajectory.speed = 0.06; // set m/s.
            base_trajectory.spline.interpolate(trajectory_map);

            // Stuff it in a vector (it's possible to send several trajectories
            // which would be completed consecutively)
            std::vector<base::Trajectory> base_trajectory_vector;
            base_trajectory_vector.push_back(base_trajectory);
            _trajectory_spline_out.write(base_trajectory_vector);

            // Store the recalculated-trajectory-position.
            mLastStartPosition = mStartPos;
        }
        else
        {
            RTT::log(RTT::Warning) << "Trajectory could not be calculated, error " << 
                    mPlanningError << " has been returned" << RTT::endlog();
                    
            //write empty trajectopry to stop robot
            _trajectory_spline_out.write(std::vector<base::Trajectory>());
                    
            switch (mPlanningError) {
                case nav_graph_search::DStarLite::GOAL_SET_ON_OBSTACLE: exception(GOAL_SET_ON_OBSTACLE); break;
                case nav_graph_search::DStarLite::OBSTACLE_SET_ON_GOAL: exception(OBSTACLE_SET_ON_GOAL); break;
                case nav_graph_search::DStarLite::NO_PATH_TO_GOAL: exception(NO_PATH_TO_GOAL); break;
                default: break;
            }
        }
        RTT::log(RTT::Info) << "Planning Done" << RTT::endlog();
        mLastReplanTime = currentTime;
    }
}

void Task::stopHook() {
    std::string path = _statistics_path.get();
    if(!path.empty()){
        RTT::log(RTT::Info) << "Writing path planning statistics to " << path << RTT::endlog();
        std::ofstream stat_file;
        stat_file.open(path.c_str());
        std::string stat_str = mPlanner->getStatistics().toString();
        stat_file << stat_str;
        RTT::log(RTT::Info) << stat_str << RTT::endlog();
        stat_file.close();
    }
}

// PRIVATE

RTT::FlowStatus Task::receiveEnvireData()
{
    envire::OrocosEmitter::Ptr binary_event;
    RTT::FlowStatus ret = mTraversabilityMapStatus;
    while(_envire_environment_in.read(binary_event) == RTT::NewData)
    {
        ret = RTT::NewData;
        mEnv->applyEvents(*binary_event);   
    }

    if ((ret == RTT::NoData) || (ret == RTT::OldData))
    {
        return ret;
    }
    
    // Just to add height informations to the trajectory.
    extractMLS();
    
    // Extracts data and adds it to the planner. 
    if(!extractTraversability()) {
        return mTraversabilityMapStatus;
    }

    // Set from NoData to OldData, variable should only be used locally.
    mTraversabilityMapStatus = RTT::OldData;
    return RTT::NewData;
}

bool Task::extractTraversability() {
    std::vector<envire::TraversabilityGrid*> maps = mEnv->getItems<envire::TraversabilityGrid>();
    
    mEnv->serialize("/tmp/env_tmp");
    
    // Lists all received traversability maps.
    std::stringstream ss;
    if(maps.size()) {
        ss << "Received traversability map(s): " << std::endl;
 
        std::string trav_map_id;
        std::vector<envire::TraversabilityGrid*>::iterator it = maps.begin();
        for(int i=0; it != maps.end(); ++it, ++i)
        {
            ss << i << ": " << (*it)->getUniqueId() << std::endl;
        }
        RTT::log(RTT::Info) << ss.str() << RTT::endlog(); 
    } else {
        RTT::log(RTT::Warning) << "Environment does not contain any traversability grids" << RTT::endlog();
        return false;
    }

    // Extract traversability map from evironment.
    envire::TraversabilityGrid* traversability =
            mEnv->getItem< envire::TraversabilityGrid >(_traversability_map_id.get()).get();
    if (!traversability)
    {
        RTT::log(RTT::Info) << "No traversability map with id" << _traversability_map_id.get() << RTT::endlog();
        if(maps.size() > 1) {
            RTT::log(RTT::Warning) << "The environment contains more than one traversability map, please specify the map ID" << RTT::endlog();
            return false;
        } else {
            RTT::log(RTT::Info) << "The first traversability map will be used" << RTT::endlog();
            std::vector<envire::TraversabilityGrid*>::iterator it = maps.begin();
            traversability = mEnv->getItem< envire::TraversabilityGrid >((*it)->getUniqueId()).get();
            if (!traversability)
            {
                RTT::log(RTT::Warning) << "Traversability map '" << (*it)->getUniqueId() << 
                        "' could not be extracted" << RTT::endlog();
                return false;
            } 
        }
    } 
    
    RTT::log(RTT::Info) << "Traversability map " << traversability->getUniqueId() << " extracted" << RTT::endlog();
    
    // Adds the trav map to the planner.
    mPlanner->updateTraversabilityMap(traversability);

    return true;
}

bool Task::extractMLS() {
    std::vector<envire::MLSGrid*> mls_maps = mEnv->getItems<envire::MLSGrid>();
    if(mls_maps.size()) {
        std::stringstream ss;
        ss << "Received MLS map(s): " << std::endl;
        std::vector<envire::MLSGrid*>::iterator it = mls_maps.begin();
        for(int i=0; it != mls_maps.end(); ++it, ++i)
        {
            ss << i << ": "<< (*it)->getUniqueId() << std::endl;
        }
        RTT::log(RTT::Info) << ss.str() << RTT::endlog();
    } else {
        RTT::log(RTT::Info) << "Environment does not contain any MLS grids" << RTT::endlog();
        return false;
    }
    
    // Extracts the first MLS of the environment.
    try {
        boost::intrusive_ptr<envire::MLSGrid> new_mls_grid = mEnv->getItem< envire::MLSGrid >();    
        if(new_mls_grid)
        {
            mMlsGrid = new_mls_grid;
            RTT::log(RTT::Info) << "MLS Grid " <<  new_mls_grid->getUniqueId() << 
                    " extracted" << RTT::endlog();
        }
    } catch (std::exception e) {
        RTT::log(RTT::Warning) << "MLS Grid could not be extracted: " << 
            e.what() << RTT::endlog();
        return false;
    }
    return true;
}

void Task::sendInternalDStarLiteMap() {
    RTT::log(RTT::Info) <<  "Send internal dstar lite trav map" << RTT::endlog();
    envire::Environment env_tmp;
    // Copy first received trav map.
    envire::TraversabilityGrid* trav_grid_tmp = new envire::TraversabilityGrid(*(mPlanner->getRootTravMap()));
    env_tmp.attachItem(trav_grid_tmp);
    envire::FrameNode* p_fn = new envire::FrameNode();
    env_tmp.getRootNode()->addChild(p_fn);
    trav_grid_tmp->setFrameNode(p_fn);

    int terrain_class = 0;
    envire::TraversabilityGrid::ArrayType& trav_array = trav_grid_tmp->getGridData();
    for(size_t x=0; x < trav_grid_tmp->getCellSizeX(); ++x) {
        for(size_t y=0; y < trav_grid_tmp->getCellSizeY(); ++y) {
            if(mPlanner->getTerrainClass(x,y,terrain_class)) {
                // add to trav map
                trav_array[y][x] = terrain_class;
            }
        }
    } 
    envire::OrocosEmitter emitter_tmp(&env_tmp, _debug_internal_trav_map);
    emitter_tmp.setTime(base::Time::now());
    emitter_tmp.flush();
}
