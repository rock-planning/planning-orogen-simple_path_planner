/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

#include <nav_graph_search/terrain_classes.hpp>
#include <envire/Orocos.hpp>

using namespace simple_path_planner;

Task::Task(std::string const& name)
    : TaskBase(name),
    mInitialized(false),
    terrain_classes(),
    mpNavGraphTravMap(NULL),
    mPlanner(NULL),
    mReceivedStartPos(false),
    mReceivedGoalPos(false),
    mNumOfUpdatedPatches(0)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine),
    mInitialized(false),
    terrain_classes(),
    mpNavGraphTravMap(NULL),
    mPlanner(NULL),
    mReceivedStartPos(false),
    mReceivedGoalPos(false),
    mNumOfUpdatedPatches(0)
{
}

Task::~Task()
{
    if(mPlanner != NULL) {
        delete mPlanner; mPlanner = NULL;
    }
}

bool Task::init() {
    if(mInitialized)
        return true;

    envire::OrocosEmitter::Ptr binary_event;
    if (!_traversability_map_in.read(binary_event))
    {
        return false;
    }

    envire::Environment env;
    env.applyEvents(*binary_event);   

    // Extract traversability map from evironment.
    envire::Grid<uint8_t>* traversability =
            env.getItem< envire::Grid<uint8_t> >(_traversability_map_id.get()).get();

    if (!traversability)
    {
        RTT::log(RTT::Warning) << "No traversability map with ID " << 
                _traversability_map_id.get() << RTT::endlog();
        return false;
    }

    // Load terrain classes.
    terrain_classes.clear();
    try {
        terrain_classes = nav_graph_search::TerrainClass::load(_terrain_classes_path.get());
        if(terrain_classes.empty()) {
            throw std::runtime_error("No terrain classes loaded, file empty?");
        }
    } catch (std::runtime_error& e) {
        RTT::log(RTT::Warning) << "Loading terrain classes: " << e.what() << RTT::endlog();
        RTT::log(RTT::Info) << "Default terrain classes will be used" << RTT::endlog();

        struct nav_graph_search::TerrainClass terrain_class;
        // cost -> 0: map_scale / 0.29
        //         1: 1000000
        //         2: map_scale / 0.083 
        // to 
        //        12: map_scale / 0.83
        for(int i=1; i<13; i++) {
            terrain_class.in = terrain_class.out = i;
            terrain_class.cost = (i-1) / 12.0; 
            terrain_classes.push_back(terrain_class);
        }
        terrain_class.in = terrain_class.out = 0; // Unknown.
        terrain_class.cost = 3.5 / 12;
        terrain_classes.push_back(terrain_class);
    }
    
    // Import envire traversability map into nav_graph_search::TraversabilityMap
    try {
        mpNavGraphTravMap = nav_graph_search::TraversabilityMap::load(*traversability, 
                _traversability_map_band.get(), terrain_classes);
    } catch (std::runtime_error& e) {
        RTT::log(RTT::Error) << "Loading traversability map error: " << e.what() << RTT::endlog();
        return false;
    }
    if(mpNavGraphTravMap == NULL){
        RTT::log(RTT::Error) << "Unknown class found, check the terrain class definition file" << RTT::endlog();
        return false;
    }

    // Create SimplePathPlanner object.
    mPlanner = new SimplePathPlanner(*mpNavGraphTravMap, terrain_classes, 
            _robot_footprint_size.get(), _inflate_max.get());
   
    return true;
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook()) {
        return false;
    }
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

    bool received_new_positions = false;
    bool received_trav_update = false;

    if(!mInitialized) {
        if(init()) {
            mInitialized = true;    
            received_trav_update = true;
        } else {
            return;
        }
    }

    base::Vector3d pos;
    if (_start_position_in.read(pos) == RTT::NewData)
    {
        mPlanner->setStartPositionWorld(pos);
        mReceivedStartPos = received_new_positions = true;
    }
    if (_target_position_in.read(pos) == RTT::NewData)
    {
        mPlanner->setGoalPositionWorld(pos);
        mReceivedGoalPos = received_new_positions = true;
    }
    base::samples::RigidBodyState rbs;
    if (_robot_pose_in.read(rbs) == RTT::NewData)
    {
        mPlanner->setStartPositionWorld(rbs.position);
        mReceivedStartPos = received_new_positions = true;
    }

    pointcloud_creator::GridUpdate grid_update;
    if (_traversability_update_in.read(grid_update) == RTT::NewData) // while? == RTT::NewData?
    {
        base::Vector3d vec;
        for(unsigned int i=0; i < grid_update.mUpdatedPatches.size(); ++i) {
            vec = grid_update.mUpdatedPatches[i];
            mPlanner->updateCellEnvire(vec[0], vec[1], (uint8_t)vec[2]);
        }
        mNumOfUpdatedPatches += grid_update.mUpdatedPatches.size();
        double updated_per = (mNumOfUpdatedPatches * 100) / (mpNavGraphTravMap->xSize() * mpNavGraphTravMap->ySize() * 1.0);
        if( updated_per > _recalculate_trajectory_threshold.get() ) {
            RTT::log(RTT::Info) << "Trajectory will be recalculated, " << updated_per 
                    << "% of the patches have been updated" << RTT::endlog();
            received_trav_update = true;
            mNumOfUpdatedPatches = 0;
        }
    }

    if(mReceivedStartPos && mReceivedGoalPos) {
        if(received_new_positions || received_trav_update) {
            if(!mPlanner->calculateTrajectory()) {
                RTT::log(RTT::Warning) << "Trajectory could not be calculated" << RTT::endlog();
            } else {
                std::vector<base::Waypoint> trajectory = mPlanner->getTrajectory();
                _trajectory_out.write(trajectory );
                std::stringstream oss;
                oss << "Calculated trajectory: " << std::endl;
                for(unsigned int i = 0; i < trajectory.size(); ++i) {
                    base::Vector3d p = trajectory[i].position;
                    oss << "(" << p[0] << ", " << p[1] << ") ";
                }
                oss << std::endl;
                RTT::log(RTT::Info) << oss.str() << RTT::endlog();
            }
        }
    }
}

// void Task::errorHook()
// {
//     TaskBase::errorHook();
// }
// void Task::stopHook()
// {
//     TaskBase::stopHook();
// }
// void Task::cleanupHook()
// {
//     TaskBase::cleanupHook();
// }

