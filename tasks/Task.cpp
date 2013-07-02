/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

#include <nav_graph_search/terrain_classes.hpp>
#include <envire/Orocos.hpp>
#include <envire/maps/MLSGrid.hpp>
#include <iterator>

using namespace simple_path_planner;

Task::Task(std::string const& name)
    : TaskBase(name),
    mMlsGrid(NULL),
    mPlanner(NULL),
    mStartPos(Eigen::Vector3d::Zero()),
    mGoalPos(Eigen::Vector3d::Zero()),
    mLastStartPosition(Eigen::Vector3d::Zero())
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine),
    mMlsGrid(NULL),
    mPlanner(NULL),
    mStartPos(Eigen::Vector3d::Zero()),
    mGoalPos(Eigen::Vector3d::Zero()),
    mLastStartPosition(Eigen::Vector3d::Zero())
{
}

Task::~Task()
{
    delete mPlanner;
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
    } catch (std::runtime_error& e) {
        RTT::log(RTT::Warning) << e.what() << ", default terrain classes will be used instead" << RTT::endlog();
    }

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

    RTT::log(RTT::Info) << "Received map" << RTT::endlog();

    std::vector<envire::TraversabilityGrid*> maps = mEnv->getItems<envire::TraversabilityGrid>();
    for(std::vector<envire::TraversabilityGrid*>::iterator it = maps.begin(); it != maps.end(); it++)
    {
        RTT::log(RTT::Info) << "Map id is " << (*it)->getUniqueId() << RTT::endlog();
    }


    // Extract traversability map from evironment.
    envire::TraversabilityGrid *traversability =
            mEnv->getItem< envire::TraversabilityGrid >(_traversability_map_id.get()).get();

    if (!traversability)
    {
        RTT::log(RTT::Warning) << "No traversability map with ID " << 
                _traversability_map_id.get() << RTT::endlog();
        return mTraversabilityMapStatus;
    } else {
        RTT::log(RTT::Info) << "Traversability map with ID " << 
                _traversability_map_id.get() << " extracted" << RTT::endlog();

        mPlanner->updateTraversabilityMap(traversability, mTraversabilityGrid.get());

        if(mTraversabilityGrid)
        {
            envire::FrameNode *lastGridFrame = mTraversabilityGrid->getFrameNode();
            mEnv->detachFrameNode(mTraversabilityGrid.get(), lastGridFrame);
            mEnv->detachItem(mTraversabilityGrid.get());
            mEnv->detachItem(lastGridFrame);
        }
        
        mTraversabilityGrid = traversability;
        envire::FrameNode *gridFrame = mTraversabilityGrid->getFrameNode();
        mEnv->detachFrameNode(mTraversabilityGrid.get(), gridFrame);
        envire::EnvironmentItem::Ptr trGrid = mEnv->detachItem(mTraversabilityGrid.get());
        mTraversabilityGrid->setUniqueId("lastTrGrid");
        
        mEnv->attachItem(mTraversabilityGrid.get(), gridFrame);
    }

    try {
        boost::intrusive_ptr<envire::MLSGrid> newMlsGrid = mEnv->getItem< envire::MLSGrid >();
        if(newMlsGrid)
        {
            mMlsGrid = newMlsGrid;
        }
    } catch (std::exception e) {
    }

    //set from NoData to OldData. This variable
    //should only be used internaly in this function.
    mTraversabilityMapStatus = RTT::OldData;
    return RTT::NewData;
}

void Task::updateHook()
{
    TaskBase::updateHook();

    bool needsReplan = false;

    RTT::FlowStatus ret = receiveEnvireData();
    if (ret == RTT::NoData) {
        return;
    }

    if (ret == RTT::NewData)
    {
        std::cout << "Got Map" << std::endl;
        needsReplan = true;
    }

    ret = _start_position_in.read(mStartPos);
    if (ret == RTT::NoData) {
        return;
    }

    if (ret == RTT::NewData)
    {
        std::cout << "Got start Position " << mStartPos.transpose() << std::endl;
        needsReplan = true;
    }

    ret = _target_position_in.read(mGoalPos);
    if (ret == RTT::NoData) {
        return;
    }

    if (ret == RTT::NewData)
    {
        std::cout << "Got goal Position " << mGoalPos.transpose() << std::endl;
        needsReplan = true;
    }

    base::samples::RigidBodyState robotPose;
    if (_robot_pose_in.read(robotPose) == RTT::NewData)
    {
        mStartPos = robotPose.position;  
        // Recalculate the trajectory if the distance exceeds a threshold
        base::Vector3d p1 = robotPose.position;
        base::Vector3d p2 = mLastStartPosition;
        p1.z() = p2.z() = 0;

        double distance = (p1 -p2).norm();
        if(distance > _recalculate_trajectory_distance_threshold.get()) {  
            RTT::log(RTT::Info) << "Trajectory will be recalculated, distance to last position of recalculation (" <<
                    distance << ") exceeds threshold" << RTT::endlog();
            needsReplan = true;
        }
    }

    // Initiate replanning if the robot stucks 
    base::Time currentTime = base::Time::now();
    if((currentTime - mLastReplanTime).toMilliseconds() > _replan_timeout_ms.get()) {
        RTT::log(RTT::Info) << "Replanning initiated, robot did not change its position for " << _replan_timeout_ms.get() << " msec" << RTT::endlog();
        needsReplan = true;
    }    

    if(needsReplan) {
        RTT::log(RTT::Info) << "Planning" << RTT::endlog();

        size_t startX, startY, endX, endY;

        if(!mTraversabilityGrid->toGrid(mStartPos, startX, startY)) {
            throw std::runtime_error("Error start is not in map");
        }

        if(!mTraversabilityGrid->toGrid(mGoalPos, endX, endY)) {
            throw std::runtime_error("Error goal is not in map");
        }

        if(mPlanner->run(startX, startY, endX, endY))
        {
            std::vector<Eigen::Vector2i> trajectoryGrid = mPlanner->getLocalTrajectory();
            std::vector<envire::GridBase::Position> trajectoryMlsGrid;
                std::vector<base::Vector3d> trajectory;
            
            for(std::vector<Eigen::Vector2i>::iterator it = trajectoryGrid.begin(); it != trajectoryGrid.end(); it++)
            {
                const Eigen::Vector3d p = mTraversabilityGrid->fromGrid(it->x(), it->y());
                envire::GridBase::Position gridPos;
                if(mMlsGrid)
                {
                    if(mMlsGrid->toGrid(p, gridPos.x, gridPos.y)) {
                        trajectoryMlsGrid.push_back(gridPos);
                    } else {
                        std::cout << "Warning Trajectory is outside of MLSGrid this ist most probably a bug" << std::endl;    
                    }
                }
                trajectory.push_back(p);
            }

            std::stringstream oss;
            oss << "Calculated trajectory: " << std::endl;
            for(unsigned int i = 0; i < trajectory.size(); ++i) {
                oss << "(" << trajectory[i].transpose() << ") ";
            }
            oss << std::endl;
            RTT::log(RTT::Info) << oss.str() << RTT::endlog();
            std::cout << oss.str() << std::endl;
        
            if(mMlsGrid)            
            {
                std::vector<Eigen::Vector3d> pTrajectory = mMlsGrid->projectPointsOnSurface(mStartPos.z(), trajectoryMlsGrid, _trajectory_z_offset.get());
                trajectory.clear();
                for(std::vector<Eigen::Vector3d>::const_iterator it = pTrajectory.begin(); it != pTrajectory.end();it++)
                {
                    double x, y;
                    mMlsGrid->fromGrid(it->x(), it->y(), x, y);
                    trajectory.push_back(base::Vector3d(x,y, it->z()));
                }
            }
                
            _trajectory_out.write(trajectory);

            base::Trajectory base_trajectory;
            base_trajectory.speed = 0.06; // set m/s.
            base_trajectory.spline.interpolate(trajectory);

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
            RTT::log(RTT::Warning) << "Trajectory could not be calculated" << RTT::endlog();
        }

        RTT::log(RTT::Info) << "Planning Done" << RTT::endlog();
        mLastReplanTime = currentTime;
    }
}
