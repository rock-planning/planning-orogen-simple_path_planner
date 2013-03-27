/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

#include <nav_graph_search/terrain_classes.hpp>
#include <envire/Orocos.hpp>
#include <envire/maps/MLSGrid.hpp>
#include <simple_path_planner/SimplePathPlanner.hpp>

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

    //delete old mld grids
    mMlsGrid.reset();
    
    //we did not get a map yet
    mTraversabilityMapStatus = RTT::NoData;    
    mLastReplanTime = base::Time();
    
    std::list<nav_graph_search::TerrainClass> classList;
    nav_graph_search::TerrainClass unknown;
    unknown.cost = 2;
    unknown.out = 0;
    unknown.name = "unknown";
    nav_graph_search::TerrainClass obstacle;
    obstacle.cost = -1;
    obstacle.out = 1;
    obstacle.name = "obstacle";

    classList.push_back(unknown);
    classList.push_back(obstacle);
    
    for(int i = 2; i < 255; i++)
    {
	nav_graph_search::TerrainClass c;
	c.cost = 1+ i * 0.01 ;
	c.out = i;
	c.name = "Custom";
	classList.push_back(c);
    }
    
    delete mPlanner;
    mPlanner = new nav_graph_search::DStarLite(classList);
    
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
    RTT::FlowStatus ret;
    ret = _envire_environment_in.read(binary_event);
    if ((ret == RTT::NoData) || (ret == RTT::OldData))
    {
        return ret;
    }

    std::cout << "GOT MAP" << std::endl;
    
    static envire::Environment env;
    env.applyEvents(*binary_event);   

    std::vector<envire::TraversabilityGrid*> maps = env.getItems<envire::TraversabilityGrid>();
    for(std::vector<envire::TraversabilityGrid*>::iterator it = maps.begin(); it != maps.end(); it++)
    {
	std::cout << "FOO Map id is " << (*it)->getUniqueId() <<std::endl;
    }

    
    // Extract traversability map from evironment.
    envire::TraversabilityGrid* traversability =
            env.getItem< envire::TraversabilityGrid >(_traversability_map_id.get()).get();

    if (!traversability)
    {
        RTT::log(RTT::Warning) << "No traversability map with ID " << 
                _traversability_map_id.get() << RTT::endlog();
        return mTraversabilityMapStatus;
    } else {
        RTT::log(RTT::Info) << "Traversability map with ID " << 
                _traversability_map_id.get() << " extracted" << RTT::endlog();
    }

    mPlanner->updateTraversabilityMap(traversability);
    
    mTraversabilityGrid = traversability;
    
    try {
	boost::intrusive_ptr<envire::MLSGrid> newMlsGrid = env.getItem< envire::MLSGrid >();
	if(newMlsGrid)
	{
	    mMlsGrid = newMlsGrid;
	}
    }
    catch (std::exception e)
    {
    }
    
    //set from NoData to OldData. this variable
    //should only be used internaly in this function.
    mTraversabilityMapStatus = RTT::OldData;
    return RTT::NewData;
}

void Task::adjustTrajectoryHeight(std::vector< base::Vector3d >& trajectory)
{
    if(!mMlsGrid)
	return;
    
    // Add z values if available, otherwise 0.
    double lastHeight = mStartPos.z();
    std::vector<base::Vector3d>::iterator it = trajectory.begin();
    for(; it != trajectory.end(); ++it) {
	envire::MLSGrid::const_iterator cIt = mMlsGrid->beginCell(it->x(), it->y());
	
	double minDiff = std::numeric_limits< double >::max();
	double closestZ = base::unset<double>();
	for(;cIt != mMlsGrid->endCell_const(); cIt++)
	{
	    double diff = fabs(lastHeight - cIt->getMaxZ());
	    if(diff < minDiff)
	    {
		minDiff = diff;
		closestZ = cIt->getMaxZ();
	    }
	}
	
	if(!base::isUnset<double>(closestZ))
	{
	    lastHeight = closestZ;
	    it->z() = closestZ;
	    RTT::log(RTT::Debug) << "Assign height " << closestZ << " to point (" <<
		it->x() << ", " << it->y() << ")" <<  RTT::endlog(); 
	} 
	else
	{
	    RTT::log(RTT::Warning) << "Trajectoty contains out-of-grid point (" <<
		it->x() << ", " << it->y() << ") " <<  RTT::endlog(); 		    
	}
    }
}



void Task::updateHook()
{
//     std::cout << "UPDATE " << std::endl;
    TaskBase::updateHook();

    bool needsReplan = false;
    
    RTT::FlowStatus ret = receiveEnvireData();
    if (ret == RTT::NoData)
	return;
    
    if (ret == RTT::NewData)
    {
	std::cout << "Got Map" << std::endl;

	needsReplan = true;
    }

    ret = _start_position_in.read(mStartPos);
    if (ret == RTT::NoData)
	return;
    
    if (ret == RTT::NewData)
    {
	std::cout << "Got start Position " << mStartPos.transpose() << std::endl;
	needsReplan = true;
    }

    ret = _target_position_in.read(mGoalPos);
    if (ret == RTT::NoData)
	return;
    
    if (ret == RTT::NewData)
    {
	std::cout << "Got goal Position " << mGoalPos.transpose() << std::endl;
	needsReplan = true;
//         mPlanner->setGoalPositionWorld(mGoalPos);
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

    
    
    
//     pointcloud_creator::GridUpdate grid_update;
//     if (_traversability_update_in.read(grid_update) == RTT::NewData)
//     {
//         base::Vector3d vec;
//         for(unsigned int i=0; i < grid_update.mUpdatedPatches.size() && 
//                 i < grid_update.mUpdatedClasses.size(); ++i) {
//             vec = grid_update.mUpdatedPatches[i];
//             int class_ = grid_update.mUpdatedClasses[i];
//             mPlanner->updateCellEnvire(vec[0], vec[1], (uint8_t)class_);
//             //Update height to be able to generate 3D trajectories.
//             setHeightMLS((size_t)vec[0], (size_t)vec[1], vec[2]);
//             //mpMLSHeights[vec[0] + vec[1] * mpNavGraphTravMap->ySize()] = vec[2];
//         }
//         mNumOfUpdatedPatches += grid_update.mUpdatedPatches.size();
//         double updated_per = (mNumOfUpdatedPatches * 100) / (mpNavGraphTravMap->xSize() * mpNavGraphTravMap->ySize() * 1.0);
//         if( updated_per > _recalculate_trajectory_patch_threshold.get() ) {
//             RTT::log(RTT::Info) << "Trajectory will be recalculated, " << updated_per 
//                     << "% of the patches have been updated" << RTT::endlog();
//             received_trav_update = true;
//             mNumOfUpdatedPatches = 0;
//         }
//     }
    
    if(needsReplan)
    {
	std::cout << "Planning" << std::endl;

// 	mPlanner->setStartPositionWorld(mStartPos);
    
	size_t startX, startY, endX, endY;

	if(!mTraversabilityGrid->toGrid(mStartPos, startX, startY))
	    throw std::runtime_error("Error start is not in map");

	if(!mTraversabilityGrid->toGrid(mGoalPos, endX, endY))
	    throw std::runtime_error("Error goal is not in map");

	if(mPlanner->run(startX, startY, endX, endY))
	{
	    
	    std::vector<Eigen::Vector2i> trajectoryGrid = mPlanner->getLocalTrajectory();
	    std::vector<base::Vector3d> trajectory;
	    
	    for(std::vector<Eigen::Vector2i>::iterator it = trajectoryGrid.begin(); it != trajectoryGrid.end(); it++)
	    {
		trajectory.push_back(mTraversabilityGrid->fromGrid(it->x(), it->y()));
	    }
	    
	    if(mMlsGrid)
		adjustTrajectoryHeight(trajectory);

	    _trajectory_out.write(trajectory);

	    std::stringstream oss;
	    oss << "Calculated trajectory: " << std::endl;
	    for(unsigned int i = 0; i < trajectory.size(); ++i) {
		oss << "(" << trajectory[i].transpose() << ") ";
	    }
	    oss << std::endl;
	    RTT::log(RTT::Info) << oss.str() << RTT::endlog();
	    std::cout << oss.str() << std::endl;

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
	    std::cout << "Trajectory could not be calculated" << std::endl;
	    RTT::log(RTT::Warning) << "Trajectory could not be calculated" << RTT::endlog();
	}
	
	std::cout << "Planning Done" << std::endl;
	mLastReplanTime = currentTime;
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

