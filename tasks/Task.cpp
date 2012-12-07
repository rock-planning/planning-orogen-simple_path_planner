/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

#include <nav_graph_search/terrain_classes.hpp>
#include <envire/Orocos.hpp>
#include <envire/maps/MLSGrid.hpp>

using namespace simple_path_planner;

Task::Task(std::string const& name)
    : TaskBase(name),
    mInitialized(false),
    terrain_classes(),
    mpNavGraphTravMap(NULL),
    mpMLSHeights(NULL),
    mPlanner(NULL),
    mReceivedStartPos(false),
    mReceivedGoalPos(false),
    mNumOfUpdatedPatches(0),
    mRobotPose(),
    mPosLastRecalculation(0,0,0)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine),
    mInitialized(false),
    terrain_classes(),
    mpNavGraphTravMap(NULL),
    mpMLSHeights(NULL),
    mPlanner(NULL),
    mReceivedStartPos(false),
    mReceivedGoalPos(false),
    mNumOfUpdatedPatches(0),
    mRobotPose(),
    mPosLastRecalculation(0,0,0)
{
}

Task::~Task()
{
    if(mPlanner != NULL) {
        delete mPlanner; mPlanner = NULL;
    }
    if(mpMLSHeights != NULL) {
        free(mpMLSHeights);
        mpMLSHeights = NULL;
    }
}

bool Task::init() {
    if(mInitialized)
        return true;

    envire::OrocosEmitter::Ptr binary_event;
    if (!_envire_environment_in.read(binary_event))
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
    } else {
        RTT::log(RTT::Info) << "Traversability map with ID " << 
                _traversability_map_id.get() << " extracted" << RTT::endlog();
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
        /*
        for(int i=1; i<13; i++) {
            terrain_class.in = terrain_class.out = i;
            terrain_class.cost = ((i-1) * (i-1) ) / 12.0;//(i-1) / 12.0; 
            terrain_classes.push_back(terrain_class);
        }
        terrain_class.in = terrain_class.out = 0; // Unknown.
        terrain_class.cost = (5 * 5) / 12.0; //3.5 / 12;
        terrain_classes.push_back(terrain_class);
        */
        terrain_class.in = terrain_class.out = 0;
        terrain_class.cost = 1000;
        terrain_classes.push_back(terrain_class);
        for(int i=1; i<5; i++) {
            terrain_class.in = terrain_class.out = i;
            terrain_class.cost = 0;
            terrain_classes.push_back(terrain_class);
        }
        for(int i=5; i<13; i++) {
            terrain_class.in = terrain_class.out = i;
            terrain_class.cost = 1000;
            terrain_classes.push_back(terrain_class);
        }
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

    // Create array to store heights.
    if(mpMLSHeights != NULL) {
        free(mpMLSHeights);
        mpMLSHeights = NULL;
    }
    mpMLSHeights = (double*)calloc(mpNavGraphTravMap->xSize() * mpNavGraphTravMap->ySize(), sizeof(double));

    // Create SimplePathPlanner object.
    mPlanner = new SimplePathPlanner(*mpNavGraphTravMap, terrain_classes, 
            _robot_footprint_size.get(), _inflate_max.get());
   
    return true;
}

double Task::getHeightMLS(size_t xi, size_t yi) {
    if(mpMLSHeights == NULL) {
        RTT::log(RTT::Warning) << "No memory allocated to store the height of the patches" << RTT::endlog();
        return 0;
    }
    if(xi < (size_t)mpNavGraphTravMap->xSize() && yi < (size_t)mpNavGraphTravMap->ySize()) {
        return mpMLSHeights[xi + yi * mpNavGraphTravMap->ySize()];
    } else {
        RTT::log(RTT::Warning) << "Height of cell (" << xi << ", " << yi <<
                ") could not be requested, out of grid" << RTT::endlog();
        return 0;
    }
}

bool Task::setHeightMLS(size_t xi, size_t yi, double height) {
    if(xi < mpNavGraphTravMap->xSize() && yi <  mpNavGraphTravMap->ySize()) {
        mpMLSHeights[xi + yi * mpNavGraphTravMap->ySize()] = height;
        return true;
    }
    RTT::log(RTT::Info) << "(" << xi << ", " << yi << ") is not located within the grid" << RTT::endlog();
    return false;
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

    if (_robot_pose_in.read(mRobotPose) == RTT::NewData)
    {
        // Recalculate the trajectory if the distance exceeds a threshold
        base::Vector3d p1 = mRobotPose.position;
        base::Vector3d p2 = mPosLastRecalculation;
        double distance = sqrt(pow(p1[0] - p2[0],2) + pow(p1[1] - p2[1],2) + pow(p1[2] - p2[2],2));
        if(distance > _recalculate_trajectory_distance_threshold.get()) {
            mPlanner->setStartPositionWorld(mRobotPose.position);
            mReceivedStartPos = received_new_positions = true;
            RTT::log(RTT::Info) << "Trajectory will be recalculated, distance to last position of recalculation (" <<
                    distance << ") exceeds threshold" << RTT::endlog();
        }
    }

    pointcloud_creator::GridUpdate grid_update;
    if (_traversability_update_in.read(grid_update) == RTT::NewData)
    {
        base::Vector3d vec;
        for(unsigned int i=0; i < grid_update.mUpdatedPatches.size() && 
                i < grid_update.mUpdatedClasses.size(); ++i) {
            vec = grid_update.mUpdatedPatches[i];
            int class_ = grid_update.mUpdatedClasses[i];
            mPlanner->updateCellEnvire(vec[0], vec[1], (uint8_t)class_);
            //Update height to be able to generate 3D trajectories.
            setHeightMLS((size_t)vec[0], (size_t)vec[1], vec[2]);
            //mpMLSHeights[vec[0] + vec[1] * mpNavGraphTravMap->ySize()] = vec[2];
        }
        mNumOfUpdatedPatches += grid_update.mUpdatedPatches.size();
        double updated_per = (mNumOfUpdatedPatches * 100) / (mpNavGraphTravMap->xSize() * mpNavGraphTravMap->ySize() * 1.0);
        if( updated_per > _recalculate_trajectory_patch_threshold.get() ) {
            RTT::log(RTT::Info) << "Trajectory will be recalculated, " << updated_per 
                    << "% of the patches have been updated" << RTT::endlog();
            received_trav_update = true;
            mNumOfUpdatedPatches = 0;
        }
    }

    if(mReceivedStartPos && mReceivedGoalPos) {
        if(received_new_positions || received_trav_update) {
            //mPlanner->printInformations();
            if(!mPlanner->calculateTrajectory()) {
                RTT::log(RTT::Warning) << "Trajectory could not be calculated" << RTT::endlog();
            } else {
                std::vector<base::Vector3d> trajectory = mPlanner->getTrajectory();
                // Add z values if available, otherwise 0.
                size_t xi = 0, yi =0;
                base::Vector3d v;
                std::vector<base::Vector3d>::iterator it = trajectory.begin();
                for(; it != trajectory.end(); ++it) {
                    v = *it;
                    // Convert trajectory waypoint from world to local again.
                    if(mPlanner->toLocal(v[0], v[1], xi, yi)) {
                        (*it)[2] = getHeightMLS(xi, yi) + 0.1; // Assign height to the vector + 0.1
                        RTT::log(RTT::Debug) << "Assign height " << (*it)[2] << " to point (" <<
                            v[0] << ", " << v[1] << ")" <<  RTT::endlog(); 
                    } else {
                        RTT::log(RTT::Warning) << "Trajectoty contains out-of-grid point (" <<
                            v[0] << ", " << v[1] << "), patch is (" << xi << ", " << yi << ")" <<  RTT::endlog(); 
                    }
                }
                _trajectory_out.write(trajectory);

                std::stringstream oss;
                oss << "Calculated trajectory: " << std::endl;
                for(unsigned int i = 0; i < trajectory.size(); ++i) {
                    base::Vector3d p = trajectory[i];
                    oss << "(" << p[0] << ", " << p[1] << ", " << p[2] << ") ";
                }
                oss << std::endl;
                RTT::log(RTT::Info) << oss.str() << RTT::endlog();
    
                // Convert to a spline trajectory and output to port.
                std::vector<base::geometry::Spline<3>::vector_t> trajectory_spline;
                for(unsigned int i = 0; i < trajectory.size(); ++i) {
                    base::Vector3d p = trajectory[i];
                    base::geometry::Spline<3>::vector_t v_t(p[0], p[1], p[2]);
                    trajectory_spline.push_back(v_t);
                }

                base::Trajectory base_trajectory;
                base_trajectory.speed = 0.06; // set m/s.
                base_trajectory.spline.interpolate(trajectory_spline);

                // Stuff it in a vector (it's possible to send several trajectories
                // which would be completed consecutively)
                std::vector<base::Trajectory> base_trajectory_vector;
                base_trajectory_vector.push_back(base_trajectory);
                _trajectory_spline_out.write(base_trajectory_vector);

                // Store the recalculated-trajectory-position.
                mPosLastRecalculation = mRobotPose.position;
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

