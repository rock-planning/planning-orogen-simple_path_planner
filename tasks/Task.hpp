/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef SIMPLE_PATH_PLANNER_TASK_TASK_HPP
#define SIMPLE_PATH_PLANNER_TASK_TASK_HPP

#include "simple_path_planner/TaskBase.hpp"

#include <nav_graph_search/dstar_lite.hpp>

#include <sys/time.h>

namespace envire {
    class MLSGrid;
}

namespace simple_path_planner {

class Task : public TaskBase
{
    friend class TaskBase;
    
 protected:
    /** Optional mls grid for trajectory height adjustment. */
    boost::intrusive_ptr<envire::MLSGrid> mMlsGrid;

    nav_graph_search::DStarLite *mPlanner;

    /** Current start position */
    base::Vector3d mStartPos;
    /** Current goal position */
    base::Vector3d mGoalPos;

    /**
     * Status of the traversability map
     * is is either 
     * old (no replan needed)
     * new (replan needed)
     * noData (no planning possible)
     */
    RTT::FlowStatus mTraversabilityMapStatus;
    
    envire::Environment *mEnv;

    /** Time of the last planning. */
    base::Time mLastReplanTime;
    /** Start position of last planning. */
    base::Vector3d mLastStartPosition;
    
 public:
    /** 
     * TaskContext constructor for Task
     * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
     * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
     */
    Task(std::string const& name = "simple_path_planner::Task");

    /** 
     * TaskContext constructor for Task 
     * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
     * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
     * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
     */
    Task(std::string const& name, RTT::ExecutionEngine* engine);

    /** 
     * Default deconstructor of Task
     */
    ~Task();

    /** 
     * This hook is called by Orocos when the state machine transitions
     * from PreOperational to Stopped. If it returns false, then the
     * component will stay in PreOperational. Otherwise, it goes into
     * Stopped.
     *
     * It is meaningful only if the #needs_configuration has been specified
     * in the task context definition with (for example):
     */
    bool configureHook();

    /** 
     * This hook is called by Orocos when the state machine transitions
     * from Stopped to Running. If it returns false, then the component will
     * stay in Stopped. Otherwise, it goes into Running and updateHook()
     * will be called.
     */
    // bool startHook();

    /** This hook is called by Orocos when the component is in the Running
     * state, at each activity step. Here, the activity gives the "ticks"
     * when the hook should be called.
     *
     * The error(), exception() and fatal() calls, when called in this hook,
     * allow to get into the associated RunTimeError, Exception and
     * FatalError states. 
     *
     * In the first case, updateHook() is still called, and recover() allows
     * you to go back into the Running state.  In the second case, the
     * errorHook() will be called instead of updateHook(). In Exception, the
     * component is stopped and recover() needs to be called before starting
     * it again. Finally, FatalError cannot be recovered.
     */
    void updateHook();

    /** This hook is called by Orocos when the component is in the
     * RunTimeError state, at each activity step. See the discussion in
     * updateHook() about triggering options.
     *
     * Call recover() to go back in the Runtime state.
     */
    // void errorHook();

    /** This hook is called by Orocos when the state machine transitions
     * from Running to Stopped after stop() has been called.
     */
    void stopHook();

    /** This hook is called by Orocos when the state machine transitions
     * from Stopped to PreOperational, requiring the call to configureHook()
     * before calling start() again.
     */
    // void cleanupHook();
    
 private:
     /**
     * This function receives the traversability grid
     * and (if send) an mlsGrid.
     * 
     * Returns whether a new traversability grid is available / has been received.
     */
    RTT::FlowStatus receiveEnvireData();
    
    /**
     * Extracts the traversability map and adds the information to the planner.
     */
    bool extractTraversability();
    
    /**
     * Tries to extract the MLS from the environment to add height informations to the
     * trajectory.
     */
    bool extractMLS();
};
}

#endif

