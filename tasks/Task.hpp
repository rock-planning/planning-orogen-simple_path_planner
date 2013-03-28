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

    /*! \class Task 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * 
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','simple_path_planner::Task')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:
	///Optional mls grid for trajectory height adjustment
	boost::intrusive_ptr<envire::MLSGrid> mMlsGrid;
	
	///The path planner
//         SimplePathPlanner* mPlanner;
	
	nav_graph_search::DStarLite *mPlanner;
	
	///Current start position
        base::Vector3d mStartPos;
	///Current goal position
        base::Vector3d mGoalPos;

	/**
	 * Status of the traversability map
	 * is is either 
	 * old (no replan needed)
	 * new (replan needed)
	 * noData (no planning possible)
	 */
	RTT::FlowStatus mTraversabilityMapStatus;
	///the last received traversability grid
        envire::TraversabilityGrid::Ptr mTraversabilityGrid;
        envire::FrameNode::Ptr mTraversabilityGridFrame;
	envire::Environment *mEnv;
	
	///time of the last planning
	base::Time mLastReplanTime;
	///start position of last planning 
	base::Vector3d mLastStartPosition;

        /**
         * Does the following initializations:\n
         * 1. Tries to receive a envire::Environment on port 'traversability_map_in'.\n
         * 2. Extracts the envire traversability map.
         * 3. Loads or creates the terrain classes.
         * 4. Loads the trav. map into nav_graph_search.
         * 5. Creates the SimplePathPlanner object.
         */
        bool init();

	/**
	 * This function adjustes the Z values of the given trajectory,
	 * so that the trajectory is on the Surface of the MLS-Grid.
	 * */
	void adjustTrajectoryHeight(std::vector<base::Vector3d> &trajectory);
	
	/**
	 * This function receives the traversability grid
	 * and (if send) an mlsGrid.
	 * 
	 * It returns weather a new traversability grid is there / was received.
	 * */
	RTT::FlowStatus receiveEnvireData();
	
	void computeTraversabilityDiff(envire::Grid<uint8_t>* newGrid);
    public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "simple_path_planner::Task");

        /** TaskContext constructor for Task 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of Task
         */
	~Task();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
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
        // void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        // void cleanupHook();
    };
}

#endif

