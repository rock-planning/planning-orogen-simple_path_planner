require 'orocos'
require 'vizkit'
require 'readline'

Orocos::CORBA.max_message_size = 8000000 # stucks if > than this

include Orocos
Orocos.initialize

# Content of these language variables must not be german, otherwise '.' and ','
# are mixed reading numbers and a bad_alloc error occurs loading the scenes.
ENV['LANG'] = 'C'
ENV['LC_NUMERIC'] = 'C'

Orocos.run 'spacebot_simulation',
        'spacebot_motioncontroller::Task' => 'controller',
        'envire::SynchronizationTransmitter' => 'transmitter', 
        'corridor_planner::Traversability' => 'traversability',
        'simple_path_planner::Task' => 'planner',
        'trajectory_follower::Task' => 'follower',
        'visualizer_module::Task' => 'visualizer', 
        "valgrind" => false, "wait" => 1000 do
        
    Orocos.conf.load_dir('./config')

    # SIMULATION
    simulation = TaskContext.get 'mars_simulation'  
    simulation.apply_conf(['default'])
    simulation.configure
    simulation.start
    # Has to be called after configure.
    simulation.loadScene("#{ENV['AUTOPROJ_PROJECT_BASE']}/install/configuration/mars_scenes/spaceBot.scn")
    
    # ACTUATORS
    locomotion_actuators_names = ["rear_left", "rear_left_turn", "middle_left", "middle_left_turn", "front_right", "front_right_turn", 
            "front_left", "front_left_turn", "rear_right", "rear_right_turn", "middle_right", "middle_right_turn"]
    locomotion_actuators_indices = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12] # Disable actuator by using 0, invert by using a negative index. 
    locomotion_actuators = TaskContext.get 'mars_locomotion'
    locomotion_actuators.names = locomotion_actuators_names
    if not locomotion_actuators.dispatch("locomotion_actuators", locomotion_actuators_indices) # Has to be executed before configure
        puts "Locomotion actuators could not be dispatched (not available in the scene file?), exit"
        exit 1
    end
    locomotion_actuators.configure
    locomotion_actuators.start
    
    # IMU
    imu = TaskContext.get 'mars_imu'
    imu.apply_conf(['default'])
    imu.configure
    imu.start
    
    # LOAD ENV AND CREATE TRAV
    # What is 'Orocos.name_service.get' instead of 'TaskContext::get'
    transmitter = TaskContext::get 'transmitter'
    transmitter.configure()
    transmitter.start()
    
    traversability = Orocos.name_service.get 'traversability'
    traversability.apply_conf(['default'])
    traversability.configure()
    traversability.start()
    
    # PLANNER 
    planner = TaskContext::get 'planner'
    planner.apply_conf(['default'])
    planner.configure()
    planner.start()
    
    # VISUALIZER
    visualizer = TaskContext::get 'visualizer'
    visualizer.configure()
    visualizer.start()
    
    # CONTROLLER
    controller = TaskContext::get 'controller'
    controller.ackermann_ratio_p = 1.0
    controller.start
    
    # TRAJECTORY FOLLOWER
    follower = TaskContext::get 'follower'
    follower.apply_conf(['default'])
    follower.configure
    follower.start
    
    # CONNECT PORTS
    imu.pose_samples.connect_to(simulation.pose_in)
    imu.pose_samples.connect_to(follower.pose)
    imu.pose_samples.connect_to(planner.robot_pose_in)
    imu.pose_samples.connect_to(visualizer.robot_pose_in)
    controller.actuators_command.connect_to(locomotion_actuators.cmd_locomotion_actuators)
    transmitter.envire_events.connect_to(traversability.mls_map)
    transmitter.envire_events.connect_to(planner.envire_environment_in)
    traversability.traversability_map.connect_to(visualizer.envire_environment_in)
    traversability.traversability_map.connect_to(planner.envire_environment_in)
    planner.trajectory_out.connect_to(visualizer.trajectory_in)
    planner.trajectory_out.connect_to(simulation.trajectory_in) 
    planner.trajectory_spline_out.connect_to(follower.trajectory)
    planner.trajectory_spline_out.connect_to(visualizer.trajectory_spline_in)
    follower.motion_command.connect_to(controller.motion_command)
    follower.motion_command.connect_to(visualizer.motion_command_in)

    # LOAD MAP
    transmitter.loadEnvironment('dlr.env')
    
    # PLANNER: SET GOAL POS
    planner_write_stop = planner.target_position_in.writer()
    stop_pos = planner_write_stop.new_sample()
    stop_pos.data[0] = 8
    stop_pos.data[1] = -10
    planner_write_stop.write(stop_pos)

    Vizkit.exec
    
    Readline::readline("Press ENTER to exit ...")
end

