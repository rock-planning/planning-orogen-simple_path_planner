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

def distance(p1, p2)
    return Math.sqrt( ((p1.data[0] - p2.data[0]) * (p1.data[0] - p2.data[0])) +
                ((p1.data[1] - p2.data[1]) * (p1.data[1] - p2.data[1])) )
end

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
    
    # VELODYNE
    velodyne = TaskContext.get 'mars_velodyne'
    velodyne.apply_conf(['default'])
    velodyne.configure
    velodyne.start   
    velodyne.addCamera('velodyne90',90);
    velodyne.addCamera("velodyne180",180);
    velodyne.addCamera("velodyne270",270);
    
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
    velodyne.pointcloud.connect_to(visualizer.pointcloud_in)

    # LOAD MAP
    transmitter.loadEnvironment('dlr.env')
    
    # PLANNER: SET GOAL POS
    planner_write_stop = planner.target_position_in.writer
    stop_pos = planner_write_stop.new_sample
    stop_pos.data[0] = 2 #0
    stop_pos.data[1] = 2 #-10
    planner_write_stop.write(stop_pos)

    r = Random.new
    goal_writer = planner.target_position_in.writer
    goal_pos = goal_writer.new_sample
    pose_reader = imu.pose_samples.reader
    while true
        goal_pos.data[0] = r.rand(-17...17)
        goal_pos.data[1] = r.rand(-13...13)
        puts "Set new goal pos"
        goal_writer.write(goal_pos)

        while true
            if rbs = pose_reader.read
                position = rbs.position
                sleep 0.2
                dist = distance(goal_pos, position)
                puts "Distanz: #{dist}"
                if(dist < 1.0)
                    puts "Goal reached"
                    break
                else
                    sleep 1
                end
            else
                puts "no position received"
                sleep 1
            end
        end
    end
    


    Vizkit.exec
    
    Readline::readline("Press ENTER to exit ...")
end

