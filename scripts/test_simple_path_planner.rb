require 'orocos'
require 'vizkit'
require 'readline'

Orocos::CORBA.max_message_size = 12000000 # stucks if > than this

include Orocos
Orocos.initialize

# Content of these language variables must not be german, otherwise '.' and ','
# are mixed reading numbers and a bad_alloc error occurs loading the scenes.
ENV['LANG'] = 'C'
ENV['LC_NUMERIC'] = 'C'

def distance(p1, p2)
    return Math.sqrt( ((p1.data[0] - p2.data[0]) * (p1.data[0] - p2.data[0])) +
                ((p1.data[1] - p2.data[1]) * (p1.data[1] - p2.data[1])) +
                ((p1.data[2] - p2.data[2]) * (p1.data[2] - p2.data[2])) )
end

Orocos.run 'envire::SynchronizationTransmitter' => 'transmitter', 
        'corridor_planner::Traversability' => 'traversability',
        'simple_path_planner::Task' => 'planner',
        'visualizer_module::Task' => 'visualizer', 
        "valgrind" => false, "wait" => 1000 do
        
    Orocos.conf.load_dir("./config")
    
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
    
    # CONNECT PORTS
    transmitter.envire_events.connect_to(traversability.mls_map)
    transmitter.envire_events.connect_to(planner.envire_environment_in)
    traversability.traversability_map.connect_to(visualizer.envire_environment_in)
    traversability.traversability_map.connect_to(planner.envire_environment_in)
    planner.trajectory_out.connect_to(visualizer.trajectory_in)
    planner.trajectory_spline_out.connect_to(visualizer.trajectory_spline_in)

    # LOAD MAP
    transmitter.loadEnvironment("#{ENV['AUTOPROJ_PROJECT_BASE']}/bundles/spacebot/data/traversability_maps/dlr.env")
    
    # PLANNER: SET START AND GOAL POS
    r = Random.new
    start_writer = planner.start_position_in.writer
    start_pos = start_writer.new_sample
    start_pos.data[0] = -15
    start_pos.data[1] = -11
    start_writer.write(start_pos)
    
    r = Random.new
    goal_writer = planner.target_position_in.writer
    goal_pos = goal_writer.new_sample
    goal_pos.data[0] = 15 #r.rand(-17...17)
    goal_pos.data[1] = 11 # r.rand(-13...13)
    goal_writer.write(goal_pos)
    
    view3d = Vizkit.vizkit3d_widget
    view3d.show
    dstar_lite_trav_map_viz = Vizkit.default_loader.EnvireVisualization
    
    Vizkit.connect_port_to 'planner', 'internal_trav_map', :pull => false, :update_frequency => 33 do |sample, name|
        dstar_lite_trav_map_viz.updateBinaryEvents(sample)
    end
    
    #while true
        transmitter.loadEnvironment("#{ENV['AUTOPROJ_PROJECT_BASE']}/bundles/spacebot/data/traversability_maps/dlr.env")
    #end
   
    Vizkit.display planner
    begin
        Vizkit.exec
    rescue Interrupt => e

    end

    Readline::readline("Press ENTER to exit ...")
end

