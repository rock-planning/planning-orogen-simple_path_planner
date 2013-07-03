#! /usr/bin/env ruby

require 'orocos'
require 'readline'
Orocos::CORBA.max_message_size = 80000000

include Orocos
Orocos.initialize


Orocos.run 'envire::SynchronizationTransmitter' => 'transmitter', 
    'corridor_planner::Traversability' => 'traversability',
    'simple_path_planner::Task' => 'planner', :valgrind=>false do  
  
    transmitter = Orocos.name_service.get 'transmitter'
    traversability = Orocos.name_service.get 'traversability'
    planner = Orocos.name_service.get 'planner'
    
    Orocos.log_all_ports()
    Orocos.conf.load_dir('.')
    traversability.apply_conf(['default'])

    planner.traversability_map_id = '/traversability/map'
    
    transmitter.envire_events.connect_to(traversability.mls_map)
    traversability.traversability_map.connect_to(planner.envire_environment_in)

    #optional for height adjusted trajectories
    #transmitter.envire_events.connect_to(planner.envire_environment_in)
    
    transmitter.configure()
    traversability.configure()
    planner.configure()
    
    transmitter.start()
    traversability.start()
    planner.start()
    
    start_writer = planner.start_position_in.writer();
    stop_writer = planner.target_position_in.writer();
    
    start_pos = start_writer.new_sample()
    stop_pos = stop_writer.new_sample()
    
    start_pos.data[0] = 19
    start_pos.data[1] = 6

    stop_pos.data[0] = 19
    stop_pos.data[1] = 12

    start_writer.write(start_pos)
    stop_writer.write(stop_pos)
    
    transmitter.loadEnvironment('dlr.env')
    Readline::readline("Press ENTER to exit\n") do
    end 
end
