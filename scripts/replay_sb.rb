#! /usr/bin/env ruby
#library for displaying data
require 'vizkit'
require 'readline'
require 'eigen'
require 'rock/bundle'

if !ARGV[0]  then 
    puts "usage: replay.rb log_dir"
    exit
end


#load log file 
log = Orocos::Log::Replay.open(ARGV[0])
Orocos::CORBA::max_message_size = 100000000

log.traversability.track(false) 
log.global_planner.track(false) 
log.transformer_broadcaster.track(false) 
log.transformer_broadcaster.rename('foo')
log.name_service.deregister 'traversability'
log.name_service.deregister 'global_planner'
log.name_service.deregister 'transformer_broadcaster'

Bundles.initialize
# Use rock-bundle-default spacebot before
Bundles.transformer.load_conf(Bundles.find_file('config', 'transforms_scripts.rb'))

Bundles.run 'corridor_planner::Traversability' => 'traversability', 
        'simple_path_planner::Task' => 'global_planner', :valgrind => false, :output => nil do |p|
        
    velodyne_slam = Bundles::get 'velodyne_slam'
    traversability = Bundles::get 'traversability'
    global_planner = Bundles::get 'global_planner'

    velodyne_slam.envire_map.connect_to(traversability.mls_map)
    traversability.traversability_map.connect_to(global_planner.envire_environment_in)
    
    traversability.apply_conf(['default'])
    traversability.configure()
    traversability.start()
    
    global_planner.apply_conf(['default'])
    global_planner.configure()
    global_planner.start()
    
    Vizkit.display velodyne_slam.pose_samples, :widget => Vizkit.default_loader.RigidBodyStateVisualization
    Vizkit.display global_planner.trajectory_spline_out
    Vizkit.display traversability.traversability_map
    Vizkit.control log


    Vizkit.exec()
end

