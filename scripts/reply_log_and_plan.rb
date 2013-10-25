require 'orocos'
require 'vizkit'
require 'readline'
require 'orocos/log'
include Orocos

Orocos::CORBA.max_message_size = 82000000 

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


replay = Log::Replay.open('sb_navigation.0.log', 'sb_graph_slam.0.log')


Orocos.run 'simple_path_planner::Task' => 'planner',
           "valgrind" => false, "wait" => 1000 do
     
    Orocos.conf.load_dir("#{ENV['AUTOPROJ_PROJECT_BASE']}/bundles/spacebot/config/orogen")
        
    # PLANNER 
    planner = TaskContext::get 'planner'
    planner.apply_conf(['default'])
    planner.configure()
    planner.start()
    
    gotMap = false
    
    replay.traversability.traversability_map.connect_to(planner.envire_environment_in)
    
    replay.velodyne_slam.pose_samples.connect_to(planner.robot_pose_in)    
    
    view3d = Vizkit.vizkit3d_widget
    view3d.show

    Vizkit.display planner.debug_internal_trav_map
    Vizkit.display planner.trajectory_spline_out
    Vizkit.display replay.traversability.traversability_map
    
    
    Vizkit.control replay

    #start gui
    Vizkit.exec
#     Vizkit.exec do
#         
#         tr_map = nil
#        while(!(tr_map = map_reader.read))
#            
#            replay.step
# 
#        end
#        
#        input_map.updateBinaryEvents(tr_map)
#        
#        Vizkit.step
#        
#        puts("got map")
#        
#         input = Readline::readline
#         if input == "p"
#             puts("writing goal pos")
#             start_writer.write(start_pos)
#         end
# 
#         gotMap = false
# 
#     end
#     
# #     view3d = Vizkit.vizkit3d_widget
# #     view3d.show
# #     dstar_lite_trav_map_viz = Vizkit.default_loader.EnvireVisualization
# #     
# #     Vizkit.connect_port_to 'planner', 'internal_trav_map', :pull => false, :update_frequency => 33 do |sample, name|
# #         dstar_lite_trav_map_viz.updateBinaryEvents(sample)
# #     end
# #     
# #     Vizkit.display planner
# #     begin
# #         Vizkit.exec
# #     rescue Interrupt => e
# # 
# #     end

    Readline::readline("Press ENTER to exit ...")
end

