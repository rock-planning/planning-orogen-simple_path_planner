require 'orocos'
require 'vizkit'
require 'readline'
require 'orocos/log'
include Orocos

Orocos::CORBA.max_message_size = 82000000 

Orocos.initialize

        
# PLANNER 
planner = TaskContext::get 'planner'

target_writer = planner.target_position_in.writer
target_pos = target_writer.new_sample
target_pos.data[0] = 50
target_pos.data[1] = 2

while(true)
    puts("Press enter to plan")
    readline
    target_writer.write(target_pos)
end

