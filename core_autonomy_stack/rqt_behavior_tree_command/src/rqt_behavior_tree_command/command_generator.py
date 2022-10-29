#!/usr/bin/python3
import rospy
import sys
import yaml
from collections import OrderedDict
from std_msgs.msg import String
from behavior_tree import behavior_tree as bt
from behavior_tree_msgs.msg import Status, BehaviorTreeCommand, BehaviorTreeCommands


def command_callback(msg):
    for group in groups:
        if msg.data in group:
            commands = BehaviorTreeCommands()
            for c in group:
                command = BehaviorTreeCommand()
                command.condition_name = c
                if msg.data == c:
                    command.status = Status.SUCCESS
                else:
                    command.status = Status.FAILURE
                commands.commands.append(command)
            commands_pub.publish(commands)
            return


if __name__ == '__main__':
    rospy.init_node('command_generator')

    # init params
    config_filename = rospy.get_param('~config_filename', '')

    # init subscribers
    command_sub = rospy.Subscriber('behavior_tree_command_string', String, command_callback)
    
    # init publishers
    commands_pub = rospy.Publisher('behavior_tree_commands', BehaviorTreeCommands, queue_size=1)
    
    #config_filename = sys.argv[1]
    y = yaml.safe_load(open(config_filename, 'r').read())
    
    groups = []
    
    for group in y['groups']:
        group_name = list(group.keys())[0]
        groups.append([])
        
        for command_dict in group[group_name]:
            command_name = list(command_dict.keys())[0]
            condition_name = command_dict[command_name]['condition_name']
            groups[-1].append(condition_name)

    rospy.spin()
