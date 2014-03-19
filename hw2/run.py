#!/usr/bin/env python

import argparse, numpy, openravepy, time

from HerbRobot import HerbRobot
from SimpleRobot import SimpleRobot
from HerbEnvironment import HerbEnvironment
from SimpleEnvironment import SimpleEnvironment

from RRTPlanner import RRTPlanner
from RRTConnectPlanner import RRTConnectPlanner

import time

def main(robot, planning_env, planner, iterations = 1, show = False):

    start_config = numpy.array(robot.GetCurrentConfiguration())
    if robot.name == 'herb':
        goal_config = numpy.array([ 3.68, -1.90,  0.00,  2.20,  0.00,  0.00,  0.00 ]) #3.68, -1.90,  0.00,  2.20,  0.00,  0.00,  0.00
    else:
        goal_config = numpy.array([2.0, 0.0]) # 2.0, 0.0

    i = 0
    while i < iterations:
        print 'Pass: ' + str(i)
        start = time.time()
        plan = planner.Plan(start_config, goal_config)
        plantime = time.time() - start
        print("Original Path Length: {:.3f}".format(planning_env.ComputePathLength(plan)))
        print("Original Path Nodes: " + str(len(plan)))
        if show:
            traj = robot.ConvertPlanToTrajectory(plan)
            robot.ExecuteTrajectory(traj)
        plan_short = planning_env.ShortenPath(plan)
        print("Shortened Path Length: {:.3f}".format(planning_env.ComputePathLength(planning_env.ShortenPath(plan))))
        print("Shortened Path Nodes: " + str(len(plan_short)))
        if show:
            traj = robot.ConvertPlanToTrajectory(plan_short)
            robot.ExecuteTrajectory(traj)
        print("Planning time: " + str(plantime))
        i += 1

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description='script for testing planners')
    
    parser.add_argument('-r', '--robot', type=str, default='simple',
                        help='The robot to load (herb or simple)')
    parser.add_argument('-p', '--planner', type=str, default='rrt',
                        help='The planner to run (rrt or rrtconnect)')
    parser.add_argument('-m', '--manip', type=str, default='right',
                        help='The manipulator to plan with (right or left) - only applicable if robot is of type herb')
    parser.add_argument('-v', '--visualize', action='store_true',
                        help='Enable visualization of tree growth (only applicable for simple robot)')
    parser.add_argument('-d', '--debug', action='store_true',
                        help='Enable debug logging')
    parser.add_argument('-n', '--noshow', action='store_true',
                        help='Disable Robot Dancing for speed')
    parser.add_argument('-i', '--iterations', type=int, default=1,
                    help='How many times to iterate')

    args = parser.parse_args()
    
    openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
    openravepy.misc.InitOpenRAVELogging()

    if args.debug:
        openravepy.RaveSetDebugLevel(openravepy.DebugLevel.Debug)

    env = openravepy.Environment()
    env.SetViewer('qtcoin')
    env.GetViewer().SetName('Homework 2 Viewer')

    # First setup the environment and the robot
    visualize = args.visualize
    if args.robot == 'herb':
        robot = HerbRobot(env, args.manip)
        planning_env = HerbEnvironment(robot)
        visualize = False
    elif args.robot == 'simple':
        robot = SimpleRobot(env)
        planning_env = SimpleEnvironment(robot)
    else:
        print 'Unknown robot option: %s' % args.robot
        exit(0)

    # Next setup the planner
    if args.planner == 'rrt':
        planner = RRTPlanner(planning_env, visualize=visualize)
    elif args.planner == 'rrtconnect':
        planner = RRTConnectPlanner(planning_env, visualize=visualize)
    else:
        print 'Unknown planner option: %s' % args.planner
        exit(0)
    main(robot, planning_env, planner, args.iterations, not args.noshow)
