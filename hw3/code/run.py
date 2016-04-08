#!/usr/bin/env python
import time

import argparse, numpy, openravepy, time

from HerbRobot import HerbRobot
from SimpleRobot import SimpleRobot
from HerbEnvironment import HerbEnvironment
from SimpleEnvironment import SimpleEnvironment

from AStarPlanner import AStarPlanner
from DepthFirstPlanner import DepthFirstPlanner
from BreadthFirstPlanner import BreadthFirstPlanner
from HeuristicRRTPlanner import HeuristicRRTPlanner

def main(robot, planning_env, planner):

    raw_input('Press any key to begin planning')

    start_config = numpy.array(robot.GetCurrentConfiguration())
    if robot.name == 'herb':
        goal_config = numpy.array([ 4.6, -1.76, 0.00, 1.96, -1.15, 0.87, -1.43] )
        #goal_config = numpy.array([ 3.68, -1.90,  0.00,  2.20,  0.00,  0.00,  0.00 ])
    else:
        goal_config = numpy.array([3.0, 0.0])

    plan_start_time=time.time()

    #use this for hrrt
    plan = planner.Plan(start_config, goal_config)

    #use this for rrt
    #plan = planner.PlanOriginal(start_config, goal_config)

    print ('plan time', (time.time() - plan_start_time))
    print ('path length', len(plan))

    #print plan

    if planner.visualize:
        planning_env.InitializePlot(goal_config)
        if robot.name != 'herb':
            #Visualize
            old_vertex = plan[0]
            for idx in range (1,len(plan)):
                new_vertex = plan[idx]
                planning_env.PlotEdge(old_vertex,new_vertex)
                old_vertex = new_vertex

    short_path = planning_env.ShortenPath(plan)

    rrt_dist = 0
    for i in range(len(short_path)-1):
        rrt_dist = rrt_dist + planning_env.ComputeConfigDistance(short_path[i], short_path[i+1])
    print('rrtdist', rrt_dist)
    traj = robot.ConvertPlanToTrajectory(plan)

    if planner.visualize:
        #plot shortened path
        planning_env.InitializePlot(goal_config)
        for i in range(0, len(short_path)-1):
            planning_env.PlotEdge(short_path[i], short_path[i+1])


    raw_input('Press any key to execute trajectory')
    robot.ExecuteTrajectory(traj)

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description='script for testing planners')
    
    parser.add_argument('-r', '--robot', type=str, default='simple',
                        help='The robot to load (herb or simple)')
    parser.add_argument('-p', '--planner', type=str, default='astar',
                        help='The planner to run (astar, bfs, dfs or hrrt)')
    parser.add_argument('-v', '--visualize', action='store_true',
                        help='Enable visualization of tree growth (only applicable for simple robot)')
    parser.add_argument('--resolution', type=float, default=0.1,
                        help='Set the resolution of the grid (default: 0.1)')
    parser.add_argument('-d', '--debug', action='store_true',
                        help='Enable debug logging')
    parser.add_argument('-m', '--manip', type=str, default='right',
                        help='The manipulator to plan with (right or left) - only applicable if robot is of type herb')
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
        planning_env = HerbEnvironment(robot, args.resolution)
        visualize = False
    elif args.robot == 'simple':
        robot = SimpleRobot(env)
        planning_env = SimpleEnvironment(robot, args.resolution)
    else:
        print 'Unknown robot option: %s' % args.robot
        exit(0)

    # Next setup the planner
    if args.planner == 'astar':
        planner = AStarPlanner(planning_env, visualize)
    elif args.planner == 'bfs':
        planner = BreadthFirstPlanner(planning_env, visualize)
    elif args.planner == 'dfs':
        planner = DepthFirstPlanner(planning_env, visualize)
    elif args.planner == 'hrrt':
        planner = HeuristicRRTPlanner(planning_env, visualize)
    else:
        print 'Unknown planner option: %s' % args.planner
        exit(0)

    main(robot, planning_env, planner)

    import IPython
    IPython.embed()

        
    
