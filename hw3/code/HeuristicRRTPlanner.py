from RRTTree import RRTTree
from random import random


class HeuristicRRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize

    def Plan(self, start_config, goal_config, epsilon = 0.001):
        tree = RRTTree(self.planning_env, start_config)
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        prob_goal = 0.2
        dist = self.planning_env.ComputeConfigDistance(goal_config, start_config)
        while dist > epsilon:
            if random() < prob_goal:
                target_config = goal_config
            else:
                target_config = self.planning_env.GenerateRandomConfiguration()

            n_ind, n_vertex = tree.GetNearestVertex(target_config)
            e_vertex = self.planning_env.Extend(n_vertex, target_config)

            if e_vertex is None:
                pass
            else:
                e_vid = tree.AddVertex(e_vertex)
                tree.AddEdge(n_ind, e_vid)
                dist = self.planning_env.ComputeConfigDistance(goal_config, e_vertex)
                if self.visualize:
                    self.planning_env.PlotEdge(n_vertex, e_vertex)

        plan = []

        #traverse backwards
        while e_vid != 0:
            e_vid = tree.edges[e_vid]
            #insert at front of the list
            plan.insert(0, tree.vertices[e_vid])

        plan.append(goal_config)

        self.num_vertices = len(tree.vertices)
        return plan
