from RRTTree import RRTTree
from random import random
import numpy as np


class HeuristicRRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize

    def Plan(self, start_config, goal_config, epsilon = 0.001):

        dist = self.planning_env.ComputeConfigDistance(goal_config, start_config)
        C_opt = dist

        #configs are in form (config, current cost, currcost+heuristic)
        start_config = (start_config, 0, 0+dist)

        tree = RRTTree(self.planning_env, start_config)
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        prob_goal = 0.2


        prob_floor = 0.5
        tree.C_max = dist

        while dist > epsilon:
            '''
            if random() < prob_goal:
                target_config = goal_config
            else:
                target_config = self.planning_env.GenerateRandomConfiguration()
            '''
            if random() < prob_goal:
                target_config = goal_config
                n_ind, n_vertex = tree.GetNearestVertex(target_config)
            else:
                ## loop until we find a good quality config to extend to
                m_quality = -1
                r = np.random.rand()
                while r > m_quality:
                    target_config = self.planning_env.GenerateRandomConfiguration()
                    n_ind, n_vertex = tree.GetNearestVertex(target_config)

                    C_vertex = n_vertex[2]
                    #print(C_vertex, C_opt, tree.C_max)
                    m_quality = 1 - ((C_vertex-C_opt)/(tree.C_max-C_opt))
                    m_quality = max(m_quality, prob_floor)
                    r = np.random.rand()
                    #print m_quality

            #not returning a tuple
            e_vertex = self.planning_env.Extend(n_vertex[0], target_config)


            if e_vertex is None:
                pass
            else:
                #compute C_vertex: (dist from n_vertex to e_vertex) + heuristic
                path_cost = self.planning_env.ComputeConfigDistance(n_vertex[0], e_vertex)
                new_cost = n_vertex[1] + path_cost + \
                           self.planning_env.ComputeConfigDistance(e_vertex, goal_config)

                e_vid = tree.AddVertex((e_vertex, n_vertex[1] + path_cost, new_cost))
                if new_cost > tree.C_max:
                    tree.C_max = new_cost

                tree.AddEdge(n_ind, e_vid)
                dist = self.planning_env.ComputeConfigDistance(goal_config, e_vertex)
                if self.visualize:
                    self.planning_env.PlotEdge(n_vertex[0], e_vertex)

        plan = []

        #traverse backwards
        while e_vid != 0:
            e_vid = tree.edges[e_vid]
            #insert at front of the list
            plan.insert(0, tree.vertices[e_vid][0])

        plan.append(goal_config)

        self.num_vertices = len(tree.vertices)
        return plan

    def PlanOriginal(self, start_config, goal_config, epsilon = 0.001):
>>>>>>> 19aee7e... update hrrt
        tree = RRTTree(self.planning_env, start_config)
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        prob_goal = 0.4
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

        return plan
