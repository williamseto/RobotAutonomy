import Queue
import numpy

class AStarPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()


    def Plan(self, start_config, goal_config):

        plan = []
        
        # TODO: Here you will implement the AStar planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
                
        q = Queue.PriorityQueue()
        start_node_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_node_id = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)

        q.put((0, 0, start_node_id))
        
        total_nodes = 1
        for idx in range(self.planning_env.discrete_env.dimension):
            total_nodes = total_nodes * self.planning_env.discrete_env.num_cells[idx]

        visited = {}
        parents = {}

        while (q.empty() is False):
            tup = q.get()
            g_val = tup[1]
            node_id = tup[2]

            #print self.planning_env.discrete_env.NodeIdToConfiguration(node_id)

            succ_node_id = self.planning_env.GetSuccessors(node_id)
            

            for idx in succ_node_id:
                if idx not in visited:
                    visited[idx] = 1
                    parents[idx] = node_id

                    h_val = self.planning_env.ComputeHeuristicCost(idx, goal_node_id)
                    new_g = g_val + 1
                    f_val = (new_g) + h_val
                    q.put((f_val, new_g, idx))

                if (idx == goal_node_id):
                    break
            if (idx == goal_node_id):
                    break

        next_node_id = goal_node_id
        while (next_node_id != start_node_id):
            plan.append(numpy.array(self.planning_env.discrete_env.NodeIdToConfiguration(next_node_id)))
            next_node_id = parents[next_node_id]
        plan.append(numpy.array(start_config))

        plan = numpy.array(plan)

        return plan[::-1]

        return plan
