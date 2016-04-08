import numpy 

class DepthFirstPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()

    def Plan(self, start_config, goal_config):
        
        # TODO: Here you will implement the depth first planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        plan = []

        q = []
        start_node_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_node_id = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)

        q.append(start_node_id)
        
        total_nodes = 1
        for idx in range(self.planning_env.discrete_env.dimension):
            total_nodes = total_nodes * self.planning_env.discrete_env.num_cells[idx]

        visited = {}
        parents = {}
        print len(visited)
        count = 0
        while q:
            node_id = q.pop()
            count = count + 1
            succ_node_id = self.planning_env.GetSuccessors(node_id)
            #print succ_node_id
            if node_id not in visited:
                visited[node_id] = 1
                for idx in succ_node_id:
                    '''
                    if (visited[idx] is 0):
                        visited[idx] = 1
                        parents[idx] = node_id
                        q.append(idx)
                    '''
                    if idx not in visited:
                        parents[idx] = node_id
                        q.append(idx)
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

        print('Number of nodes:',count)

        return plan[::-1]

