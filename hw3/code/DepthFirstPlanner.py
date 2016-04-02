class DepthFirstPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()

    def Plan(self, start_config, goal_config):
        
        plan = []

        # TODO: Here you will implement the depth first planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        q = Stack()
        start_node_id = self.planning_env.ConfigurationToNodeId(start_config)
        goal_node_id = self.planning_env.ConfigurationToNodeId(goal_config)

        q.push(start_node_id)
        
        total_nodes = 1
        for idx in range(self.dimension):
            total_nodes = total_nodes * self.num_cells[idx]

        visited = [0] * total_nodes
        parents = [0] * total_nodes

        while (!q.empty()):
            node_id = q.pop()
            succ_node_id = self.planning_env.GetSuccessors(node_id)
            for idx in range(succ_node_id):
                if (!visited[succ_node_id[idx]]):
                    visited[succ_node_id[idx]] = 1
                    parents[succ_node_id[idx]] = node_id
                    q.push(succ_node_id[idx])
                if (succ_node_id[idx] == goal_node_id):
                    break
            if (succ_node_id[idx] == goal_node_id):
                    break

        next_node_id = goal_node_id
        while (next_node_id ! = start_node_id):
            plan.append(numpy.array(self.planning_env.NodeIdToConfiguration(next_node_id)))
            next_node_id = parents[next_node_id]
        plan.append(numpy.array(start_config))

        return plan.reverse()

