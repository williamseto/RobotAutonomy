import numpy
from DiscreteEnvironment import DiscreteEnvironment

class HerbEnvironment(object):
    
    def __init__(self, herb, resolution):
        
        self.robot = herb.robot
        self.lower_limits, self.upper_limits = self.robot.GetActiveDOFLimits()
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

        # account for the fact that snapping to the middle of the grid cell may put us over our
        #  upper limit
        upper_coord = [x - 1 for x in self.discrete_env.num_cells]
        upper_config = self.discrete_env.GridCoordToConfiguration(upper_coord)
        for idx in range(len(upper_config)):
            self.discrete_env.num_cells[idx] -= 1

        # add a table and move the robot into place
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 0.7], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)
        
        # set the camera
        camera_pose = numpy.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
                                   [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
                                   [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.robot.GetEnv().GetViewer().SetCamera(camera_pose)
    
    def collision_check(self, x):
        env = self.robot.GetEnv()
        self.robot.SetDOFValues(x, self.robot.GetActiveDOFIndices(), True)
        return env.CheckCollision(self.robot) or self.robot.CheckSelfCollision()

    def GetSuccessors(self, node_id):

        successors = []

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids that represent the neighboring
        #  nodes

        config = self.discrete_env.NodeIdToConfiguration(node_id)

        resolution = self.discrete_env.resolution

        for dim in range(len(self.robot.GetActiveDOFIndices())):
            #make copy of original config
            pos_config = list(config)
            pos_config[dim] = pos_config[dim] + 2*resolution
            if pos_config[dim] < self.upper_limits[dim]:
                if self.collision_check(pos_config) is False:
                    successors.append(self.discrete_env.ConfigurationToNodeId(pos_config))

            neg_config = list(config)
            neg_config[dim] = neg_config[dim] - 2*resolution
            if neg_config[dim] > self.lower_limits[dim]:
                if self.collision_check(neg_config) is False:
                    successors.append(self.discrete_env.ConfigurationToNodeId(neg_config))


        
        return successors

    def ComputeDistance(self, start_id, end_id):

        dist = 0

        # TODO: Here you will implement a function that 
        # computes the distance between the configurations given
        # by the two node ids

        #do it in config space

        start_config = self.discrete_env.NodeIdToConfiguration(start_id)
        end_config = self.discrete_env.NodeIdToConfiguration(end_id)
       
        return numpy.linalg.norm(end_config - start_config)

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        cost = 0

        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids
        
        return self.ComputeDistance(start_id, goal_id)

