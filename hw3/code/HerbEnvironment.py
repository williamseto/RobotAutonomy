import numpy
from DiscreteEnvironment import DiscreteEnvironment
import random

import collections
compare = lambda x, y: collections.Counter(x) == collections.Counter(y)

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
            pos_config[dim] = pos_config[dim] + resolution
            if pos_config[dim] < self.upper_limits[dim]:
                if self.collision_check(pos_config) is False:
                    successors.append(self.discrete_env.ConfigurationToNodeId(pos_config))

            neg_config = list(config)
            neg_config[dim] = neg_config[dim] - resolution
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
        
        #return self.ComputeDistance(start_id, goal_id)


        start_config = self.discrete_env.NodeIdToConfiguration(start_id)
        goal_config = self.discrete_env.NodeIdToConfiguration(goal_id)

        start_coord = self.discrete_env.NodeIdToGridCoord(start_id)
        goal_coord = self.discrete_env.NodeIdToGridCoord(goal_id)

        #cost = 1000*numpy.linalg.norm(goal_config-start_config)
        #cost = sum(numpy.subtract(start_coord, goal_coord))
        cost = 100*numpy.linalg.norm(numpy.subtract(goal_coord,start_coord))
        
        return cost

    def GenerateRandomConfiguration(self):
        config = [0] * len(self.robot.GetActiveDOFIndices())
        #
        # TODO: Generate and return a random configuration
        #
        lower_limits, upper_limits = self.robot.GetActiveDOFLimits()
        while True:
            config = [random.uniform(lower_limits[i], upper_limits[i]) for i in range(0, len(lower_limits))]
            if self.collision_check(config):
                pass
            else:
                break
        return numpy.array(config)

    def ComputeConfigDistance(self, start_config, end_config):
        return numpy.linalg.norm(end_config - start_config)

    def Extend(self, start_config, end_config):
        
        #
        # TODO: Implement a function which attempts to extend from 
        #   a start configuration to a goal configuration
        #

        steps = self.ComputeConfigDistance(start_config, end_config)*10
        config_temp = [numpy.linspace(start_config[i], end_config[i], steps) for i in range(0, len(self.robot.GetActiveDOFIndices()))]
        config_temp = numpy.array(zip(*config_temp))
        config = None
        for i in range(0, len(config_temp)):
            if self.collision_check(config_temp[i]):
                return numpy.array(config_temp[i-1])
            else:
                config = numpy.array(config_temp[i])
        return config
        
    def ShortenPath(self, path, timeout=5.0):
        # 
        # TODO: Implement a function which performs path shortening
        #  on the given path.  Terminate the shortening after the 
        #  given timout (in seconds).
        #
        
        #make copy so we don't overwrite the original path
        short_path = list(path)

        init_time = time.time()
        while time.time() - init_time < timeout:
            start = random.randint(0, len(short_path)-3)
            end = random.randint(start+1, len(short_path)-1)
            extend = self.Extend(short_path[start], short_path[end])
            if compare(extend, short_path[end]):
                short_path[start+1:end] = []

        return short_path