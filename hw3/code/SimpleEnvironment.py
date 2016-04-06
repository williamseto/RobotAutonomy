import numpy
import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment
import random

class SimpleEnvironment(object):
    
    def __init__(self, herb, resolution):
        self.robot = herb.robot
        self.lower_limits = [-5., -5.]
        self.upper_limits = [5., 5.]
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)


        # add an obstacle
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 1.5], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)

    def GetSuccessors(self, node_id):

        successors = []

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids that represent the neighboring
        #  nodes
        config = self.discrete_env.NodeIdToConfiguration(node_id)
        resolution = self.discrete_env.resolution
        #doing 4-connected
        transform_t = numpy.eye(4)
        #print config
        #print resolution
        #left side
        left_config = numpy.subtract(config, [resolution, 0])
        #print self.discrete_env.ConfigurationToNodeId(left_config)
        if left_config[0] > self.lower_limits[0]:
            transform_t[0][3] = left_config[0]
            transform_t[1][3] = left_config[1]
            self.robot.SetTransform(transform_t)
            if self.robot.GetEnv().CheckCollision(self.robot) is False:
                successors.append(self.discrete_env.ConfigurationToNodeId(left_config))


        #right side
        right_config = numpy.add(config, [resolution, 0])
        #print self.discrete_env.ConfigurationToNodeId(right_config)
        if right_config[0] < self.upper_limits[0]:
            transform_t[0][3] = right_config[0]
            transform_t[1][3] = right_config[1]
            self.robot.SetTransform(transform_t)
            if self.robot.GetEnv().CheckCollision(self.robot) is False:
                successors.append(self.discrete_env.ConfigurationToNodeId(right_config))

        #top side
        top_config = numpy.subtract(config, [0, resolution])
        #print self.discrete_env.ConfigurationToNodeId(top_config)
        if top_config[1] > self.lower_limits[1]:
            transform_t[0][3] = top_config[0]
            transform_t[1][3] = top_config[1]
            self.robot.SetTransform(transform_t)
            if self.robot.GetEnv().CheckCollision(self.robot) is False:
                successors.append(self.discrete_env.ConfigurationToNodeId(top_config))

        #bottom side
        bottom_config = numpy.add(config, [0, resolution])
        #print self.discrete_env.ConfigurationToNodeId(bottom_config)
        if bottom_config[1] < self.upper_limits[1]:
            transform_t[0][3] = bottom_config[0]
            transform_t[1][3] = bottom_config[1]
            self.robot.SetTransform(transform_t)
            if self.robot.GetEnv().CheckCollision(self.robot) is False:
                successors.append(self.discrete_env.ConfigurationToNodeId(bottom_config))

        return successors

    def ComputeDistance(self, start_id, end_id):

        dist = 0

        # TODO: Here you will implement a function that 
        # computes the distance between the configurations given
        # by the two node ids

        start_config = self.discrete_env.NodeIdToGridCoord(start_id)
        end_config = self.discrete_env.NodeIdToGridCoord(end_id)

        dist = sum(numpy.subtract(start_config, end_config))

        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        cost = 0

        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids

        return self.ComputeDistance(start_id, goal_id)

    def InitializePlot(self, goal_config):
        self.fig = pl.figure()
        pl.xlim([self.lower_limits[0], self.upper_limits[0]])
        pl.ylim([self.lower_limits[1], self.upper_limits[1]])
        pl.plot(goal_config[0], goal_config[1], 'gx')

        # Show all obstacles in environment
        for b in self.robot.GetEnv().GetBodies():
            if b.GetName() == self.robot.GetName():
                continue
            bb = b.ComputeAABB()
            pl.plot([bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0]],
                    [bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1]], 'r')
                    
                     
        pl.ion()
        pl.show()
        
    def PlotEdge(self, sconfig, econfig):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                'k.-', linewidth=2.5)
        pl.draw()

       
    def collision_check(self, x, y):
        transform_t = numpy.eye(4)
        transform_t[0][3] = x
        transform_t[1][3] = y
        self.robot.SetTransform(transform_t)
        return self.robot.GetEnv().CheckCollision(self.robot) 
    def GenerateRandomConfiguration(self):
        #
        # TODO: Generate and return a random configuration
        #
        while True:
            x = random.uniform(self.lower_limits[0], self.upper_limits[0])
            y = random.uniform(self.lower_limits[1], self.upper_limits[1])
            if self.collision_check(x, y):
                pass
            else:
                config = [x, y]
                break
        return numpy.array(config)

    def ComputeConfigDistance(self, start_config, end_config):
        return numpy.linalg.norm(end_config - start_config)

    def Extend(self, start_config, end_config):
        #
        # TODO: Implement a function which attempts to extend from 
        #   a start configuration to a goal configuration
        #
        steps = self.ComputeConfigDistance(start_config, end_config)*25
        x = numpy.linspace(start_config[0], end_config[0], steps)
        y = numpy.linspace(start_config[1], end_config[1], steps)
        config = None
        for i in range(0, len(x)):
            if self.collision_check(x[i], y[i]) and i>0:
                #print "collision"
                return numpy.array([x[i-1], y[i-1]])
            else:
                config = numpy.array([x[i], y[i]])
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