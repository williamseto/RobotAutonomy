import numpy
import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment

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

        return cost

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

        
