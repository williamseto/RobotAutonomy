import numpy

class DiscreteEnvironment(object):

    def __init__(self, resolution, lower_limits, upper_limits):

        # Store the resolution
        self.resolution = resolution

        # Store the bounds
        self.lower_limits = lower_limits
        self.upper_limits = upper_limits

        # Calculate the dimension
        self.dimension = len(self.lower_limits)

        # Figure out the number of grid cells that are in each dimension
        self.num_cells = self.dimension*[0]
        for idx in range(self.dimension):
            self.num_cells[idx] = numpy.ceil((upper_limits[idx] - lower_limits[idx])/resolution)


    def ConfigurationToNodeId(self, config):
        
        # TODO:
        # This function maps a node configuration in full configuration
        # space to a node in discrete space
        #
        node_id = 0
        node_id = self.GridCoordToNodeId(self.ConfigurationToGridCoord(config))
        return node_id

    def NodeIdToConfiguration(self, nid):
        
        # TODO:
        # This function maps a node in discrete space to a configuraiton
        # in the full configuration space
        #
        config = [0] * self.dimension
        config = self.GridCoordToConfiguration(self.NodeIdToGridCoord(nid))
        return config
        
    def ConfigurationToGridCoord(self, config):
        
        # TODO:
        # This function maps a configuration in the full configuration space
        # to a grid coordinate in discrete space
        #
        coord = [0] * self.dimension
        for idx in range(self.dimension):
            coord[idx] = numpy.floor(config[idx] / self.resolution)
        return coord

    def GridCoordToConfiguration(self, coord):
        
        # TODO:
        # This function smaps a grid coordinate in discrete space
        # to a configuration in the full configuration space
        #
        config = [0] * self.dimension
        for idx in range(self.dimension):
            config[idx] = (coord[idx]*self.resolution) + (0.5*self.resolution)
        return config

    def GridCoordToNodeId(self,coord):
        
        # TODO:
        # This function maps a grid coordinate to the associated
        # node id 
        node_id = 0
        #always add the first one (no multiplication)
        node_id = node_id + coord[0]
        for idx in range(1, self.dimension):
            stride = 1
            for dim in range(idx):
                stride = stride * self.num_cells[dim]
            node_id = node_id + (coord[idx] * stride)
        return node_id

    def NodeIdToGridCoord(self, node_id):
        
        # TODO:
        # This function maps a node id to the associated
        # grid coordinate
        coord = [0] * self.dimension

        stride = 1
        for dim in range(self.dimension-1):
            stride = stride * self.num_cells[dim]

        for idx in list(reversed(range(1,self.dimension))):
            
            coord[idx] = numpy.floor(node_id / stride)
            node_id = node_id - (coord[idx]*stride)
            stride = stride / self.num_cells[idx-1]

        coord[0] = node_id
        return coord
        
        
        
