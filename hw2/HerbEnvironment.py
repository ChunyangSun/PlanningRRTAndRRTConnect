import numpy as np
import pylab as pl
import time
import copy

class HerbEnvironment(object):
    
    def __init__(self, herb):
        self.robot = herb.robot

        # add a table and move the robot into place
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)

        table_pose = np.array([[ 0, 0, -1, 0.7], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)

        # set the camera
        camera_pose = np.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
                                   [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
                                   [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.robot.GetEnv().GetViewer().SetCamera(camera_pose)
        
        # goal sampling probability
        self.p = 0.0

    def SetGoalParameters(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p
    
    def isValid(self, config):
        self.robot.SetActiveDOFValues(config)
        return not self.robot.GetEnv().CheckCollision(self.robot)


    def GenerateRandomConfiguration(self):
        lower_limits, upper_limits = self.robot.GetActiveDOFLimits()
        config = np.random.random(len(lower_limits))
        config = (config * (upper_limits - lower_limits)) + lower_limits
        return config if self.isValid(config) else self.GenerateRandomConfiguration() #could recur infinitely, but unlikely


    #distance in configuration space is just sum of joint differences?
    #could do FK, but do we need to? --NJC
    def ComputeDistance(self, start_config, end_config):
        return np.sum(np.absolute(end_config - start_config))


    def Extend(self, start_config, end_config):
        configs = np.array([start_config, end_config])
        last = start_config
        for tau in np.arange(0.0, 1.01, 0.01):
            newconfig = np.average(configs, axis=0, weights=[1.0-tau, tau])
            if(self.isValid(newconfig)):
                last = newconfig
            else:
                break
        # return the last collision free point 
        return last
    
    def ComputePathLength(self, path):
        i = 1
        length = 0.0
        while i < len(path):
            length += self.ComputeDistance(path[i-1], path[i])
            i += 1
	print "Number of nodes: ", i
        return length


    def ShortenPath(self, path, timeout=5.0):
        
        # 
        # TODO: Implement a function which performs path shortening
        #  on the given path.  Terminate the shortening after the 
        #  given timout (in seconds).
        #

	start = end = time.time()
        toBeDeleted = [1000,1000,1000,1000,1000,1000,1000]        
	lastPath = []
	idxList = []

	# stop when there's no more shortening to be done 
	while not np.array_equal(path, lastPath): 
	    lastPath = copy.deepcopy(path)

	    # keep checking alternate adjacent nodes  
	    for idx in xrange(1, len(path) - 2):
                n1 = path[idx - 1] 
                n2 = path[idx + 1]

                # make sure you are not connecting the two toBeDeleted nodes
                if (not np.array_equal(n1, toBeDeleted)) and (not np.array_equal(n2, toBeDeleted)) and \
	        np.array_equal(self.Extend(n1, n2), n2):

		    # record the toBeDeleted node and its index
                    path[idx] = toBeDeleted
		    idxList.append(idx)

	    # delete the toBeDeleted node according to index
	    shortPath = np.delete(path, idxList, axis = 0)

	    # break out of while loop when time out 
	    end = time.time() 
	    if end - start > timeout: break 
	#print "after shortening: "
	#print shortPath

        return shortPath
