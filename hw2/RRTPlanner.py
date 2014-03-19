import numpy
from RRTTree import RRTTree
from SimpleRobot import SimpleRobot
from SimpleEnvironment import SimpleEnvironment
import copy

import IPython

class RRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize

    #  Returns a path should be an array
    #  of dimension k x n where k is the number of waypoints
    #  and n is the dimension of the robots configuration space
    def Plan(self, start_config, goal_config, epsilon = 0.01): # epsilon = 0.001
        #set up visualizer
        if self.visualize: 
            self.planning_env.InitializePlot(goal_config)
        
        tree = RRTTree(self.planning_env, start_config)
        q_new = start_config
        count = 0
        dist = 9999
        vid_end = 0
        # if q_new is very near to the goal, break out of the loop 
        # don't worry about goal not included in vertices or edges: start and goal are in path 
        while (dist > epsilon) : 
            # Pick a sample q_rand in space C, sample the goal every 5 cycles 
            q_rand  = self.planning_env.GenerateRandomConfiguration() if (count % 5) else goal_config
            count += 1
            # find q_rand's nearest milestone q in the tree:    
            [vid_start, q_near] = tree.GetNearestVertex(q_rand)             
            # extend a step from q to q_new towards q_rand 
            q_new = self.planning_env.Extend(q_near, q_rand)

            # add q_new to the tree, #TODO: possible improvement, detect/avoid duplicates
            vid_end = tree.AddVertex(q_new)
            tree.AddEdge(vid_start, vid_end)
            # update the distance from the goal
            dist = self.planning_env.ComputeDistance(q_new, goal_config)

            #visualize the updated tree
            if self.visualize:
                self.planning_env.PlotEdge(q_near, q_new) 

        print "Tree size: " + str(len(tree.vertices))
        plan = []
        currVertex = vid_end
        while (currVertex != tree.GetRootId()):
            currVertex = tree.edges[currVertex] 
            plan.insert(0, tree.vertices[currVertex]) 
        plan.append(goal_config)

        return plan
