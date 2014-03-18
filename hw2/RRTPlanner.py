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

    def Plan(self, start_config, goal_config, epsilon = 0.01): # epsilon = 0.001
        
        tree = RRTTree(self.planning_env, start_config)

        plan = []
        plancopy = []
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space - (jane) which is 2 in this case? 
        
        q_new = start_config
        sampleGoal = 5
        count = 0
        dist = 100
        # if q_new is very near to the goal, break out of the loop 
        # don't worry about goal not included in vertices or edges: start and goal are in path 
        while (dist > epsilon) : 
            # Pick a sample q_rand in space C, occasionally, sample the goal 
            if count == sampleGoal:
                q_rand = goal_config
                count = 0 
            else: q_rand = self.planning_env.GenerateRandomConfiguration()
            count += 1
            # find q_rand's nearest milestone q in the tree:    
            [vid_start, q_near] = tree.GetNearestVertex(q_rand)             
            # extend a step from q to q_new towards q_rand 
            q_new = self.planning_env.Extend(q_near, q_rand)
            # add q_new to the tree
            if q_new != None:
                vid_end = tree.AddVertex(q_new)
                tree.AddEdge(vid_start, vid_end)
                # call whenever an edge is added on the tree 
                self.planning_env.PlotEdge(q_near, q_new) 
                # get out of the loop if q_new is very near to q_goal
                dist = self.planning_env.ComputeDistance(q_new, goal_config)    #q_new, goal_config
        # get the goal ID in edges 
            goalID = max(tree.edges.keys())
            keyID = goalID
        # now q_new is goal, find way back to start 
	pathlength = 0
	lastx = 0
	lasty = 0
        while (keyID != tree.GetRootId()):
            # ID in dict edge is indexes of [x,y] position in 2D list vertices
            valueID = tree.edges[keyID] 
            plancopy.append(tree.vertices[valueID]) 
	    pathlength = pathlength + pow(pow((tree.vertices[valueID][0]-lastx), 2) + pow((tree.vertices[valueID][1]-lasty), 2), 0.5)
            lastx = tree.vertices[valueID][0]
            lasty = tree.vertices[valueID][1]
            keyID = valueID

        while (len(plancopy) > 0):
            plan.append(plancopy.pop())
        plan.append(goal_config)
	
	print pathlength	
        return plan
