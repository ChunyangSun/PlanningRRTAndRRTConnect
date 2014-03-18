import numpy, operator
from RRTPlanner import RRTTree
import IPython

class RRTConnectPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        

    def Plan(self, start_config, goal_config, epsilon = 0.001):
        
        ftree = RRTTree(self.planning_env, start_config)
        rtree = RRTTree(self.planning_env, goal_config)
        plan = []
        plancopy = []
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt connect planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        fq_new = start_config
        sampleGoal = 5
        count = 0
        dist = 100

        while (dist > epsilon) : 
            # Pick a sample q_rand in space C, occasionally, sample the goal 
            if count == sampleGoal:
                q_rand = goal_config
                count = 0 
            else: q_rand = self.planning_env.GenerateRandomConfiguration()
            count += 1
            # find q_rand's nearest milestone q in the tree:    
            [fvid_start, fq_near] = ftree.GetNearestVertex(q_rand)             
            # extend a step from q to q_new towards q_rand 
            fq_new = self.planning_env.Extend(fq_near, q_rand)
            # add q_new to the tree
            if fq_new != None:
                # IPython.embed()
                # extend node from the reverse tree 
                [rvid_start, rq_near] = rtree.GetNearestVertex(fq_new)
                rq_new = self.planning_env.Extend(rq_near, fq_new)
                # get the front and reverse tree vertex ID, they are independent, both start from root 0 
                fvid_end = ftree.AddVertex(fq_new)
                rvid_end = rtree.AddVertex(rq_new)
                # add new edge to front and reverse tree, they are independent 
                ftree.AddEdge(fvid_start, fvid_end)
                rtree.AddEdge(rvid_start, rvid_end)
                # call whenever an edge is added on the tree 
                self.planning_env.PlotEdge(fq_near, fq_new)
                self.planning_env.PlotEdge(rq_near, rq_new) 
                # get out of the loop if q_new is very near to q_goal
                dist = self.planning_env.ComputeDistance(fq_new, rq_new)    
        # get the goal ID in edges 
        fkeyID = max(ftree.edges.keys())
        rkeyID = max(rtree.edges.keys())
	pathlength = 0
	lastx = 0
	lasty = 0
        # add front-tree path into plancopy
        while (fkeyID != ftree.GetRootId()):
            # ID in dict edge is indexes of [x,y] position in 2D list vertices
            fvalueID = ftree.edges[fkeyID] 
            plancopy.append(ftree.vertices[fvalueID]) 
	    pathlength = pathlength + pow(pow((ftree.vertices[fvalueID][0]-lastx), 2) + pow((ftree.vertices[fvalueID][1]-lasty), 2), 0.5)
            lastx = ftree.vertices[fvalueID][0]
            lasty = ftree.vertices[fvalueID][1]
            fkeyID = fvalueID
        # reverse plancopy, from front tree root to the connecting vertice
        while (len(plancopy) > 0):
            plan.append(plancopy.pop())

	lastx = 0
	lasty = 0
        # add reverse-tree path into path directly, from connecting vertice to goal
        while (rkeyID != rtree.GetRootId()):
            # ID in dict edge is indexes of [x,y] position in 2D list vertices
            rvalueID = rtree.edges[rkeyID] 
            plan.append(rtree.vertices[rvalueID]) 
	    pathlength = pathlength + pow(pow((rtree.vertices[rvalueID][0]-lastx), 2) + pow((rtree.vertices[rvalueID][1]-lasty), 2), 0.5)
	    lastx = rtree.vertices[rvalueID][0]
            lasty = rtree.vertices[rvalueID][1]
	    rkeyID = rvalueID

        print "points are: ", plan 
        print "path length is: ", pathlength

        return plan
