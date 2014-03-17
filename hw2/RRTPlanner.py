import numpy
from RRTTree import RRTTree
from SimpleRobot import SimpleRobot
from SimpleEnvironment import SimpleEnvironment

class RRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize

    def Plan(self, start_config, goal_config, epsilon = 0.001):
        
        tree = RRTTree(self.planning_env, start_config)

        plan = []
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space - (jane) which is 2 in this case? 
        
        plan.append(start_config)
        plan.append(goal_config)
        
        # starting id is 0
        vid = vid_last = 0
        q_new = start_config
        sampleGoal = 5
        count = 0
        thr = 0.1
        dist = 100
        # if q_new is very near to the goal, break out of the loop 
        # don't worry about goal not included in vertices or edges: start and goal are in path 
        while (dist > thr) : 
            # Pick a sample q_rand in space C, occasionally, sample the goal 
            if count == sampleGoal:
                q_rand = goal_config 
            else: q_rand = self.planning_env.GenerateRandomConfiguration()
            count += 1

            # find q_rand's nearest milestone q in the tree:    
            q_near = tree.GetNearestVertex(q_rand)
            # extend a step from q to q_new towards q_rand 
            q_new = self.planning_env.Extend(q_near, q_rand)
            # add q_new to the tree
            if q_new != None:
                vid = tree.AddVertex(str(q_new))
                tree.AddEdge(vid_last, vid)
                # call whenever an edge is added on the tree 
                PlotEdge() 
                vid_last = vid
                # get out of the loop if q_new is very near to q_goal
                dist = ComputeDistance(q_new, q_goal)    

        # get the goal ID in edges 
            goalID = max(tree.edges.keys())
            keyID = goalID
        # now q_new is goal, find way back to start 
        while (vid != 0):
            # ID in dict edge is indexes of [x,y] position in 2D list vertices
            valueID = tree.edges(keyID) 
            path.append(tree.vertices[valueID]) 
            keyID = valueID 
        return plan
