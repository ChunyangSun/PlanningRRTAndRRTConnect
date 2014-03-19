import numpy, operator
from RRTPlanner import RRTTree
import IPython

class RRTConnectPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        
    #  RRT Connect Planner Implementation
    #  returns an array
    #  of dimension k x n where k is the number of waypoints
    #  and n is the dimension of the robots configuration space
    def Plan(self, start_config, goal_config, epsilon = 0.001):
        #set up visualizer
        if self.visualize: 
            self.planning_env.InitializePlot(goal_config)

        ftree = RRTTree(self.planning_env, start_config)
        rtree = RRTTree(self.planning_env, goal_config)

        fq_new = start_config
        count = 0
        dist = 9999

        while (dist > epsilon) : 
            # Pick a sample q_rand in space C, sample the goal every 5 cycles 
            q_rand  = self.planning_env.GenerateRandomConfiguration() if (count % 5) else goal_config
            count += 1

            # find q_rand's nearest milestone q in both trees:    
            [fvid_start, fq_near] = ftree.GetNearestVertex(q_rand)   
            [rvid_start, rq_near] = rtree.GetNearestVertex(q_rand)

            #find the nearest tree
            if(self.planning_env.ComputeDistance(fq_near, q_rand) < self.planning_env.ComputeDistance(rq_near, q_rand)) or not (count % 5):
                #attach the node to the forward tree
                fq_new = self.planning_env.Extend(fq_near, q_rand)
                fvid_end = ftree.AddVertex(fq_new)
                ftree.AddEdge(fvid_start, fvid_end)

                #find the nearest reverse tree node
                [rvid_end, rq_new] = rtree.GetNearestVertex(fq_new)

                # draw the new edge 
                if self.visualize: 
                    self.planning_env.PlotEdge(fq_near, fq_new)
            else:
                #attach the node to the reverse tree
                rq_new = self.planning_env.Extend(rq_near, fq_new)
                rvid_end = rtree.AddVertex(rq_new)
                rtree.AddEdge(rvid_start, rvid_end)

                #find the nearest forward tree node
                [fvid_end, fq_new] = ftree.GetNearestVertex(rq_new)  

                # draw the new edge 
                if self.visualize: 
                    self.planning_env.PlotEdge(rq_near, rq_new) 

            #update the distance metric
            dist = self.planning_env.ComputeDistance(fq_new, rq_new)

        # final goal IDs are fvid_end, rvid_end
        plan = []

        # add front-tree path into the plan
        currVertex = fvid_end
        while (currVertex != ftree.GetRootId()):
            currVertex = ftree.edges[currVertex] 
            plan.insert(0, ftree.vertices[currVertex]) 

        # add reverse-tree into the plan
        currVertex = rvid_end
        while (currVertex != rtree.GetRootId()):
            currVertex = rtree.edges[currVertex ] 
            plan.append(rtree.vertices[currVertex]) 

        # shorten path and return 
        return self.planning_env.ShortenPath(plan)
