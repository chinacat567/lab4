#!/usr/bin/env python
import heapq, math, rospy, tf
from collections import defaultdict, deque
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class Astar:
    #dimensions 20x18

    grid  =     [[0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0],
                [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0],
                [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                [0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                [0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0],
                [0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0],
                [0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0],
                [0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1],
                [0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1],
                [0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1],
                [0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0],
                [0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0],
                [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0],
                [0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0],
                [0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0],
                [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1]]

    direc = {(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (-1, 1), (-1, -1), (1, -1)}


    def __init__(self, source = (-8.0, -2.0), goal = (4.5, 9.0)):
        self.goal = self.convert(goal)
        self.source = self.convert(source)
        self.cameFrom = defaultdict(tuple)
        self.gscore = defaultdict(float('inf'))
        self.fscore = defaultdict(float('inf'))
        self.m = len(self.map)
        self.n = len(self.map[0])

    #convert (x, y) robot pos  to (r, c) in grid map
    def convert(self, point):
        x, y = point
        (round((self.m // 2) - y), round((self.n // 2) + x))

    def h(self, curr):
        goalX, goalY = self.goal
        x, y = curr
        return math.sqrt((x - goalX)**2  + (goalY- y)**2)
    def build_path(self, curr):
        total_path = deque([curr])
        while curr in self.cameFrom:
            curr = self.cameFrom[curr]
            total_path.append(curr)
        return total_path
    def astar(self):
        self.gscore[self.start] = 0
        self.fscore[self.start] = self.h(self.start)
        openset = [(self.fscore[self.start], self.start)]
        visited = set()
        visited.add(self.start)
        while openset:
            _, curr = openset.heappop()
            if curr == self.goal:
                return self.reconstruct_path(self.goal)
            x, y = curr
            for d_x, d_y in self.direc:
                a, b = x + d_x, y + d_y
                if 0 <= a < self.m and 0 <= b < self.n and (a, b) not in visited and self.grid[a][b] == 0:
                    tentative_gScore = self.gscore[curr] + 1
                    if tentative_gScore < self.gscore[(a, b)]
                        visited.add((a, b))
                        self.gscore[(a, b)] = tentative_gScore
                        self.fscore[(a, b)] = tentative_gScore + self.h((a, b))
                        heapq.heappush(openset, self.fscore[(a, b)])
        rospy.logerr("ASTAR path planning failed \n")




class Robot:
    def __init__(self):
        self.publisher = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        self.subs = rospy.Subscriber("/base_pose_ground_truth", Odometry,self.robot_motion)
        self.isdone = False
        self.path = None
        self.idx = 0
        self.rotate = False


    def _make_twist(self, linear=[0, 0, 0], angular=[0, 0, 0]):
        t = Twist()
        t.linear.x = linear[0]
        t.linear.y = linear[1]
        t.linear.z = linear[2]

        t.angular.x = angular[0]
        t.angular.y = angular[1]
        t.angular.z = angular[2]
        return t
    def robot_motion(self,data,args):
        if not args["isDone"]:
            path=args['path']
            #rospy.logerr(str(path))
            nextIndex=args['nextIndex']
            pub=args['pub']
            trans=data.pose.pose.position
            rot=data.pose.pose.orientation
            robotW=tf.transformations.euler_from_quaternion([rot.x,rot.y,rot.z,rot.w])[2]
            robotposx=trans.x
            robotposy=trans.y
            goalx=path[nextIndex][0]-9+0.7
            goaly=10-path[nextIndex][1]-0.7
            theta=math.atan2(goaly-robotposy,goalx-robotposx)
            error = theta - robotW
            #rospy.logerr("Goal "+str((goalx,goaly)))
            #rospy.logerr("robotpos "+str((robotposx,robotposy)))
            if args['isRotation']:
                if math.fabs(error) > math.radians(6):
                    # rotate
            #        rospy.logerr("publish rot theta  robotw"+str((math.degrees(theta),math.degrees(robotW))))
                    if error < 0:
                        error += math.pi * 2
                    elif error > math.pi * 2:
                        error -= math.pi * 2
                    if error > math.pi:
                        pub.publish(self._make_twist(angular=[0,0,-0.75]))
                    else:
                        pub.publish(self._make_twist(angular=[0,0,0.75]))
                else:
            #        rospy.logerr("set rot false")
                    args["isRotation"]=False
            else:
                error=math.sqrt((goalx-robotposx)**2+(goaly-robotposy)**2)
                if error> 0.5:
                    #rospy.logerr("publish trans -"+str(error))
                    pub.publish(self._make_twist(linear=[0.75,0,0]))
                else:
            #        rospy.logerr("set rot True")
                    args["isRotation"]=True
                    if nextIndex+1<len(path):
                        args['nextIndex']+=1
                    else:
                        args['isDone']=True





if __name__ == '__main__':
#   try:

    #rospy.logerr( astar(map, (1,12), (13,1)))
    rospy.init_node('astar')
    if rospy.has_param('/goalx') and rospy.has_param('/goaly'):
        planner = Astar(rospy.get_param('/goalx'))
    robot = Robot()
    while not rospy.is_shutdown():
        if self.isdone == True:




    # start = (1,12)
    # goal = (13,1)
    # goalx,goaly = goal

    # args={}
    # args['isDone']=False
    # args["isRotation"]=True
    # args['nextIndex']=0
    # args['pub']= rospy.Publisher('/cmd_vel',Twist,queue_size=1)
    # args['path']=astar(map, start, goal)
    # robot_pos_pub = rospy.Subscriber("/base_pose_ground_truth", Odometry,robot._robot_move,args )
    # while not rospy.is_shutdown():
    #     if args['isDone']:

    #         if rospy.has_param("/goalx") and rospy.has_param("/goaly"):
    #                 goalx,goaly=rospy.get_param("/goalx"),rospy.get_param("/goaly")
    #                 rospy.delete_param("/goalx")
    #                 rospy.delete_param("/goaly")
    #                 nstart = goal
    #                 goal = (round(goalx+9),round(10-goaly))
    #                 args['path']=astar(map, nstart, goal)
    #                 #rospy.logerr("new path "+str(args["path"]))
    #                 args["isRotation"]=True
    #                 args['nextIndex']=0
    #                 args['isDone']=False

    #     rate = rospy.Rate(2)
    #     rospy.sleep(1)













# cameFrom[neighbor] := current
#                 gScore[neighbor] := tentative_gScore
#                 fScore[neighbor] := tentative_gScore + h(neighbor)
#                 if neighbor not in openSet
#                     openSet.add(neighbor)






# function reconstruct_path(cameFrom, current)
#     total_path := {current}
#     while current in cameFrom.Keys:
#         current := cameFrom[current]
#         total_path.prepend(current)
#     return total_path

# // A* finds a path from start to goal.
# // h is the heuristic function. h(n) estimates the cost to reach goal from node n.
# function A_Star(start, goal, h)
#     // The set of discovered nodes that may need to be (re-)expanded.
#     // Initially, only the start node is known.
#     // This is usually implemented as a min-heap or priority queue rather than a hash-set.
#     openSet := {start}

#     // For node n, cameFrom[n] is the node immediately preceding it on the cheapest path from start
#     // to n currently known.
#     cameFrom := an empty map

#     // For node n, gScore[n] is the cost of the cheapest path from start to n currently known.
#     gScore := map with default value of Infinity
#     gScore[start] := 0

#     // For node n, fScore[n] := gScore[n] + h(n). fScore[n] represents our current best guess as to
#     // how short a path from start to finish can be if it goes through n.
#     fScore := map with default value of Infinity
#     fScore[start] := h(start)

#     while openSet is not empty
#         // This operation can occur in O(1) time if openSet is a min-heap or a priority queue
#         current := the node in openSet having the lowest fScore[] value
#         if current = goal
#             return reconstruct_path(cameFrom, current)

#         openSet.Remove(current)
#         for each neighbor of current
#             // d(current,neighbor) is the weight of the edge from current to neighbor
#             // tentative_gScore is the distance from start to the neighbor through current
#             tentative_gScore := gScore[current] + d(current, neighbor)
#             if tentative_gScore < gScore[neighbor]
#                 // This path to neighbor is better than any previous one. Record it!
#                 cameFrom[neighbor] := current
#                 gScore[neighbor] := tentative_gScore
#                 fScore[neighbor] := tentative_gScore + h(neighbor)
#                 if neighbor not in openSet
#                     openSet.add(neighbor)

#     // Open set is empty but goal was never reached
#     return failure