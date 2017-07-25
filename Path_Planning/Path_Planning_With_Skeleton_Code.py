#!/usr/bin/env python

from copy import deepcopy
import math
import numpy
import random
from threading import Thread, Lock
import sys
import matplotlib.pyplot as plt

import actionlib
import control_msgs.msg
import geometry_msgs.msg
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.srv
import rospy
import sensor_msgs.msg
import tf
import trajectory_msgs.msg
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import Marker
import datetime

#Robotics Homework 3
#Fall 2015

#Xiangbing Ji
#Blair Kania
#Ori Kedar


#Use_RRT=True for RRT and Use_RRT=False for PRM 

Use_RRT=True

#Creates class to store nodes on path
class node(object):
	#self.neighbour is a dictionary, keys in the dict is the index of the neighbors of a given node, self.neighbour[key] is the distance between the neighbor and the node
    def __init__(self,idx,q_value):
        self.idx=idx 
        self.q_value=q_value
        self.neighbour={}
    def add_neighbour(self,nbr_idx,distance):
        self.neighbour[nbr_idx]=distance

#Creates class to store nodes in a graph
class graph(object):
    def __init__(self):
        self.number_nodes=0
        self.adjList={}
   #idx, the idx of a node
    def add_node(self,idx,q_value):
        if idx not in self.adjList:
            self.number_nodes=self.number_nodes+1
            new_node=node(idx,q_value)
            self.adjList[idx]=new_node
   #node1 and node2 are two indexs of two nodes in one edge
    def add_edge(self,node1,node2,length):
        if node1 not in self.adjList and node2 not in self.adjList:
           print 'error,edge nodes must be already within the graph'        
        else:
           self.adjList[node1].add_neighbour(node2,length)
           self.adjList[node2].add_neighbour(node1,length)

def convert_to_message(T):
    t = geometry_msgs.msg.Pose()
    position = tf.transformations.translation_from_matrix(T)
    orientation = tf.transformations.quaternion_from_matrix(T)
    t.position.x = position[0]
    t.position.y = position[1]
    t.position.z = position[2]
    t.orientation.x = orientation[0]
    t.orientation.y = orientation[1]
    t.orientation.z = orientation[2]
    t.orientation.w = orientation[3]        
    return t

def convert_from_message(msg):
    R = tf.transformations.quaternion_matrix((msg.orientation.x,
                                              msg.orientation.y,
                                              msg.orientation.z,
                                              msg.orientation.w))
    T = tf.transformations.translation_matrix((msg.position.x, 
                                               msg.position.y, 
                                               msg.position.z))
    return numpy.dot(T,R)

class RRTNode(object):
    def __init__(self):
        self.q=numpy.zeros(7)
        self.parent = None

class MoveArm(object):
    def __init__(self):
        print "HW3 initializing..."
        # Prepare the mutex for synchronization
        self.mutex = Lock()

        # min and max joint values are not read in Python urdf, so we must hard-code them here
        self.q_min = []
        self.q_max = []
        self.q_min.append(-1.700);self.q_max.append(1.700)
        self.q_min.append(-2.147);self.q_max.append(1.047)
        self.q_min.append(-3.054);self.q_max.append(3.054)
        self.q_min.append(-0.050);self.q_max.append(2.618)
        self.q_min.append(-3.059);self.q_max.append(3.059)
        self.q_min.append(-1.570);self.q_max.append(2.094)
        self.q_min.append(-3.059);self.q_max.append(3.059)

        # Subscribes to information about what the current joint values are.
        rospy.Subscriber("robot/joint_states", sensor_msgs.msg.JointState, self.joint_states_callback)

        # Initialize variables
        self.q_current = []
        self.joint_state = sensor_msgs.msg.JointState()

        # Create interactive marker
        self.init_marker()

        # Connect to trajectory execution action
        self.trajectory_client = actionlib.SimpleActionClient('/robot/limb/left/follow_joint_trajectory', 
                                                              control_msgs.msg.FollowJointTrajectoryAction)
        self.trajectory_client.wait_for_server()
        print "Joint trajectory client connected"

        # Wait for moveit IK service
        rospy.wait_for_service("compute_ik")
        self.ik_service = rospy.ServiceProxy('compute_ik',  moveit_msgs.srv.GetPositionIK)
        print "IK service ready"

        # Wait for validity check service
        rospy.wait_for_service("check_state_validity")
        self.state_valid_service = rospy.ServiceProxy('check_state_validity',  
                                                      moveit_msgs.srv.GetStateValidity)
        print "State validity service ready"

        # Initialize MoveIt
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("left_arm") 
        print "MoveIt! interface ready"

        # How finely to sample each joint
        self.q_sample = [0.1, 0.1, 0.2, 0.2, 0.4, 0.4, 0.4]
        self.joint_names = ["left_s0", "left_s1",
                            "left_e0", "left_e1",
                            "left_w0", "left_w1","left_w2"]

        # Options
        self.subsample_trajectory = True
        self.spline_timing = True
        self.show_plots = False

        print "Initialization done."


    def control_marker_feedback(self, feedback):
        pass

    def get_joint_val(self, joint_state, name):
        if name not in joint_state.name:
            print "ERROR: joint name not found"
            return 0
        i = joint_state.name.index(name)
        return joint_state.position[i]

    def set_joint_val(self, joint_state, q, name):
        if name not in joint_state.name:
            print "ERROR: joint name not found"
        i = joint_state.name.index(name)
        joint_state.position[i] = q

    """ Given a complete joint_state data structure, this function finds the values for 
    a particular set of joints in a particular order (in our case, the left arm joints ordered
    from proximal to distal) and returns a list q[] containing just those values.
    """
    def q_from_joint_state(self, joint_state):
        q = []
        q.append(self.get_joint_val(joint_state, "left_s0"))
        q.append(self.get_joint_val(joint_state, "left_s1"))
        q.append(self.get_joint_val(joint_state, "left_e0"))
        q.append(self.get_joint_val(joint_state, "left_e1"))
        q.append(self.get_joint_val(joint_state, "left_w0"))
        q.append(self.get_joint_val(joint_state, "left_w1"))
        q.append(self.get_joint_val(joint_state, "left_w2"))
        return q

    """ Given a list q[] of joint values and an already populated joint_state, this function assumes 
    that the passed in values are for a particular set of joints in a particular order (in our case,
    the left arm joints ordered from proximal to distal) and edits the joint_state data structure to
    set the values to the ones passed in.
    """
    def joint_state_from_q(self, joint_state, q):
        self.set_joint_val(joint_state, q[0], "left_s0")
        self.set_joint_val(joint_state, q[1], "left_s1")
        self.set_joint_val(joint_state, q[2], "left_e0")
        self.set_joint_val(joint_state, q[3], "left_e1")
        self.set_joint_val(joint_state, q[4], "left_w0")
        self.set_joint_val(joint_state, q[5], "left_w1")
        self.set_joint_val(joint_state, q[6], "left_w2")        

    """ Creates simple timing information for a trajectory, where each point has velocity
    and acceleration 0 for all joints, and all segments take the same amount of time
    to execute.
    """
    def compute_simple_timing(self, q_list, time_per_segment):
        v_list = [numpy.zeros(7) for i in range(0,len(q_list))]
        a_list = [numpy.zeros(7) for i in range(0,len(q_list))]
        t = [i*time_per_segment for i in range(0,len(q_list))]
        return v_list, a_list, t

    """ This function will perform IK for a given transform T of the end-effector. It returs a list q[]
    of 7 values, which are the result positions for the 7 joints of the left arm, ordered from proximal
    to distal. If no IK solution is found, it returns an empy list.
    """
    def IK(self, T_goal):
        req = moveit_msgs.srv.GetPositionIKRequest()
        req.ik_request.group_name = "left_arm"
        req.ik_request.robot_state = moveit_msgs.msg.RobotState()
        req.ik_request.robot_state.joint_state = self.joint_state
        req.ik_request.avoid_collisions = True
        req.ik_request.pose_stamped = geometry_msgs.msg.PoseStamped()
        req.ik_request.pose_stamped.header.frame_id = "base"
        req.ik_request.pose_stamped.header.stamp = rospy.get_rostime()
        req.ik_request.pose_stamped.pose = convert_to_message(T_goal)
        req.ik_request.timeout = rospy.Duration(3.0)
        res = self.ik_service(req)
        q = []
        if res.error_code.val == res.error_code.SUCCESS:
            q = self.q_from_joint_state(res.solution.joint_state)
        return q

    """ This function checks if a set of joint angles q[] creates a valid state, or one that is free
    of collisions. The values in q[] are assumed to be values for the joints of the left arm, ordered
    from proximal to distal. 
    """
    def is_state_valid(self, q):
        req = moveit_msgs.srv.GetStateValidityRequest()
        req.group_name = "left_arm"
        current_joint_state = deepcopy(self.joint_state)
        current_joint_state.position = list(current_joint_state.position)
        self.joint_state_from_q(current_joint_state, q)
        req.robot_state = moveit_msgs.msg.RobotState()
        req.robot_state.joint_state = current_joint_state
        res = self.state_valid_service(req)
        return res.valid

    # This function will plot the position, velocity and acceleration of a joint
    # based on the polynomial coefficients of each segment that makes up the 
    # trajectory.
    # Arguments:
    # - num_segments: the number of segments in the trajectory
    # - coefficients: the coefficients of a cubic polynomial for each segment, arranged
    #   as follows [a_1, b_1, c_1, d_1, ..., a_n, b_n, c_n, d_n], where n is the number
    #   of segments
    # - time_per_segment: the time (in seconds) allocated to each segment.
    # This function will display three plots. Execution will continue only after all 
    # plot windows have been closed.
    def plot_trajectory(self, num_segments, coeffs, time_per_segment):
        resolution = 1.0e-2
        assert(num_segments*4 == len(coeffs))
        t_vec = []
        q_vec = []
        a_vec = []
        v_vec = []
        for i in range(0,num_segments):
            t=0
            while t<time_per_segment:
                q,a,v = self.sample_polynomial(coeffs,i,t+i)
                t_vec.append(t+i*time_per_segment)
                q_vec.append(q)
                a_vec.append(a)
                v_vec.append(v)
                t = t+resolution
        self.plot_series(t_vec,q_vec,"Position")
        self.plot_series(t_vec,v_vec,"Velocity")
        self.plot_series(t_vec,a_vec,"Acceleration")
        plt.show()

    """ This is the main function to be filled in for HW3.
    Parameters:
    - q_start: the start configuration for the arm
    - q_goal: the goal configuration for the arm
    - q_min and q_max: the min and max values for all the joints in the arm.
    All the above parameters are arrays. Each will have 7 elements, one for each joint in the arm.
    These values correspond to the joints of the arm ordered from proximal (closer to the body) to 
    distal (further from the body). 

    The function must return a trajectory as a tuple (q_list,v_list,a_list,t).
    If the trajectory has n points, then q_list, v_list and a_list must all have n entries. Each
    entry must be an array of size 7, specifying the position, velocity and acceleration for each joint.

    For example, the i-th point of the trajectory is defined by:
    - q_list[i]: an array of 7 numbers specifying position for all joints at trajectory point i
    - v_list[i]: an array of 7 numbers specifying velocity for all joints at trajectory point i
    - a_list[i]: an array of 7 numbers specifying acceleration for all joints at trajectory point i
    Note that q_list, v_list and a_list are all lists of arrays. 
    For example, q_list[i][j] will be the position of the j-th joint (0<j<7) at trajectory point i 
    (0 < i < n).

    For example, a trajectory with just 2 points, starting from all joints at position 0 and 
    ending with all joints at position 1, might look like this:

    q_list=[ numpy.array([0, 0, 0, 0, 0, 0, 0]),
             numpy.array([1, 1, 1, 1, 1, 1, 1]) ]
    v_list=[ numpy.array([0, 0, 0, 0, 0, 0, 0]),
             numpy.array([0, 0, 0, 0, 0, 0, 0]) ]
    a_list=[ numpy.array([0, 0, 0, 0, 0, 0, 0]),
             numpy.array([0, 0, 0, 0, 0, 0, 0]) ]
             
    Note that the trajectory should always begin from the current configuration of the robot.
    Hence, the first entry in q_list should always be equal to q_start. 

    In addition, t must be a list with n entries (where n is the number of points in the trajectory).
    For the i-th trajectory point, t[i] must specify when this point should be reached, relative to
    the start of the trajectory. As a result t[0] should always be 0. For the previous example, if we
    want the second point to be reached 10 seconds after starting the trajectory, we can use:

    t=[0,10]

    When you are done computing all of these, return them using

    return q_list,v_list,a_list,t

    In addition, you can use the function self.is_state_valid(q_test) to test if the joint positions 
    in a given array q_test create a valid (collision-free) state. q_test will be expected to 
    contain 7 elements, each representing a joint position, in the same order as q_start and q_goal.
    """ 

#Function to Execute RRT    
    def RRT(self,q_min, q_max, q_goal, q_start,G):
	q_start=numpy.array(q_start)
	q_goal=numpy.array(q_goal)        
	G.add_node(0,q_start)
        aaa=True
	while G.number_nodes<2000 and aaa==True: #Finish when you've added the goal
		print "Number Nodes"
                print G.number_nodes
	#Generate random point
		r=numpy.array([numpy.random.uniform(q_min[0],q_max[0]),
		numpy.random.uniform(q_min[1],q_max[1]),
		numpy.random.uniform(q_min[2],q_max[2]),
		numpy.random.uniform(q_min[3],q_max[3]),
		numpy.random.uniform(q_min[4],q_max[4]),
		numpy.random.uniform(q_min[5],q_max[5]),
		numpy.random.uniform(q_min[6],q_max[6])
		])
		D=[]
	#Find Closest point
		for k in range(G.number_nodes):

			dist=math.sqrt(sum((G.adjList[k].q_value-r)**2))
			D.append(dist)

		#Creates a vector to closest point
		closest=G.adjList[numpy.argmin(D)]
		vec=numpy.array(r-closest.q_value)
		norm_vec=numpy.linalg.norm(vec)
		norm_vec=numpy.divide(vec,norm_vec)
		j=.1 
		while j<=.5:
			q_check=closest.q_value+j*norm_vec
			test=self.is_state_valid(q_check)			
			if test==False:
				break
			if test==True and j==.5:
				G.add_node(G.number_nodes, q_check)
                                G.add_edge(closest.idx,G.number_nodes-1,0.5)
				if self.can_see(G.adjList[G.number_nodes-1].q_value,q_goal)==True:
					G.add_node(G.number_nodes,q_goal)
                                        G.add_edge(G.number_nodes-2,G.number_nodes-1,self.distance(G.number_nodes-2,G.number_nodes-1,G))
                                        aaa=False
			j=j+.1
	return aaa
               
    #Function for running PRM 		
    def PRM(self,q_min,q_max,q_goal,q_start,G):
        q_start=numpy.array(q_start)
        q_goal=numpy.array(q_goal)        
        start_time=datetime.datetime.now()
        time_elapsed=0
        G.add_node(0,q_start)
        G.add_node(1,q_goal)
        while time_elapsed<=60:

            #Generate random point
            random_point=numpy.array([numpy.random.uniform(q_min[0],q_max[0]),
            numpy.random.uniform(q_min[1],q_max[1]),
            numpy.random.uniform(q_min[2],q_max[2]),
            numpy.random.uniform(q_min[3],q_max[3]),
            numpy.random.uniform(q_min[4],q_max[4]),
            numpy.random.uniform(q_min[5],q_max[5]),
            numpy.random.uniform(q_min[6],q_max[6])
            ])
            if self.is_state_valid(random_point):
                G.add_node(G.number_nodes,random_point)
                for i in range(0,G.number_nodes-1):
                    if self.can_see(random_point,G.adjList[i].q_value):
                        G.add_edge(i,G.number_nodes-1,self.distance(i,G.number_nodes-1,G))
                X,Y,Z=self.dijkstra(G,1)
                print "exist a path?"
                print X
                print "path"
                print Y
                print "pathvalue"
                print Z
            current_time=datetime.datetime.now()
            time_elapsed=(current_time.hour-start_time.hour)*3600+(current_time.minute-start_time.minute)*60+(current_time.second-start_time.second)
            print "time_elapsed"
            print time_elapsed
        print "number of nodes"
        print G.number_nodes

    def dijkstra(self,graph,goal_index):
            exist_path=True
	    visited=[]
	    size=graph.number_nodes
	    q_n=1000*numpy.ones((size,1));
	    q_n[0]=0
	    current_node=0 # set the current node as the starting node
	# compute the shortest path
	    while len(visited)<=size:
		lowest_path=1000
		for i in range(0,size):
		    if i not in visited and q_n[i]<=lowest_path:
		       current_node=i
		       lowest_path=q_n[i]
		visited.append(current_node) # find the current node,mark it as visited
		neighbors=graph.adjList[current_node].neighbour # find the neighbors of the current node
		for neighbor in neighbors:
		    q_n[neighbor]=min(q_n[neighbor],q_n[current_node]+graph.adjList[current_node].neighbour[neighbor])
  
	# finding the shortest path by going from the goal back to the start
	    current_node=goal_index # set the current node as the goal node
	    path=[goal_index]

	    while current_node!=0 and exist_path==True: 
		neighbors=graph.adjList[current_node].neighbour
		#lowest_path=1000
                exist_path=False;
		for neighbor in neighbors:
		    if (q_n[neighbor]+neighbors[neighbor]) == q_n[current_node]:
		        current_node=neighbor
                        exist_path=True;
		path.append(current_node)
	    path.reverse()
	    return (exist_path,path,q_n[goal_index])

    #Checks distance between two points
    def distance(self,node1,node2,graph):
	    q1=graph.adjList[node1].q_value
	    q2=graph.adjList[node2].q_value
	    distance=0
	    dim=len(q1)
	    for i in range(0,dim):
		distance=distance+(q1[i]-q2[i])**2
	    distance=math.sqrt(distance)
	    return distance
    def shortcutting(self,path,graph):
        size=graph.number_nodes
        shortcutting_path=[0]
        current_node=0
        while current_node!=size-1:
           # if self.can_see(graph.adjList[current_node].q_value,graph.adjList[size-1].q_value):
 # if we can see the final node from the current_node, break t
               # shortcutting_path.append(size-1)
                #break
            current_largest_distance=0
            for node in path:
                if path.index(node)>path.index(current_node) and self.can_see(graph.adjList[current_node].q_value,graph.adjList[node].q_value):
                    distance=self.distance(current_node,node,graph)
                    if distance>current_largest_distance:
                        farthest_node=node
                        current_largest_distance=distance
            current_node=farthest_node
            shortcutting_path.append(current_node)
                
        return shortcutting_path

    #Checks if two points can see eachother
    def can_see(self,q1,goal):
	l=.1
	vec=goal-q1
	norm_vec=numpy.linalg.norm(vec)
	norm_vec=numpy.divide(vec,norm_vec)	
	added_dist=0
        test_2=True
	while numpy.linalg.norm(added_dist)<numpy.linalg.norm(vec):

		added_dist=l*norm_vec
		q_check=q1+added_dist
		test_2=self.is_state_valid(q_check)

		if test_2==False:
			break
		if test_2==True:
			l=l+.1

        return test_2

    #Trajectory computation: Interpolation using splines. This function will return qlist, vlist, alist, and t.
    def TrajectoryComputation(self, resample): 
	q_list = numpy.zeros((len(resample),7))
	v_list = numpy.zeros((len(resample),7))
	a_list = numpy.zeros((len(resample),7))
	
	coefficients=numpy.zeros((7,4*len(resample)-4))

        #Builds Matrix for each joint
	i=0
	while i<7:
		Matrix = numpy.zeros((4*len(resample)-4, 4*len(resample)-4))
		resultvect = numpy.zeros(4*len(resample)-4)
		polynomial=numpy.zeros((len(resample)-1,4))

		#q(t)
		b=2
		s=0
		t=0
		primaryrun=True
		while b<2*len(resample):
			Matrix[b][s]=numpy.power(t,3)
			Matrix[b][s+1]=numpy.power(t,2)
			Matrix[b][s+2]=t
			Matrix[b][s+3]=1
			if primaryrun is False:
				s=s+4
				primaryrun=True
			else:
				t=t+1
				primaryrun=False
			b=b+1
		
		#Velocity 0 at start and end
		Matrix[0][2]=1
		Matrix[1][4*len(resample)-6]=1
		Matrix[1][4*len(resample)-7]=(2*(len(resample)-1))
		Matrix[1][4*len(resample)-8]=(3*numpy.power(len(resample)-1,2))

		#Continuous Velocity
		b=2*len(resample)
		s=0
		t=1
		while b<3*len(resample)-2:
			Matrix[b][0+s]=3*numpy.power(t,2)
			Matrix[b][4+s]=-1*Matrix[b][0+s]
			Matrix[b][1+s]=2*t
			Matrix[b][5+s]=-1*Matrix[b][1+s]
			Matrix[b][2+s]=1
			Matrix[b][6+s]=-1*Matrix[b][2+s]
			b=b+1
			s=s+4
			t=t+1

		#Contnuous Acceleration
		b=3*len(resample)-2
		s=0
		t=1
		while b<4*len(resample)-4:
			Matrix[b][0+s]=6*t
			Matrix[b][4+s]=-1*Matrix[b][0+s]
			Matrix[b][1+s]=2
			Matrix[b][5+s]=-1*Matrix[b][1+s]
			b=b+1
			s=s+4
			t=t+1
			
		#Result Vector	
		resultvect[2]=resample[0][i]
		resultvect[2*len(resample)-1]=resample[len(resample)-1][i]
		j=3
		b=1
		firstTime=True
		while j<2*len(resample)-1:	
			resultvect[j]=resample[b][i]
			if firstTime is False:
				b=b+1
				firstTime=True
			else:
				firstTime=False
			j=j+1

		#Coefficients
		coef=[]
		coef=numpy.dot(numpy.linalg.inv(Matrix), resultvect)
		#print "coef"
		#print coef
		coefficients[i,:]=coef
		
		b=0
		c=0
		while b<len(resample)-1:
			j=0
			while j<4:
				polynomial[b][j]=coef[c]
				j=j+1
				c=c+1
			b=b+1

		b=0
		while b<len(resample):
			if b<2:
				q_list[b][i]=polynomial[0][0]*numpy.power(b,3)+polynomial[0][1]*numpy.power(b,2)+polynomial[0][2]*b+polynomial[0][3]
				v_list[b][i]=3*polynomial[0][0]*numpy.power(b,2)+2*polynomial[0][1]*b+polynomial[0][2]
				a_list[b][i]=6*polynomial[0][0]*b+2*polynomial[0][1]
			else:
				q_list[b][i]=polynomial[b-1][0]*numpy.power(b,3)+polynomial[b-1][1]*numpy.power(b,2)+polynomial[b-1][2]*b+polynomial[b-1][3]
				v_list[b][i]=3*polynomial[b-1][0]*numpy.power(b,2)+2*polynomial[b-1][1]*b+polynomial[b-1][2]
				a_list[b][i]=6*polynomial[b-1][0]*b+2*polynomial[b-1][1]
			b=b+1
		i=i+1

	#time list
	i=0
	t = []
	while i<len(resample):
		t.append(i)
		i=i+1

	return coefficients, q_list, v_list, a_list, t

    #Function to resample shortcutted path
    def resample(self, shortcut_list, G):
	resample=[]
	resample.append(G.adjList[0].q_value)
	i=0
	while i<(len(shortcut_list)-1):
		vector=numpy.array(G.adjList[shortcut_list[i+1]].q_value - G.adjList[shortcut_list[i]].q_value)		
		mag=numpy.linalg.norm(vector)
		seg_len=mag/math.floor(mag/.5)
		add_dist=0
		unit_vector=numpy.divide(vector,mag)
		while add_dist<mag:
			add_dist+=seg_len
			sample=G.adjList[shortcut_list[i]].q_value+add_dist*unit_vector
			resample.append(sample)
		resample.append(G.adjList[shortcut_list[i+1]].q_value)
		i+=1
	return resample

    #Function to create list of joint values
    def make_list(self, shortcutting_list, G):
	number_path=[]
	i=0		
	while i<len(shortcutting_list):
		number_path.append(G.adjList[shortcutting_list[i]].q_value)
		i+=1
	return number_path
    
    #execute motion planning
    def motion_plan(self, q_start, q_goal, q_min, q_max):
        
        #Creates graph (See classes created on line 32 and 42)
        G=graph()
        find_goal=False
        exist_path=True
        global Use_RRT

	#for use of RRT
        if Use_RRT==True:
            if self.can_see(q_start,q_goal):
                G.add_node(0,q_start)
                G.add_node(1,q_goal)
                G.add_edge(0,1,self.distance(0,1,G))
                shortcutting_path=[0,1]  
            else:
                find_goal=self.RRT(q_min, q_max, q_goal, q_start,G)
                A,path,x=self.dijkstra(G,G.number_nodes-1)
                shortcutting_path=self.shortcutting(path,G)
                print "shortcutting_path"
                print shortcutting_path    
	
	#for use of PRM  
        else:
            if self.can_see(q_start,q_goal):
                G.add_node(0,q_start)
                G.add_node(1,q_goal)
                G.add_edge(0,1,self.distance(0,1,G))
                shortcutting_path=[0,1] 
            else:
                 self.PRM(q_min, q_max, q_goal, q_start,G)
                 exist_path,path,path_value=self.dijkstra(G,1)
	         print "path"
                 print path
                 print "shortest path value"
                 print path_value
                 shortcutting_path=path
 	
	#Execute trajectory computation
	if find_goal==False and exist_path==True:
		resample=self.resample(shortcutting_path, G)
		coefficients, q_list, v_list, a_list, t=self.TrajectoryComputation(resample)
		self.plot_trajectory(len(resample)-1, coefficients[0,:], 1)
	if find_goal==True:
	   q_list, v_list, a_list, t=[],[],[],0
           print "too many RRT nodes" 
        if exist_path==False:
	   q_list, v_list, a_list, t=[],[],[],0
           print "can't find a path for PRM" 

        return q_list, v_list, a_list, t
       

    def project_plan(self, q_start, q_goal, q_min, q_max):
        q_list, v_list, a_list, t = self.motion_plan(q_start, q_goal, q_min, q_max)
        joint_trajectory = self.create_trajectory(q_list, v_list, a_list, t)
        return joint_trajectory

    def moveit_plan(self, q_start, q_goal, q_min, q_max):
        self.group.clear_pose_targets()
        self.group.set_joint_value_target(q_goal)
        plan=self.group.plan()
        joint_trajectory = plan.joint_trajectory
        for i in range(0,len(joint_trajectory.points)):
            joint_trajectory.points[i].time_from_start = \
              rospy.Duration(joint_trajectory.points[i].time_from_start)
        return joint_trajectory        

    def create_trajectory(self, q_list, v_list, a_list, t):
        joint_trajectory = trajectory_msgs.msg.JointTrajectory()
        for i in range(0, len(q_list)):
            point = trajectory_msgs.msg.JointTrajectoryPoint()
            point.positions = list(q_list[i])
            point.velocities = list(v_list[i])
            point.accelerations = list(a_list[i])
            point.time_from_start = rospy.Duration(t[i])
            joint_trajectory.points.append(point)
        joint_trajectory.joint_names = self.joint_names
        return joint_trajectory

    def execute(self, joint_trajectory):
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        goal.trajectory = joint_trajectory
        goal.goal_time_tolerance = rospy.Duration(0.0)
        self.trajectory_client.send_goal(goal)
        self.trajectory_client.wait_for_result()

    def sample_polynomial(self, coeffs, i, T):
        q = coeffs[4*i+0]*T*T*T + coeffs[4*i+1]*T*T + coeffs[4*i+2]*T + coeffs[4*i+3]
        v = coeffs[4*i+0]*3*T*T + coeffs[4*i+1]*2*T + coeffs[4*i+2]
        a = coeffs[4*i+0]*6*T   + coeffs[4*i+1]*2
        return (q,a,v)

    def plot_series(self, t_vec, y_vec, title):
        fig, ax = plt.subplots()
        line, = ax.plot(numpy.random.rand(10))
        ax.set_xlim(0, t_vec[-1])
        ax.set_ylim(min(y_vec),max(y_vec))
        line.set_xdata(deepcopy(t_vec))
        line.set_ydata(deepcopy(y_vec))
        fig.suptitle(title)

    def move_arm_cb(self, feedback):
        print 'Moving the arm'
        self.mutex.acquire()
        q_start = self.q_current
        T = convert_from_message(feedback.pose)
        print "Solving IK"
        q_goal = self.IK(T)
        if len(q_goal)==0:
            print "IK failed, aborting"
            self.mutex.release()
            return

        print "IK solved, planning"
        q_start = numpy.array(self.q_from_joint_state(self.joint_state))
        trajectory = self.project_plan(q_start, q_goal, self.q_min, self.q_max)
        if not trajectory.points:
            print "Motion plan failed, aborting"
        else:
            print "Trajectory received with " + str(len(trajectory.points)) + " points"
            self.execute(trajectory)
        self.mutex.release()

    def no_obs_cb(self, feedback):
        print 'Removing all obstacles'
        self.scene.remove_world_object("obs1")
        self.scene.remove_world_object("obs2")
        self.scene.remove_world_object("obs3")
        self.scene.remove_world_object("obs4")

    def simple_obs_cb(self, feedback):
        print 'Adding simple obstacle'
        self.no_obs_cb(feedback)
        pose_stamped = geometry_msgs.msg.PoseStamped()
        pose_stamped.header.frame_id = "base"
        pose_stamped.header.stamp = rospy.Time(0)

        pose_stamped.pose = convert_to_message( tf.transformations.translation_matrix((0.5, 0.5, 0)) )
        self.scene.add_box("obs1", pose_stamped,(0.1,0.1,1))

    def complex_obs_cb(self, feedback):
        print 'Adding hard obstacle'
        self.no_obs_cb(feedback)
        pose_stamped = geometry_msgs.msg.PoseStamped()
        pose_stamped.header.frame_id = "base"
        pose_stamped.header.stamp = rospy.Time(0)
        pose_stamped.pose = convert_to_message( tf.transformations.translation_matrix((0.7, 0.5, 0.2)) )
        self.scene.add_box("obs1", pose_stamped,(0.1,0.1,0.8))
        pose_stamped.pose = convert_to_message( tf.transformations.translation_matrix((0.7, 0.25, 0.6)) )
        self.scene.add_box("obs2", pose_stamped,(0.1,0.5,0.1))

    def super_obs_cb(self, feedback):
        print 'Adding super hard obstacle'
        self.no_obs_cb(feedback)
        pose_stamped = geometry_msgs.msg.PoseStamped()
        pose_stamped.header.frame_id = "base"
        pose_stamped.header.stamp = rospy.Time(0)
        pose_stamped.pose = convert_to_message( tf.transformations.translation_matrix((0.7, 0.5, 0.2)) )
        self.scene.add_box("obs1", pose_stamped,(0.1,0.1,0.8))
        pose_stamped.pose = convert_to_message( tf.transformations.translation_matrix((0.7, 0.25, 0.6)) )
        self.scene.add_box("obs2", pose_stamped,(0.1,0.5,0.1))
        pose_stamped.pose = convert_to_message( tf.transformations.translation_matrix((0.7, 0.0, 0.2)) )
        self.scene.add_box("obs3", pose_stamped,(0.1,0.1,0.8))
        pose_stamped.pose = convert_to_message( tf.transformations.translation_matrix((0.7, 0.25, 0.1)) )
        self.scene.add_box("obs4", pose_stamped,(0.1,0.5,0.1))


    def plot_cb(self,feedback):
        handle = feedback.menu_entry_id
        state = self.menu_handler.getCheckState( handle )
        if state == MenuHandler.CHECKED: 
            self.show_plots = False
            print "Not showing plots"
            self.menu_handler.setCheckState( handle, MenuHandler.UNCHECKED )
        else:
            self.show_plots = True
            print "Showing plots"
            self.menu_handler.setCheckState( handle, MenuHandler.CHECKED )
        self.menu_handler.reApply(self.server)
        self.server.applyChanges()
        
    def joint_states_callback(self, joint_state):
        self.mutex.acquire()
        self.q_current = joint_state.position
        self.joint_state = joint_state
        self.mutex.release()

    def init_marker(self):

        self.server = InteractiveMarkerServer("control_markers")

        control_marker = InteractiveMarker()
        control_marker.header.frame_id = "/base"
        control_marker.name = "move_arm_marker"

        move_control = InteractiveMarkerControl()
        move_control.name = "move_x"
        move_control.orientation.w = 1
        move_control.orientation.x = 1
        move_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control_marker.controls.append(move_control)
        move_control = InteractiveMarkerControl()
        move_control.name = "move_y"
        move_control.orientation.w = 1
        move_control.orientation.y = 1
        move_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control_marker.controls.append(move_control)
        move_control = InteractiveMarkerControl()
        move_control.name = "move_z"
        move_control.orientation.w = 1
        move_control.orientation.z = 1
        move_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control_marker.controls.append(move_control)

        move_control = InteractiveMarkerControl()
        move_control.name = "rotate_x"
        move_control.orientation.w = 1
        move_control.orientation.x = 1
        move_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control_marker.controls.append(move_control)
        move_control = InteractiveMarkerControl()
        move_control.name = "rotate_y"
        move_control.orientation.w = 1
        move_control.orientation.z = 1
        move_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control_marker.controls.append(move_control)
        move_control = InteractiveMarkerControl()
        move_control.name = "rotate_z"
        move_control.orientation.w = 1
        move_control.orientation.y = 1
        move_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control_marker.controls.append(move_control)

        menu_control = InteractiveMarkerControl()
        menu_control.interaction_mode = InteractiveMarkerControl.BUTTON
        menu_control.always_visible = True
        box = Marker()        
        box.type = Marker.CUBE
        box.scale.x = 0.15
        box.scale.y = 0.03
        box.scale.z = 0.03
        box.color.r = 0.5
        box.color.g = 0.5
        box.color.b = 0.5
        box.color.a = 1.0
        menu_control.markers.append(box)
        box2 = deepcopy(box)
        box2.scale.x = 0.03
        box2.scale.z = 0.1
        box2.pose.position.z=0.05
        menu_control.markers.append(box2)
        control_marker.controls.append(menu_control)

        control_marker.scale = 0.25        
        self.server.insert(control_marker, self.control_marker_feedback)

        self.menu_handler = MenuHandler()
        self.menu_handler.insert("Move Arm", callback=self.move_arm_cb)
        obs_entry = self.menu_handler.insert("Obstacles")
        self.menu_handler.insert("No Obstacle", callback=self.no_obs_cb, parent=obs_entry)
        self.menu_handler.insert("Simple Obstacle", callback=self.simple_obs_cb, parent=obs_entry)
        self.menu_handler.insert("Hard Obstacle", callback=self.complex_obs_cb, parent=obs_entry)
        self.menu_handler.insert("Super-hard Obstacle", callback=self.super_obs_cb, parent=obs_entry)
        options_entry = self.menu_handler.insert("Options")
        self.plot_entry = self.menu_handler.insert("Plot trajectory", parent=options_entry,
                                                     callback = self.plot_cb)
        self.menu_handler.setCheckState(self.plot_entry, MenuHandler.UNCHECKED)
        self.menu_handler.apply(self.server, "move_arm_marker",)

        self.server.applyChanges()

        Ttrans = tf.transformations.translation_matrix((0.6,0.2,0.2))
        Rtrans = tf.transformations.rotation_matrix(3.14159,(1,0,0))
        self.server.setPose("move_arm_marker", convert_to_message(numpy.dot(Ttrans,Rtrans)))
        self.server.applyChanges()


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_arm', anonymous=True)
    ma = MoveArm()
    rospy.spin()

