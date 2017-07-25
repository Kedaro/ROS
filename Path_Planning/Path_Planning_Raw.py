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
       
