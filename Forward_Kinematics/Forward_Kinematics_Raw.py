def forward_kinematics(self, link_transforms, joints, joint_values):
    transforms = []
#Step 0: create a series of lists that will hold matricies 
	J_rot_list=[]
	L_comp_list=[]
	RL_comp_list=[]
	recourse_list=[numpy.identity(4)]
#Step 1: create for loop that will cycle through links and joints and create transformation matricies
	for i in range(len(joints)):
	
		if joints[i].type == "fixed": #Create contingencies for fixed joints
			J_rot=numpy.identity(4)
		else: #Create joint transformation using q and axis
			J_rot=tf.transformations.rotation_matrix(joint_values[i], joints[i].axis)
		J_rot_list.append(J_rot)	
	
	#Create link transformations using rpy and xyz translations
		L_trans=tf.transformations.translation_matrix(link_transforms[i].xyz)
		L_rot=tf.transformations.euler_matrix(link_transforms[i].rpy[0], link_transforms[i].rpy[1],link_transforms[i].rpy[2], 'rxyz')
		L_comp=numpy.dot(L_trans,L_rot)
		L_comp_list.append(L_comp)
	#Dot joint(i) and link(i)
		RL_comp=numpy.dot(J_rot_list[i],L_comp_list[i])
		RL_comp_list.append(RL_comp)
#Step 2: append transformaitons list with first terms 0 and 1
	L_T_0=J_rot_list[0]
	L_T_1=numpy.dot(RL_comp_list[0], J_rot_list[1])
	transforms.append(L_T_0)
	transforms.append(L_T_1)


#Step 3: append recursive list with first term joint(0) and link(0)
	recourse_list.append(RL_comp_list[0])
#Step 4: create loop that will recusively update list with incrementally
	for i in range(2,len(joints)):
		recourse=numpy.dot(recourse_list[i-1],RL_comp_list[i-1])
		recourse_list.append(recourse)
		L_T=numpy.dot(recourse_list[i], J_rot_list[i])
		transforms.append(L_T)
#Step 5: return transforms list
	print transforms
	return transforms