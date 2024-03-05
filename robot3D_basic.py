def forward_kinematics(Phi,	L1,	L2,	L3,	L4):
	"""Calculate	the	local-to-global	frame	matrices,	
	and	the	location	of	the	end-effector.
					
	Args:
					Phi	(4x1	nd.array):							Array	containing	the	four	joint	angles
					L1,	L2,	L3,	L4	(float):			lengths	of	the	parts	of	the	robot	arm.	
																													e.g.,	Phi	=	np.array([0,	-10,	20,	0])
	Returns:
					T_01,	T_02,	T_03,	T_04:			4x4	nd.arrays	of	local-to-global	matrices for	each	frame.
					e: 3x1	nd.array	of	3-D	coordinates, the location	of	the	end-effector	in	space.
	"""
	frames = []
	local_frames = []
	colors = ['red', 'blue', 'green']
	lengths = [L1, L2, L3, L4]

	# Get initial frame
	R01 = RotationMatrix(Phi[0], axis_name='z')
	T01 = getLocalFrameMatrix(R01, np.array([[3], [2], [0.0]]))
	Frame1Arrows = createCoordinateFrameMesh()
	link1_mesh = Cylinder(r=0.4, 
												height=L1, 
												pos = (L1/2,0,0),
												c="yellow", 
												alpha=.8, 
												axis=(1,0,0)
												)
	r1 = 0.4 # joint
	sphere1 = Sphere(r=r1).pos(-r1,0,0).color("gray").alpha(.8)

	Frame1 = Frame1Arrows + link1_mesh + sphere1
	Frame1.apply_transform(T01)
	frames.append(Frame1) # save

	T_0N = T01 
	# We can now calculate all subsequent frames w.r.t F01
	for i, v in enumerate([L2, L3, L4], 1):
		rot = RotationMatrix(Phi[i], axis_name= 'z') # rot for frame N w.r.t N-1
		# T_(N-1)N
		trans = getLocalFrameMatrix(rot, np.array([[lengths[i-1]+0.4], [0.0], [0.0]]))
				
		T = T_0N @ trans
		T_0N = T # update for future calcuations

		FrameArrows = createCoordinateFrameMesh()
		link_mesh = Cylinder(r=0.4, 
												height=v, 
												pos = (v/2,0,0),
												c=colors[i-1], 
												alpha=.8, 
												axis=(1,0,0)
												)
		sphere = Sphere(r=0.4).pos(-0.0,0.0,0).color("gray").alpha(.8)
		Frame = FrameArrows + link_mesh

		if i != len(lengths) - 1: # dont add joint to end
			Frame += sphere

		Frame.apply_transform(T)
		frames.append(Frame)
		
	T_01, T_02, T_03, T_04 = frames
	e = np.array(frames[-1].GetPosition())
	return T_01,	T_02,	T_03,	T_04,	e
