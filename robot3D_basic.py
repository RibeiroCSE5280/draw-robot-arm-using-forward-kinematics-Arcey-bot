#!/usr/bin/env python
# coding: utf-8


from vedo import *

def RotationMatrix(theta, axis_name):
		""" calculate single rotation of $theta$ matrix around x,y or z
				code from: https://programming-surgeon.com/en/euler-angle-python-en/
		input
				theta = rotation angle(degrees)
				axis_name = 'x', 'y' or 'z'
		output
				3x3 rotation matrix
		"""

		c = np.cos(theta * np.pi / 180)
		s = np.sin(theta * np.pi / 180)
	
		if axis_name =='x':
				rotation_matrix = np.array([[1, 0,  0],
																		[0, c, -s],
																		[0, s,  c]])
		if axis_name =='y':
				rotation_matrix = np.array([[ c,  0, s],
																		[ 0,  1, 0],
																		[-s,  0, c]])
		elif axis_name =='z':
				rotation_matrix = np.array([[c, -s, 0],
																		[s,  c, 0],
																		[0,  0, 1]])
		return rotation_matrix


def createCoordinateFrameMesh():
		"""Returns the mesh representing a coordinate frame
		Args:
			No input args
		Returns:
			F: vedo.mesh object (arrows for axis)
			
		"""         
		_shaft_radius = 0.05
		_head_radius = 0.10
		_alpha = 1
		
		
		# x-axis as an arrow  
		x_axisArrow = Arrow(start_pt=(0, 0, 0),
												end_pt=(1, 0, 0),
												s=None,
												shaft_radius=_shaft_radius,
												head_radius=_head_radius,
												head_length=None,
												res=12,
												c='red',
												alpha=_alpha)

		# y-axis as an arrow  
		y_axisArrow = Arrow(start_pt=(0, 0, 0),
												end_pt=(0, 1, 0),
												s=None,
												shaft_radius=_shaft_radius,
												head_radius=_head_radius,
												head_length=None,
												res=12,
												c='green',
												alpha=_alpha)

		# z-axis as an arrow  
		z_axisArrow = Arrow(start_pt=(0, 0, 0),
												end_pt=(0, 0, 1),
												s=None,
												shaft_radius=_shaft_radius,
												head_radius=_head_radius,
												head_length=None,
												res=12,
												c='blue',
												alpha=_alpha)
		
		originDot = Sphere(pos=[0,0,0], 
											 c="black", 
											 r=0.10)


		# Combine the axes together to form a frame as a single mesh object 
		F = x_axisArrow + y_axisArrow + z_axisArrow + originDot
				
		return F


def getLocalFrameMatrix(R_ij, t_ij): 
		"""Returns the matrix representing the local frame
		Args:
			R_ij: rotation of Frame j w.r.t. Frame i 
			t_ij: translation of Frame j w.r.t. Frame i 
		Returns:
			T_ij: Matrix of Frame j w.r.t. Frame i. 
			
		"""             
		# Rigid-body transformation [ R t ]
		T_ij = np.block([[R_ij,                t_ij],
										 [np.zeros((1, 3)),       1]])
		
		return T_ij

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
		trans = getLocalFrameMatrix(rot, np.array([[lengths[i-1]], [0.0], [0.0]]))
				
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
		sphere = Sphere(r=0.4).pos(-0.4,0,0).color("gray").alpha(.8)
		Frame = FrameArrows + link_mesh + sphere
		Frame.apply_transform(T)
		frames.append(Frame)
		
	T_01, T_02, T_03, T_04 = frames
	e = np.array([0, 0, 0])
	return T_01,	T_02,	T_03,	T_04,	e

def main():
	# Set the limits of the graph x, y, and z ranges 
	axes = Axes(xrange=(0,20), yrange=(-2,10), zrange=(0,6))

	# Lengths of arm parts 
	L1 = 5   # Length of link 1
	L2 = 8   # Length of link 2
	L3 = 3	 # Length of link 3

	# Joint angles 
	phi1 = -30     # Rotation angle of part 1 in degrees (red)
	phi2 = 50    # Rotation angle of part 2 in degrees (yellow)
	phi3 = 30      # Rotation angle of the end-effector in degrees
	phi4 = 0
	
	# Matrix of Frame 1 (written w.r.t. Frame 0, which is the previous frame) 
	R_01 = RotationMatrix(phi1, axis_name = 'z')   # Rotation matrix
	p1   = np.array([[3],[2], [0.0]])              # Frame's origin (w.r.t. previous frame)
	t_01 = p1                                      # Translation vector
	
	T_01 = getLocalFrameMatrix(R_01, t_01)         # Matrix of Frame 1 w.r.t. Frame 0 (i.e., the world frame)
	print(f'{T_01=}')
	# Create the coordinate frame mesh and transform
	Frame1Arrows = createCoordinateFrameMesh()
	
	# Now, let's create a cylinder and add it to the local coordinate frame
	link1_mesh = Cylinder(r=0.4, 
												height=L1, 
												pos = (L1/2,0,0),
												c="yellow", 
												alpha=.8, 
												axis=(1,0,0)
												)
	
	# Also create a sphere to show as an example of a joint
	r1 = 0.4
	sphere1 = Sphere(r=r1).pos(-r1,0,0).color("gray").alpha(.8)

	# Combine all parts into a single object 
	Frame1 = Frame1Arrows + link1_mesh + sphere1

	# Transform the part to position it at its correct location and orientation 
	Frame1.apply_transform(T_01)  
	
	# Matrix of Frame 2 (written w.r.t. Frame 1, which is the previous frame) 	
	R_12 = RotationMatrix(phi2, axis_name = 'z')   # Rotation matrix
	p2   = np.array([[L1],[0.0], [0.0]])           # Frame's origin (w.r.t. previous frame)
	t_12 = p2                                      # Translation vector
	
	# Matrix of Frame 2 w.r.t. Frame 1 
	T_12 = getLocalFrameMatrix(R_12, t_12)
	print(f'{T_12=}')
	
	# Matrix of Frame 2 w.r.t. Frame 0 (i.e., the world frame)
	T_02 = T_01 @ T_12
	print(f'{T_02=}')
	
	# Create the coordinate frame mesh and transform
	Frame2Arrows = createCoordinateFrameMesh()
	
	# Now, let's create a cylinder and add it to the local coordinate frame
	link2_mesh = Cylinder(r=0.4, 
												height=L2, 
												pos = (L2/2,0,0),
												c="red", 
												alpha=.8, 
												axis=(1,0,0)
												)
	
	r2 = 0.4
	sphere2 = Sphere(r=r2).pos(-r2,0,0).color("gray").alpha(.8)
	
	# Combine all parts into a single object 
	Frame2 = Frame2Arrows + link2_mesh + sphere2
	
	# Transform the part to position it at its correct location and orientation 
	Frame2.apply_transform(T_02)  
	
	# Matrix of Frame 3 (written w.r.t. Frame 2, which is the previous frame) 	
	R_23 = RotationMatrix(phi3, axis_name = 'z')   # Rotation matrix
	p3   = np.array([[L2],[0.0], [0.0]])           # Frame's origin (w.r.t. previous frame)
	t_23 = p3                                      # Translation vector
	
	# Matrix of Frame 3 w.r.t. Frame 2 
	T_23 = getLocalFrameMatrix(R_23, t_23)
	
	# Matrix of Frame 3 w.r.t. Frame 0 (i.e., the world frame)
	T_03 = T_01 @ T_12 @ T_23
	
	# Create the coordinate frame mesh and transform. This point is the end-effector. So, I am 
	# just creating the coordinate frame. 
	Frame3Arrows = createCoordinateFrameMesh()

	# Now, let's create a cylinder and add it to the local coordinate frame
	link3_mesh = Cylinder(r=0.4, 
												height=L3, 
												pos = (L3/2,0,0),
												c="green", 
												alpha=.8, 
												axis=(1,0,0)
												)
	
	r3=0.4
	sphere3 = Sphere(r=r3).pos(-r3,0,0).color("gray").alpha(.8)

	# Combine all parts into a single object 
	Frame3 = Frame3Arrows + link3_mesh + sphere3

	# Transform the part to position it at its correct location and orientation 
	Frame3.apply_transform(T_03)  

	R_34 = RotationMatrix(phi4, axis_name='z')
	p4 = np.array([[L3], [0.0], [0.0]])
	t_34 = p4

	T_34 = getLocalFrameMatrix(R_34, t_34)
	T_04 = T_01 @ T_12 @ T_23 @ T_34

	Frame4 = createCoordinateFrameMesh()

	Frame4.apply_transform(T_04)

	# Show everything 
	show([Frame1, Frame2, Frame3, Frame4], axes, viewup="z").close()

if __name__ == '__main__':
		main()
		T_01,	T_02,	T_03,	T_04,	e = forward_kinematics(
			np.array([-30, 50, 30, 0]),
			5,
			8,
			3,
			0,
		)
		print(f"{T_01=}")
		print(f"{T_02=}")
		print(f"{T_03=}")
		print(f"{T_04=}")
		print(f"{e=}")

		axes = Axes(xrange=(0,20), yrange=(-2,10), zrange=(0,6))
		show([T_01,	T_02,	T_03,	T_04], axes, viewup="z").close()

