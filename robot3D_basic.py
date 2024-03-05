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

def main(thetas, L1, L2, L3, L4):
	axes = Axes(xrange=(0,15), yrange=(0,10), zrange=(0,10))
	plotter = Plotter(axes=10, interactive=True)
	video = Video("anim.mp4", duration=7, backend="ffmpeg")
	video.action()

	for i in range(192):
		thetas = [theta + (i * .15) for theta in thetas]
		frame1, frame2, frame3, frame4, ee = forward_kinematics(Phi=thetas, L1=L1, L2=L2, L3=L3, L4=L4)
		plotter.clear() # bye bye prev frame
		plotter.show([frame1, frame2, frame3, frame4], axes=axes, viewup="z")
		video.add_frame()
	else:
		video.close()
		show([frame1, frame2, frame3, frame4], axes=10, viewup="z").close()

if __name__ == '__main__':
		from time import sleep
		main(np.array([-30, 50, 30, 0]),
			5,
			8,
			3,
			0,)
		# T_01,	T_02,	T_03,	T_04,	e = forward_kinematics(
		# 	np.array([-30, 50, 30, 0]),
		# 	5,
		# 	8,
		# 	3,
		# 	0,
		# )
		# print(f"{T_01.GetPosition()=}")
		# print(f"{T_02.GetPosition()=}")
		# print(f"{T_03.GetPosition()=}")
		# print(f"{T_04.GetPosition()=}")
		# print(f"{e=}")

		# axes = Axes(xrange=(0,20), yrange=(-2,10), zrange=(0,6))
		# show([T_01,	T_02,	T_03,	T_04], axes, viewup="z").close()

