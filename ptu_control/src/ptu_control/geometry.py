#!/usr/bin/env python

import roslib; roslib.load_manifest('ptu_control')
from tf.transformations import euler_from_matrix
import numpy as np
import cv
from missouri_telepresence import surf_helper

X_AXIS, Y_AXIS, Z_AXIS = range(3)

def euler_from_homography(H, K):
	# apply the calibration matrix to H, then take the QR decomposition
	q, r = np.linalg.qr(K.I*H*K)

	# make sure q is a rotation matrix (by checking q.T*q is the identity matrix)
	#assert np.allclose((q.T*q), np.eye(3)), 'q is not a rotation matrix'

	# euler_from_matrix needs a 4x4 matrix, so let's make one
	z = np.matrix(np.eye(4))
	print q
	z[:3,:3] = q

	# make sure z is a rotation matrix (by checking z.T*z is the identity matrix)
	#assert np.allclose((z.T*z), np.eye(4)), 'z is not a rotation matrix'

	return euler_from_matrix(z)
	
def y_rot_from_homography(H, K):
	print "r = \n %s" % str(K.I*H*K)
	q, r = np.linalg.qr(K.I*H*K)
	# q, r = np.linalg.qr(K*H*K.I)
	# return np.average([np.arccos(q[0,0]),
	# 				   np.arcsin(q[0,2]),
	# 				  -np.arcsin(q[2,0]),
	# 				   np.arccos(q[2,2])
	# 				])
	return [np.arccos(q[0,0]),
		    np.arcsin(q[0,2]),
		   -np.arcsin(q[2,0]),
		    np.arccos(q[2,2])
		   ]	

if __name__ == '__main__':
	# Homography
	H = np.matrix(
		[[1.7490, 0.0043, -433.5812],
		[0.2552,  1.5082, -116.5790],
		[0.0012,  0.0001,  1.0000]]
	)

	# Camera calibration matrix (intrinsic params)
	K = np.matrix(
		[[442.0372,        0, 285.8273],
		[        0, 445.1481, 204.9331],
		[        0,        0, 1.0000]]
	)
	print np.degrees(euler_from_homography(H, K))
	
def get_surf_features(image):
	image = cv.GetMat(image)
	img_sz = cv.GetSize(image)

	eig_image = cv.CreateImage(img_sz, cv.IPL_DEPTH_32F, 1)
	tmp_image = cv.CreateImage(img_sz, cv.IPL_DEPTH_32F, 1)
	
	return cv.ExtractSURF(image, None, cv.CreateMemStorage(), (0, 200, 3, 1))
	
def rotation_from_images(im1, im2, K, axis=None):
	feat_1 = get_surf_features(im1)
	feat_2 = get_surf_features(im2)
	
	H = cv.CreateMat(3, 3, cv.CV_32FC1)
	feat, pts = surf_helper.getPointLists(feat_1[0], feat_1[1], feat_2[0], feat_2[1])
	cv.FindHomography(np.asarray(pts), np.asarray(feat), H, cv.CV_RANSAC)
	
	
	if axis == X_AXIS:
		pass
		
	if axis == Y_AXIS:
		return y_rot_from_homography(H, K)
		
	if axis == Z_AXIS:
		pass