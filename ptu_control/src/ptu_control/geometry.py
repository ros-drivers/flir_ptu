#!/usr/bin/env python

import roslib; roslib.load_manifest('ptu_control')
from tf.transformations import euler_from_matrix
import numpy as np

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
	q, r = np.linalg.qr(K.I*H*K)
	return np.average([np.arccos(q[0,0]),
					   np.arcsin(q[0,2]),
					  -np.arcsin(q[2,0]),
					   np.arccos(q[2,2])
					])

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