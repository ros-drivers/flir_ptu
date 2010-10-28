# import roslib; roslib.load_manifest('ptu_control')
# import rospy
import cv
import numpy as np

class PanTiltKF(object):
	kalman = cv.CreateKalman(2, 2, 2)
	state  = cv.CreateMat(2, 1, cv.CV_32FC1) # (pan, tilt)

	meas = False

	# kalman.state_pre[0,0] = 1000
	# kalman.state_pre[1,0] = 1000

	cv.SetIdentity(kalman.transition_matrix)
	cv.SetIdentity(kalman.measurement_matrix)
	cv.SetIdentity(kalman.control_matrix)
	cv.SetIdentity(kalman.process_noise_cov, cv.RealScalar(1e-5))
	cv.SetIdentity(kalman.measurement_noise_cov, cv.RealScalar(1e-1))
	cv.SetIdentity(kalman.error_cov_post)

	def control(self, control):
		prediction = cv.KalmanPredict(self.kalman, vectorize(control))
		# rospy.loginfo('Control predicted (%s, %s)' % (prediction[0,0], prediction[1,0]))
		# self.kalman.state_post = self.kalman.state_post
		
		if not self.meas:
			cv.Copy(self.kalman.state_pre, self.kalman.state_post)
		self.meas = False
		return prediction[0,0], prediction[1,0]

	def measurement(self, meas):
		correction = cv.KalmanCorrect(self.kalman, vectorize(meas))
		# rospy.loginfo('Measurement corrected (%s, %s)' % (correction[0,0], correction[1,0]))
		self.meas = True
		return correction[0,0], correction[1,0]
		# return (0,0)

def vectorize(t):
	return cv.fromarray(np.array([t], dtype=np.float32).reshape(2,1))
	
if __name__ == '__main__':
	k = PanTiltKF()
	print k.kalman.transition_matrix[0,0], k.kalman.transition_matrix[0,1]
	print k.kalman.transition_matrix[1,0], k.kalman.transition_matrix[1,1]
	print k.kalman.state_pre[0,0], k.kalman.state_pre[1,0]
	for i in range(10):
		print k.control((i,i))
		print k.kalman.state_pre[0,0], k.kalman.state_pre[1,0]
