import roslib; roslib.load_manifest('ptu_control')
import cv

class PanTiltKF(object):
	kalman = cv.CreateKalman(2, 2, 2)
	state  = cv.CreateMat(2, 1, cv.CV_32FC1) # (pan, tilt)
	
	cv.SetIdentity(kalman.transition_matrix)
	cv.SetIdentity(kalman.measurement_matrix)
	cv.SetIdentity(kalman.process_noise_cov, cv.RealScalar(1e-5))
	cv.SetIdentity(kalman.measurement_noise_cov, cv.RealScalar(1e-1))
	cv.SetIdentity(kalman.error_cov_post)

	def control(self, control):
		return cv.KalmanPredict(self.kalman, cv.fromarray(np.array(control, dtype=float32).reshape(2,1)))

	def measurement(self, meas):
		return cv.KalmanCorrect(self.kalman, cv.fromarray(np.array(meas, dtype=float32).reshape(2,1)))

if __name__ == '__main__':
	k = PanTiltKF()

