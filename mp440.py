import inspect
import sys
import numpy as np

'''
Raise a "not defined" exception as a reminder 
'''


def _raise_not_defined():
	print "Method not implemented: %s" % inspect.stack()[1][3]
	sys.exit(1)


# 5 formulas
def kf_predictStateEstimate(A, xPrev, B, U):
	predX = A * xPrev + B * U
	return predX


def kf_predictErrorCovariance(A, pPrev, Q):
	predP = A * pPrev * np.transpose(A) + Q
	return predP


def kf_updateKalmanGain(P, H, R):
	return P * np.transpose(H) * np.linalg.inv(H * P * np.transpose(H) + R)


def kf_updateStateEstimate(X, kalman, H, Z):
	return X + kalman * (Z - H * X)


def kf_updateEstimateCovariance(H, kalman, P):
	return (np.identity(2) - kalman * H) * P


'''
Kalman 2D
'''
def kalman2d(data, scale=1):
	estimated = []
	
	#Noise Calculations
	Q = np.matrix([[0.0001, 0.00002], [0.00002, 0.0001]])
	R = np.matrix([[0.01, 0.05], [0.05, 0.02]])
	
	#Identity matricies
	Amx = np.identity(2)
	Bmx = np.identity(2)
	Hmx = np.identity(2)
	print("DATA VALS")
	print data
	print("ESTIMATED VALS")
	
	#initial estimation
	lambdaScaling = scale
	pPrev = lambdaScaling * np.identity(2)
	X = np.transpose(np.matrix([0,0]))
	
	estimated.append(X)
	
	index = 0
	for dat in data:
		uVal = np.matrix([dat[0], dat[1]])
		zVal = np.matrix([dat[2], dat[3]])
		uVal = np.transpose(uVal)
		zVal = np.transpose(zVal)
		
		xPrev = estimated[index]
		#use functions
		xPredict = kf_predictStateEstimate(Amx, xPrev, Bmx, uVal)
		pPredict = kf_predictErrorCovariance(Amx, pPrev, Q)
		kalmanGain = kf_updateKalmanGain(pPredict, Hmx, R)
		xUpdate = kf_updateStateEstimate(xPredict, kalmanGain, Hmx,zVal)
		pUpdate = kf_updateEstimateCovariance(Hmx, kalmanGain, pPredict)
		pPrev = pUpdate
		
		estimated.append(xUpdate)
		index += 1
	
	print estimated
	return estimated


'''
Plotting
'''


def plot(data, output):
	# Your code starts here
	# You should remove _raise_not_defined() after you complete your code
	# Your code ends here
	_raise_not_defined()
	return


'''
Kalman 2D 
'''


def kalman2d_shoot(ux, uy, ox, oy, reset=False):
	decision = (0, 0, False)
	# Your code starts here
	# Your code ends here
	_raise_not_defined()
	return decision


'''
Kalman 2D 
'''


def kalman2d_adv_shoot(ux, uy, ox, oy, reset=False):
	decision = (0, 0, False)
	# Your code starts here
	# Your code ends here
	_raise_not_defined()
	return decision
