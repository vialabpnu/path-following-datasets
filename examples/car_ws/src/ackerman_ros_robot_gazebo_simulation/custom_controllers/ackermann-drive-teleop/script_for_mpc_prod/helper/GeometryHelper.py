import time

import numpy as np


def getProjPointOnLineSeg(p, lp1, lp2, isLine=False):
	# startTime = time.time()
	lineVec = lp2 - lp1
	lineVec = lineVec/np.linalg.norm(lineVec)
	# print(time.time() - startTime)
	proj = lp1 + lineVec * np.dot(lineVec, p - lp1)
	if not isLine:
		if np.dot(lineVec, p-lp1) < 0:
			proj = lp1
		elif np.dot(lineVec, p-lp2) > 0:
			proj = lp2
	return proj

def getProjPointOnLineSegVec(p, lp1Vec, lp2Vec):
	lineVec = lp2Vec - lp1Vec
	lineVec = lineVec/np.linalg.norm(lineVec)
	lp1pVec = p - lp1Vec
	lp2pVec = p - lp2Vec

	dotLineLp1pVec = np.sum(np.multiply(lineVec, lp1pVec), axis=1)
	dotLineLp1pVecMat = np.stack([dotLineLp1pVec, dotLineLp1pVec], axis=1)
	projVec = lp1Vec + np.multiply(lineVec, dotLineLp1pVecMat)
	dotLineLp2pVec = np.sum(np.multiply(lineVec, lp2pVec), axis=1)

	projVec[dotLineLp1pVec < 0] = lp1Vec[dotLineLp1pVec < 0]
	projVec[dotLineLp2pVec > 0] = lp2Vec[dotLineLp2pVec > 0]
	return projVec

# def getProjPointOnLineSeg(p, lp1, lp2, lineVec, lp1p, lp2p, proj):
# 	# proj = lp1 + lineVec * np.dot(lineVec, lp1p)
#
# 	if np.dot(lineVec, lp1p) < 0:
# 		proj = lp1
# 	elif np.dot(lineVec, lp2p) > 0:
# 		proj = lp2
# 	return proj

def getRatioOfPointOnLineSeg(p, lp1, lp2):
	return np.linalg.norm(np.array(p-lp1))/np.linalg.norm(np.array(lp2-lp1))

def getInterpolateData(p1, p2, ratio):
	return p1 + np.array(p2 - p1) * ratio

def getPointOneLineSegWithDistFromPoint(p, lp1, lp2, dist):
	lineVec = np.array(lp2 - lp1)
	lineVec = lineVec/np.linalg.norm(lineVec)
	return p + lineVec * dist

def getAngleBetweenTwoLineSeg(l1p1, l1p2, l2p1, l2p2):
	l1 = np.array(l1p2 - l1p1)
	l1 = l1/np.linalg.norm(l1)
	l2 = np.array(l2p2 - l2p1)
	l2 = l2/np.linalg.norm(l2)
	return np.arccos(np.clip(np.dot(l1, l2), -1.0, 1.0))

def getPathLineLen(path):
	if path.shape[0] <= 1 or path.ndim < 2:
		return 0
	lineSegLen = np.linalg.norm(np.array(path[1:,:]-path[0:-1,:]),axis=1)
	return np.sum(lineSegLen)