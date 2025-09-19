import numpy as np
import matplotlib.pyplot as plt
import math
import cvxpy as cp
import time
import csv

from data import GeometryHelper
from scipy.integrate import solve_ivp


class SimpleTrajPlanning:
	def __init__(self, inputPath, speedRef, accelRef, decelRef, minSpeedToMove, minSpeedToStop, earlyStoppingDist=0, maxSpeed=10, nextSpeedCompensationByAccelFeedbackRatio=None, minSpeedAfterCompensation=0.1, samplingTime=0.1, predictionHorizonLen=10, stopTimeStepAtCuspPoint=10):
		self.samplingTime = samplingTime
		self.predictionHorizonLen = predictionHorizonLen
		self.stopTimeStepAtCuspPoint = stopTimeStepAtCuspPoint
		self.MinSpeedToMove = minSpeedToMove
		# self.minSpeedToStop = minSpeedToStop
		self.earlyStoppingDist = earlyStoppingDist
		self.RefDecel = decelRef
		self.RefAccel = accelRef
		self.RefSpeed = speedRef
		self.nextSpeedCompensationByAccelFeedbackRatio = nextSpeedCompensationByAccelFeedbackRatio
		self.minSpeedAfterCompensation = minSpeedAfterCompensation

		self.pathList, self.cuspPointIdxList = SimpleTrajPlanning.segmentPathByCuspPoint(SimpleTrajPlanning.removeOverlappedPointOnPath(inputPath))

		self.pathIdx = 0

		self.curTraj = None
		self.stopStep = 0
		self.hasStop = False
		self.finishPath = False
		self.lastSpeed = None

	# Return None if arrive at the goal
	def step(self, curPos, curSpeed, lastMPCAccel=None):
		if self.finishPath:
			return SimpleTrajPlanning.expandTrajToLen(self.curTraj, self.predictionHorizonLen)
		if self.stopStep > 0:
			self.stopStep -= 1
			self.hasStop = True
			return SimpleTrajPlanning.expandTrajToLen(self.curTraj, self.predictionHorizonLen)

		if self.curTraj is not None and self.curTraj.shape[0] <= 1:
			if self.hasStop == False:
				self.stopStep = max(self.stopTimeStepAtCuspPoint, 1)
				return SimpleTrajPlanning.expandTrajToLen(self.curTraj, self.predictionHorizonLen)
			else:
				self.pathIdx += 1
				self.hasStop = False
				if self.pathIdx >= len(self.pathList):
					self.finishPath = True
					return SimpleTrajPlanning.expandTrajToLen(self.curTraj, self.predictionHorizonLen)

		curSpeedDirectionLess = curSpeed if self.pathIdx % 2 == 0 else -curSpeed     # Only work with positive speed
		curSpeedDirectionLess = max(curSpeedDirectionLess, 0)
		# Get projection info on path
		projPointData, closestLineSegIdxOnPath = SimpleTrajPlanning.findProjInfoOnPath(curPos, self.pathList[self.pathIdx])
		pathLineLen = np.linalg.norm(projPointData[0:2] - self.pathList[self.pathIdx][closestLineSegIdxOnPath+1,0:2]) + GeometryHelper.getPathLineLen(self.pathList[self.pathIdx][closestLineSegIdxOnPath+1:,0:2])

		# Penalize the accel if it does not match the acceleration
		nextSpeedCompensationByAccelFeedback = 0
		lastMPCAccelFeedback = None
		if lastMPCAccel is not None and self.nextSpeedCompensationByAccelFeedbackRatio is not None and self.lastSpeed is not None:
			actualAccel = (curSpeed - self.lastSpeed)/self.samplingTime
			nextSpeedCompensationByAccelFeedback = (actualAccel - lastMPCAccel)*self.samplingTime*self.nextSpeedCompensationByAccelFeedbackRatio
			lastMPCAccelFeedback = lastMPCAccel

		speedProf = SimpleTrajPlanning.getTrapezoidalSpeedProfile(pathLineLen, curSpeedDirectionLess, self.RefAccel, self.RefDecel, self.RefSpeed, self.MinSpeedToMove, self.earlyStoppingDist, lastMPCAccelFeedback, nextSpeedCompensationByAccelFeedback, self.minSpeedAfterCompensation, self.samplingTime)

		if pathLineLen <= 0.01:     # Pass the goal point here, but speed is still high
			speedProf = np.array([0])

		self.curTraj = SimpleTrajPlanning.getTrajFromPath(curPos, self.pathList[self.pathIdx], projPointData, closestLineSegIdxOnPath, speedProf, self.samplingTime, self.predictionHorizonLen)
		speedProf = speedProf if self.pathIdx % 2 == 0 else -speedProf
		self.curTraj = np.column_stack([self.curTraj, speedProf[0:len(self.curTraj)]])
		self.lastSpeed = curSpeed
		if self.curTraj.shape[0] >= self.predictionHorizonLen:
			return self.curTraj
		else:
			return SimpleTrajPlanning.expandTrajToLen(self.curTraj, self.predictionHorizonLen)

	@staticmethod
	def expandTrajToLen(traj, targetLen):
		return np.row_stack([traj, np.repeat(np.expand_dims(traj[-1,:], axis=0), targetLen-traj.shape[0], axis=0)])

	@staticmethod
	def findProjInfoOnPath(curPos, path):
		# Find projection point list
		# projPointList = np.array([GeometryHelper.getProjPointOnLineSeg(curPos, lp1, lp2) for idx, (lp1, lp2) in enumerate(zip(path[:-1,0:2], path[1:,0:2]))])      # Old solution: slow
		projPointList = GeometryHelper.getProjPointOnLineSegVec(curPos, path[:-1,0:2], path[1:,0:2])

		# Find closest segment
		distToProjPoint = np.linalg.norm(curPos - projPointList, ord=2, axis=1)
		closestLineSegIdxOnPath = np.nanargmin(distToProjPoint, )
		projRatioClosest = GeometryHelper.getRatioOfPointOnLineSeg(projPointList[closestLineSegIdxOnPath, :], path[closestLineSegIdxOnPath, 0:2], path[closestLineSegIdxOnPath + 1, 0:2])
		projPointData = GeometryHelper.getInterpolateData(path[closestLineSegIdxOnPath, :], path[closestLineSegIdxOnPath + 1, :], projRatioClosest)
		return projPointData, closestLineSegIdxOnPath


	@staticmethod
	def getTrajFromPath(curPos, path, projPointData, closestLineSegIdxOnPath, speedProf, samplingTime, maxTrajLen=np.inf):
		trajLen = speedProf.size
		pathLen = path.shape[0]

		# Find the interpolated traj based on speed
		actualTrajLen = min(trajLen,maxTrajLen)
		traj = np.full((actualTrajLen, path.shape[1]), np.nan)
		traj[0,:] = projPointData
		curLineSeg = closestLineSegIdxOnPath
		curPoint = traj[0,0:2]
		for i in range(actualTrajLen-1):
			addDist = speedProf[i]*samplingTime
			if curLineSeg > pathLen - 2 or addDist == 0:
				traj[i+1,:] = traj[i,:]
				continue

			while addDist > 0:
				remDistOnLineSeg = np.linalg.norm(curPoint - path[curLineSeg+1, 0:2])
				if remDistOnLineSeg > addDist:
					curPoint = GeometryHelper.getPointOneLineSegWithDistFromPoint(curPoint, path[curLineSeg, 0:2], path[curLineSeg+1, 0:2], addDist)
					ratio = GeometryHelper.getRatioOfPointOnLineSeg(curPoint, path[curLineSeg, 0:2], path[curLineSeg + 1, 0:2])
					traj[i+1,:] = GeometryHelper.getInterpolateData(path[curLineSeg, :], path[curLineSeg + 1, :], ratio)
					break
				else:
					addDist = addDist - remDistOnLineSeg
					curPoint = path[curLineSeg + 1, 0:2]
					curLineSeg += 1
					if curLineSeg > pathLen - 2:
						traj[i+1,:] = path[curLineSeg, :]
						break

		return traj

	@staticmethod
	def removeOverlappedPointOnPath(path):
		newPath = [path[0,:]]
		for i in range(1, path.shape[0]):
			if not np.linalg.norm(path[i,0:2] - path[i-1,0:2]) < 0.01:
				newPath.append(path[i,:])
			else:
				print('Remove point on inputPath at: {}'.format(i))
		return np.array(newPath)

	@staticmethod
	def segmentPathByCuspPoint(path, minCuspPointAngleDeg=90):
		pathLen = path.shape[0]
		if pathLen < 3:
			return [path], []
		lineSegAngle = np.array([GeometryHelper.getAngleBetweenTwoLineSeg(p1, p2, p2, p3) for p1, p2, p3 in zip(path[:-2,0:2], path[1:-1,0:2], path[2:,0:2])])
		cuspPointIdx = np.array(np.where(lineSegAngle >= np.deg2rad(minCuspPointAngleDeg))[0]) + 1         # +1 because cusp point is in the middle of two line segments
		pathPoint = np.array([0, *cuspPointIdx, pathLen-1])

		pathList = []
		for i in range(len(pathPoint)-1):
			pathList.append(path[pathPoint[i]:pathPoint[i+1]+1,:])

		return pathList, cuspPointIdx

	@staticmethod
	def getDistFromSpeedProfile(speedProfile, dt):
		dist = []
		for speed in speedProfile:
			if len(dist) == 0:
				dist.append(0 + dt*speed)
			else:
				dist.append(dist[-1] + dt*speed)
		dist = np.array(dist)
		return dist

	@staticmethod
	def getTravelDistFromBetweenSpeed(speed1, speed2, accel, dt):
		speedProf = np.array(list(np.arange(speed1, speed2, dt*accel)))
		distList = SimpleTrajPlanning.getDistFromSpeedProfile(speedProf, dt)
		return distList[-1] if len(distList) > 0 else 0, speedProf
	@staticmethod
	def getTrapezoidalSpeedProfile(dist, curSpeed, RefAccel, RefDecel, RefSpeed, MinSpeedToMove, earlyStoppingDist, preMPCAccel, nextSpeedCompensationByAccelFeedback, minSpeedAfterCompensation, dt):
		decelDistFromRefSpeed, speedProfDecelFromRefSpeed = SimpleTrajPlanning.getTravelDistFromBetweenSpeed(RefSpeed, 0, -RefDecel, dt)
		decelDistFromCurSpeed, speedProfDecelFromCurSpeed = SimpleTrajPlanning.getTravelDistFromBetweenSpeed(curSpeed, 0, -RefDecel, dt)

		speedProfAll = []
		isInFinalDecelPhase = True
		if curSpeed > RefSpeed:
			decelDistFromCurSpeedToRefSpeed, speedProfDecelFromCurSpeedToRefSpeed = SimpleTrajPlanning.getTravelDistFromBetweenSpeed(curSpeed, RefSpeed, -RefDecel, dt)
			speedProfAll += list(speedProfDecelFromCurSpeedToRefSpeed)
			remDist = dist - decelDistFromCurSpeedToRefSpeed
			while remDist > decelDistFromRefSpeed + RefSpeed*dt + earlyStoppingDist:
				speedProfAll.append(RefSpeed)
				remDist -= RefSpeed*dt
				isInFinalDecelPhase = False
			speedProfAll += list(speedProfDecelFromRefSpeed)
		elif curSpeed > MinSpeedToMove - RefAccel*dt:     # if - RefAccel*dt/2: very close to MinSpeedToMove -> assume that pass
			if dist > decelDistFromCurSpeed + curSpeed*dt + earlyStoppingDist:
				for speed in speedProfDecelFromRefSpeed:
					accelDistFromCurSpeedToSelectSpeed, speedProfFromCurSpeedToSelectSpeed = SimpleTrajPlanning.getTravelDistFromBetweenSpeed(curSpeed, speed, RefAccel, dt)
					decelDistFromSelectSpeed, speedProfDecelFromSelectSpeed = SimpleTrajPlanning.getTravelDistFromBetweenSpeed(speed, 0, -RefDecel, dt)
					if dist > accelDistFromCurSpeedToSelectSpeed + decelDistFromSelectSpeed + earlyStoppingDist:
						speedProfAll += list(speedProfFromCurSpeedToSelectSpeed)
						remDist = dist - accelDistFromCurSpeedToSelectSpeed
						while remDist > decelDistFromSelectSpeed + speed*dt + earlyStoppingDist:
							speedProfAll.append(speed)
							remDist -= speed*dt
							isInFinalDecelPhase = False
						speedProfAll += list(speedProfDecelFromSelectSpeed)
						break
			else:
				speedProfAll += list(speedProfDecelFromCurSpeed)
		else:
			accelDistFromCurSpeedToMinSpeedToMove, speedProfFromCurSpeedToMinSpeedToMove = SimpleTrajPlanning.getTravelDistFromBetweenSpeed(curSpeed, MinSpeedToMove, np.inf, dt)
			decelDistFromMinSpeedToMove, speedProfileFromMinSpeedToMove = SimpleTrajPlanning.getTravelDistFromBetweenSpeed(MinSpeedToMove, 0, -RefDecel, dt)
			if dist > accelDistFromCurSpeedToMinSpeedToMove + decelDistFromMinSpeedToMove + earlyStoppingDist:
				speedProfAll += list(speedProfFromCurSpeedToMinSpeedToMove)
				remDist = dist - accelDistFromCurSpeedToMinSpeedToMove

				for speed in speedProfDecelFromRefSpeed:
					accelDistFromCurSpeedToSelectSpeed, speedProfFromCurSpeedToSelectSpeed = SimpleTrajPlanning.getTravelDistFromBetweenSpeed(MinSpeedToMove, speed, RefAccel, dt)
					decelDistFromSelectSpeed, speedProfDecelFromSelectSpeed = SimpleTrajPlanning.getTravelDistFromBetweenSpeed(speed, 0, -RefDecel, dt)
					if remDist > accelDistFromCurSpeedToSelectSpeed + decelDistFromSelectSpeed + earlyStoppingDist:
						speedProfAll += list(speedProfFromCurSpeedToSelectSpeed)
						remDist = remDist - accelDistFromCurSpeedToSelectSpeed
						while remDist > decelDistFromSelectSpeed + speed*dt + earlyStoppingDist:
							speedProfAll.append(speed)
							remDist -= speed*dt
							isInFinalDecelPhase = False
						speedProfAll += list(speedProfDecelFromSelectSpeed)
						break
			else:
				speedProfAll += list(speedProfDecelFromCurSpeed)
		speedProfAll.append(0)
		speedProfAll = np.array(speedProfAll)
		distFromSpeedProf = SimpleTrajPlanning.getDistFromSpeedProfile(speedProfAll, dt)[-1]

		if preMPCAccel is not None and preMPCAccel < 0 and nextSpeedCompensationByAccelFeedback > 0 and speedProfAll.shape[0] > 1:         # Only compensate for deceleration (nextSpeedCompensationByAccelFeedback < 0: decelerate less)
			nextSpeedCompensated = min(speedProfAll[1], max(minSpeedAfterCompensation, speedProfAll[1] - nextSpeedCompensationByAccelFeedback))
			greaterSpeedIdxList = np.where(speedProfAll > nextSpeedCompensated)[-1]
			if greaterSpeedIdxList.size > 0:
				correctUntilIdx = greaterSpeedIdxList[-1]
				speedProfAll[1:(correctUntilIdx+1)] = nextSpeedCompensated

		distFromSpeedProf2 = SimpleTrajPlanning.getDistFromSpeedProfile(speedProfAll, dt)[-1]
		return speedProfAll

	@staticmethod
	def getTrapezoidalSpeedProfile1(dist, curSpeed, RefAccelSpeedProfile, RefDecelSpeedProfile, minConstSpeedStep=5, dt=0.1):
		# Get acceleration/deceleration phase based on current speed
		decelDist = np.flip(SimpleTrajPlanning.getDistFromSpeedProfile(np.flip(RefDecelSpeedProfile), dt))      # no flip on the result distance
		closestDecelIdx = np.array(np.where(RefDecelSpeedProfile >= curSpeed)[0]).max()

		# Check first accel speed
		accelFirstNonZeroSpeed = min(RefAccelSpeedProfile[RefAccelSpeedProfile > 0])
		decelSpeedIdxForFirstNonZeroAccelSpeed = np.array(np.where(RefDecelSpeedProfile <= RefAccelSpeedProfile[-1])[0]).min()
		decelDistForFirstNonZeroAccelSpeed = decelDist[decelSpeedIdxForFirstNonZeroAccelSpeed]
		if dist - accelFirstNonZeroSpeed*dt < decelDist[closestDecelIdx] or dist < decelDistForFirstNonZeroAccelSpeed:       # Deceleration
			decelSpeedProfile = RefDecelSpeedProfile[min(closestDecelIdx+1,len(RefDecelSpeedProfile)-1):]
			accelSpeedProfile = np.empty(0)
		else:
			if curSpeed > RefAccelSpeedProfile[-1]:
				closestAccelIdx = RefAccelSpeedProfile.size - 1
			else:
				closestAccelIdx = np.array(np.where(RefAccelSpeedProfile >= curSpeed)[0]).min()
			decelSpeedProfile = RefDecelSpeedProfile
			accelSpeedProfile = RefAccelSpeedProfile[closestAccelIdx:]

		# Compute accel, decel distance array
		accelDist = SimpleTrajPlanning.getDistFromSpeedProfile(accelSpeedProfile, dt)
		decelDist = np.flip(SimpleTrajPlanning.getDistFromSpeedProfile(np.flip(decelSpeedProfile), dt))

		# Generate speed profile
		if accelSpeedProfile.shape[0] > 0:      # Match the accel speed to the biggest decel speed that smaller than accel speed
			decelSpeedIdxForAccelSpeed = np.full(accelSpeedProfile.shape[0], -1)
			for i in range(accelSpeedProfile.shape[0]):
				decelSpeedIdxSmaller = np.array(np.where(decelSpeedProfile <= accelSpeedProfile[i]))[0]
				if decelSpeedIdxSmaller.size == 0:
					decelSpeedIdxForAccelSpeed[i] = decelSpeedProfile.shape[0] - 1
				else:
					decelSpeedIdxForAccelSpeed[i] = np.array(np.where(decelSpeedProfile <= accelSpeedProfile[i])[0]).min()

			# Search for appropriate speed in accelSpeedProfile
			curAccelIdx = 0
			while curAccelIdx < accelSpeedProfile.shape[0] and dist >= accelDist[curAccelIdx] + decelDist[decelSpeedIdxForAccelSpeed[curAccelIdx]] + minConstSpeedStep*dt*accelSpeedProfile[curAccelIdx]:
				curAccelIdx += 1

			curAccelIdx = curAccelIdx - 1 if curAccelIdx > 0 else 0
			selectedDecelIdx = decelSpeedIdxForAccelSpeed[curAccelIdx]
			distRemain = dist - (accelDist[curAccelIdx] + decelDist[selectedDecelIdx])
			if distRemain >= 0 and accelSpeedProfile[curAccelIdx] > 0:
				constantSpeedStepNum = int(distRemain / (dt*accelSpeedProfile[curAccelIdx]))
				speedProf = np.concatenate([accelSpeedProfile[0:curAccelIdx+1], np.array([accelSpeedProfile[curAccelIdx]]*constantSpeedStepNum), decelSpeedProfile[decelSpeedIdxForAccelSpeed[curAccelIdx]:]], axis=None)
			else:
				speedProf = decelSpeedProfile[decelSpeedIdxForAccelSpeed[curAccelIdx]:]
		else:       # Decel to the preference speed (max speed in accel speed profile) then move at constant speed
			# Find distance to decel to preference speed
			decelSpeedIdxForPreferenceSpeed = np.array(np.where(decelSpeedProfile <= RefAccelSpeedProfile[-1])[0]).min()
			distToDecelToPreferenceSpeed = [0]
			for speed in decelSpeedProfile[:decelSpeedIdxForPreferenceSpeed]:
				distToDecelToPreferenceSpeed.append(distToDecelToPreferenceSpeed[-1] + dt*speed)

			distRemain = dist - (distToDecelToPreferenceSpeed[-1] + decelDist[decelSpeedIdxForPreferenceSpeed])
			constantSpeedStepNum = int(distRemain / (dt*RefAccelSpeedProfile[-1]))
			constantSpeedStepNum = 0 if constantSpeedStepNum < 0 else constantSpeedStepNum
			selectedDecelIdx = decelSpeedIdxForPreferenceSpeed
			speedProf = np.concatenate([decelSpeedProfile[:decelSpeedIdxForPreferenceSpeed], np.array([RefAccelSpeedProfile[-1]]*constantSpeedStepNum), decelSpeedProfile[decelSpeedIdxForPreferenceSpeed:]], axis=None)

		# # Compensate the remaining distance caused by discrete time step
		# resultDist = SimpleTrajPlanning.getDistFromSpeedProfile(speedProf, dt)
		# distRemainFineAdjust = dist - resultDist[-1]
		#
		# # Search for decel speed that can travel N steps within distRemainFineAdjust
		# curDecelIdxFineAdjust = decelSpeedProfile.shape[0] - 1
		# stepToTravelInFineAdjust = 5
		# while curDecelIdxFineAdjust >= selectedDecelIdx and distRemainFineAdjust >= stepToTravelInFineAdjust * dt*decelSpeedProfile[curDecelIdxFineAdjust]:
		# 	curDecelIdxFineAdjust -= 1
		# curDecelIdxFineAdjust = curDecelIdxFineAdjust + 1 if curDecelIdxFineAdjust < decelSpeedProfile.shape[0] - 1 else decelSpeedProfile.shape[0] - 1
		# if decelSpeedProfile[curDecelIdxFineAdjust] > 0:
		# 	stepToTravelInFineAdjustActual = int(distRemainFineAdjust / (dt*decelSpeedProfile[curDecelIdxFineAdjust]))
		# else:
		# 	stepToTravelInFineAdjustActual = 0
		#
		# speedProfFineAdjust = np.concatenate([speedProf[:-(len(decelSpeedProfile)-1-curDecelIdxFineAdjust)], np.array([decelSpeedProfile[curDecelIdxFineAdjust]]*stepToTravelInFineAdjustActual), speedProf[-(len(decelSpeedProfile)-1-curDecelIdxFineAdjust):]], axis=None)
		#
		# # Check distance again
		# resultDistFineAdjust = SimpleTrajPlanning.getDistFromSpeedProfile(speedProfFineAdjust, dt)
		speedProf[0] = curSpeed
		return speedProf


class VehicleState:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v


class LinearMPC:
    def __init__(self):
        self.A = None
        self.B = None
        self.C = None
        self.x0 = None
        self.xref = None

        self.NX = 4
        self.NU = 2
        self.N = 10

        # FIXED PARAMS
        # ## Straight Path Scenario Weight
        # self.R_straight = np.diag([0.01, 0.01])
        # self.Rd_straight = np.diag([0.01, 0.00])
        # self.Q_straight = np.diag([1.5, 1.5, 10.0, 0.75])

        # ## Curve Path Scenario Weight
        # self.R_curve = np.diag([0.01, 0.00])
        # self.Rd_curve = np.diag([0.01, 0.05])
        # self.Q_curve = np.diag([1.5, 1.5, 10.0, 0.75])

        # TESTING 1
        ## Straight Path Scenario Weight
        self.R_straight = np.diag([0.01, 0.00147003173])
        self.Rd_straight = np.diag([0.01, 0.05])
        self.Q_straight = np.diag([2.0, 2.0, 12.0, 0.75])

        ## Curve Path Scenario Weight
        self.R_curve = np.diag([0.01, 0.00147003173])
        self.Rd_curve = np.diag([0.01, 0.05])
        self.Q_curve = np.diag([2.0, 2.0, 12.0, 0.75])

        # # TESTING 2
        # ## Straight Path Scenario Weight
        # self.R_straight = np.diag([0.01, 0.0])
        # self.Rd_straight = np.diag([0.1, 0.1])
        # self.Q_straight = np.diag([2.0, 2.0, 10.0, 0.75])

        # ## Curve Path Scenario Weight
        # self.R_curve = np.diag([0.01, 0.0])
        # self.Rd_curve = np.diag([0.1, 0.1])
        # self.Q_curve = np.diag([2.0, 2.0, 10.0, 0.75])

        # # TESTING 3
        # ## Straight Path Scenario Weight
        # self.R_straight = np.diag([0.01, 0.0])
        # self.Rd_straight = np.diag([0.01, 0.1])
        # self.Q_straight = np.diag([2.0, 2.0, 12.0, 0.75])

        # ## Curve Path Scenario Weight
        # self.R_curve = np.diag([0.01, 0.0])
        # self.Rd_curve = np.diag([0.01, 0.1])
        # self.Q_curve = np.diag([2.0, 2.0, 12.0, 0.75])

        # # TESTING 4
        # ## Straight Path Scenario Weight
        # self.R_straight = np.diag([0.01, 0.0])
        # self.Rd_straight = np.diag([0.01, 0.1])
        # self.Q_straight = np.diag([2.0, 2.0, 10.0, 0.75])

        # ## Curve Path Scenario Weight
        # self.R_curve = np.diag([0.01, 0.0])
        # self.Rd_curve = np.diag([0.01, 0.1])
        # self.Q_curve = np.diag([2.0, 2.0, 10.0, 0.75])


        # Set the goal parameter here
        self.GOAL_DIS_X = 0.5
        self.GOAL_DIS_Y = 0.5
        self.STOP_MPC_X = 0 # Set this value larger or equal to the X axis Goal Distance
        self.STOP_MPC_Y = 0 # Set this value larger or equal to the Y axis Goal Distance
        self.STOP_SPEED = 0.028 # [m/s]
        self.MAX_TIME = 500.0

        self.MAX_CURVE = 0.2 # [m]

        ####################### Change the speed related parameters here ###################################
        # self.TARGET_SPEED = 0.888 # [m/s]
        self.TARGET_SPEED = 1.111 # [m/s]
        self.ACCEL_REF = 1.0 # [m/s^2]
        self.DECEL_REF = 1.0 # [m/s^2]
        self.MIN_SPEED_TO_MOVE = 0.555 # [m/s]
        self.EARLY_STOPPING_DIST = 0 #[m]

        # 230117 added
        self.COMPENSE_ACCEL = 1 # [m/s^2]
        self.MIN_SPEED_AFTER_COMP = 0.555 # [m/s]
        self.MPC_THRESHOLD_TO_STOP = 0.095 # [m/s]

        # Golf-cart params
        self.WB = 2.48
        self.MAX_STEER = np.deg2rad(29.0)
        self.MAX_DSTEER = np.deg2rad(12.5)
        self.INIT_STEER_ANGLE = np.deg2rad(-26.0)
        self.INIT_STEER_CUSP = np.deg2rad(6.0)

        self.MAX_SPEED = 2.77 # [m/s]
        self.MIN_SPEED = -2.77 # [m/s]
        self.MAX_ACCEL = 1.0
        self.MAX_DECEL = -2.0

        self.DT = 0.1  # [s] Control Period
        self.PARKING_MODE = False
        self.PATH_CHANGE = False
        self.CUSP_POINT_DIST = 0.5 #[m]

        # ---------------Sungil update---------------230112
        self.prev_oa = None
        self.nextSpeedCompensationByAccelFeedbackRatio = 1
        self.minSpeedAfterCompensation = 0
        # ---------------End update---------------230112

        # # Simulator params
        # self.WB = 2.48  # wheelbase of vehicle
        # self.MAX_STEER = np.deg2rad(29.0)
        # self.MAX_DSTEER = np.deg2rad(6.6)

        self.UMIN = np.array([self.MAX_ACCEL * -1.0, self.MAX_STEER * -1.0])
        self.UMAX = np.array([self.MAX_ACCEL, self.MAX_STEER])
        self.XMIN = np.array([-np.inf, -np.inf, self.MIN_SPEED, -np.inf])
        self.XMAX = np.array([np.inf, np.inf, self.MAX_SPEED, np.inf])

        # Time delay parameters
        self.TIME_DELAY_THROTTLE = 0.2
        self.TIME_DELAY_STEER = 0.2
        self.CONTROL_IDX_STEER = 0
        self.CONTROL_IDX_THROTTLE = 0

        self.x = None
        self.u = None
        self.yaw = None
        self.sp = None

        self.ref_x = None
        self.ref_y = None
        self.ref_yaw = None

        self.goal = None
        self.direction = None
        self.prev_track = 0
        self.mpc_start = True
        self.speed_profile_sign = None
        self.reverse = False
        self.new_path = None
        self.track_num = 1
        self.time_step = 0

        self.cusp_detected = False
        self.xref_tmp = None
        self.cusp_idx = 0
        self.cusp_point_idx = None
        self.cusp_point = None
        self.target_reset = True
        self.goal_reached = False
        self.prev_path = np.zeros((self.N-1, 1))
        self.count_wait = 0
        self.stop = False
        self.STOP_TIME = 20
        self.stop_count = 0
        self.show_animation = True

        self.objective = 0
        self.motion_planner = None

    def setup_mpc(self):
        # Setup MPC
        self.x = cp.Variable((self.NX, self.N))
        self.u = cp.Variable((self.NU, self.N - 1))

    def setup_parking_mpc(self):
        #################################### PARKING STEP PARAMETERS SETTING ################################################
        self.EARLY_STOPPING_DIST = 0
        self.TARGET_SPEED = 0.4 # [m/s]
        self.MIN_SPEED_TO_MOVE = 0.35 # [m/s]

    def get_linear_model(self, v, psi, delta):
        A = np.array(
            [
                [1, 0, self.DT * np.cos(psi), - self.DT * v * np.sin(psi)],
                [0, 1, self.DT * np.sin(psi), self.DT * v * np.cos(psi)],
                [0, 0, 1, 0],
                [0, 0, self.DT * np.tan(delta) / self.WB, 1],
            ]
        )
        B = np.array(
            [
                [0, 0],
                [0, 0],
                [self.DT, 0],
                [0, self.DT * v / (self.WB * np.cos(delta) ** 2)],
            ]
        )
        C = np.array(
            [
                self.DT * v * np.sin(psi) * psi,
                - self.DT * v * np.cos(psi) * psi,
                0,
                - self.DT * v * delta / (self.WB * np.cos(delta) ** 2),
            ]
        )
        return A, B, C

    def state_update(self, state, a, delta):
        new_state = solve_ivp(
            self.step, (0, self.DT), [state.x, state.y, state.v, state.yaw], args=(state, a, delta)
        ).y[:, -1]

        state.x = new_state[0]
        state.y = new_state[1]
        state.v = new_state[2]
        state.yaw = new_state[3]

        # Speed limit
        if state.v > self.MAX_SPEED:
            state.v = self.MAX_SPEED
        elif state.v < self.MIN_SPEED:
            state.v = self.MIN_SPEED
        return state

    def step(self, y0, t, state, a, delta):
        x_dot = state.v * np.cos(state.yaw)
        y_dot = state.v * np.sin(state.yaw)
        v_dot = a
        psi_dot = state.v * np.tan(delta) / self.WB
        return np.array([x_dot, y_dot, v_dot, psi_dot])

    def predict_motion(self, x0, oa, od, xref):
        xbar = xref * 0.0
        for i, _ in enumerate(x0):
            xbar[i, 0] = x0[i]

        state = VehicleState(x=x0[0], y=x0[1], yaw=x0[3], v=x0[2])
        for (ai, di, i) in zip(oa, od, range(1, self.N)):
            state = self.state_update(state, ai, di)
            xbar[0, i] = state.x
            xbar[1, i] = state.y
            xbar[2, i] = state.v
            xbar[3, i] = state.yaw

        return xbar

    def calc_linear_mpc_control(self, xref, x0, oa, od, ck):
        dref = np.zeros((1, self.N - 1))
        if oa is None or od is None:
            oa = [0.0] * (self.N - 1)
            od = [0.0] * (self.N - 1)

        xbar = self.predict_motion(x0, oa, od, xref)
        oa, od, ox, oy, oyaw, ov = self.linear_mpc(xref, xbar, x0, od, dref, ck)

        return oa, od, ox, oy, oyaw, ov

    def determine_initial_direction(self, cyaw, vehicle_state):
        heading_diff = np.abs(cyaw[0] - vehicle_state[3])
        if (heading_diff > np.pi / 4) and (heading_diff < 3 * np.pi / 2):
            self.direction = -1
        else:
            self.direction = 1
        pass

    # Main MPC function
    def linear_mpc(self, xref, xbar, x0, prev_od, dref, ck):
        # print(f"Previous Delta: {prev_od}")
        cost = 0.0
        constraints = []
        last_curve = np.abs(ck[-1])

        # Set the weight based on the path curvature
        Q = self.Q_straight if last_curve < 0.5 * self.MAX_CURVE else self.Q_curve
        R = self.R_straight if last_curve < 0.5 * self.MAX_CURVE else self.R_curve
        Rd = self.Rd_straight if last_curve < 0.5 * self.MAX_CURVE else self.Rd_curve
        Qf = Q

        # Q = self.Q_curve
        # R = self.R_curve
        # Rd = self.Rd_curve
        # Qf = Q

        # print(F"Q weights: {Q}")
        # print(F"R weights: {R}")
        # print(F"Rd weights: {Rd}")
        # print(F"Qf weights: {Qf}")

        # Time delay compensation
        self.CONTROL_IDX_STEER = int(round(self.TIME_DELAY_STEER / self.DT))
        self.CONTROL_IDX_ACCEL = int(round(self.TIME_DELAY_THROTTLE / self.DT))

        for t in range(self.N - 1):
            cost += cp.quad_form(self.u[:, t], R)

            if t != 0:
                cost += cp.quad_form(xref[:, t] - self.x[:, t], Q)

            self.A, self.B, self.C = self.get_linear_model(
                xbar[2, t], xbar[3, t], dref[0, t])
            constraints += [self.x[:, t + 1] == self.A @ self.x[:, t] + self.B @ self.u[:, t] + self.C]

            if t < (self.N - 2):
                cost += cp.quad_form(self.u[:, t + 1] - self.u[:, t], Rd)
                constraints += [cp.abs(self.u[1, t + 1] - self.u[1, t]) <=
                                self.MAX_DSTEER * self.DT]

        # Final horizon state constraints
        cost += cp.quad_form(xref[:, self.N - 1] - self.x[:, self.N - 1], Qf)

        # print("Current Speed", x0[2])
        # If the speed below the golf cart threshold
        if np.abs(x0[2]) < self.MIN_SPEED_TO_MOVE:
            self.MAX_ACCEL = 5.0
            self.MAX_DECEL = -5.0
        elif np.abs(x0[2]) >= self.MIN_SPEED_TO_MOVE:
            self.MAX_ACCEL = 1.5
            self.MAX_DECEL = -2.0

        # print(f"MAXIMUM ACCELERATION {self.MAX_ACCEL}")
        # print(f"MAXIMUM DECELERATION {self.MAX_DECEL}")

        constraints += [self.x[:, 0] == x0]
        constraints += [self.x[2, :] <= self.MAX_SPEED]
        constraints += [self.x[2, :] >= self.MIN_SPEED]
        constraints += [self.u[0, :] <= self.MAX_ACCEL]
        constraints += [self.u[0, :] >= self.MAX_DECEL]
        constraints += [cp.abs(self.u[1, :]) <= self.MAX_STEER]

        # Activate this constraint during parking
        constraints += [cp.abs(self.u[1, 0] - prev_od[self.CONTROL_IDX_STEER]) <= self.MAX_DSTEER * self.DT]

        prob = cp.Problem(cp.Minimize(cost), constraints)

        prob.solve(solver=cp.OSQP, verbose=False, warm_start=True)

        if prob.status == cp.OPTIMAL or prob.status == cp.OPTIMAL_INACCURATE:
            ox, oy, ov, oyaw = self.x.value[0, :].flatten(), self.x.value[1, :].flatten(), \
                               self.x.value[2, :].flatten(), self.x.value[3, :].flatten()
            # Make sure the control input is not out of the limit
            oa, odelta = np.clip(self.u.value[0, :], self.MAX_DECEL, self.MAX_ACCEL), np.clip(
                self.u.value[1, :], -self.MAX_STEER, self.MAX_STEER)
            # Steering smoother
            odelta = self.smooth_steering(prev_od[self.CONTROL_IDX_STEER], odelta, self.N - 1)

        else:
            print("Error: Cannot solve MPC")
            oa, odelta, ox, oy, oyaw, ov = None, None, None, None, None, None
        
        # print("X Horizon", ox)
        # print("Y Horizon", oy)
        # print("Yaw Horizon", oyaw)
        # print("V Horizon", ov)

        return oa, odelta, ox, oy, oyaw, ov

    def solve_mpc(self, vehicle_state, path_data, oa, od):
        # Obtain prev control input
        if oa is None or od is None:
            a = np.zeros((1, self.N - 1)).flatten()
            delta = np.zeros((1, self.N - 1)).flatten()
        else:
            a = oa
            delta = np.array(od, dtype=np.float32)

        current_state = [vehicle_state.x, vehicle_state.y, vehicle_state.v, vehicle_state.yaw]

        same_path = np.array_equal(path_data, self.prev_path)
        # print(path_data)
        if not same_path:
            ref_x, ref_y, ref_yaw = path_data[0], path_data[1], path_data[2]
            path_init = np.array([ref_x, ref_y, ref_yaw]).T
            self.goal = [ref_x[-1], ref_y[-1]]
            self.motion_planner = SimpleTrajPlanning(path_init, self.TARGET_SPEED,
                                                     self.ACCEL_REF, self.DECEL_REF,
                                                     self.MIN_SPEED_TO_MOVE, 0,
                                                     self.EARLY_STOPPING_DIST,
                                                     self.MAX_SPEED, 
                                                     self.nextSpeedCompensationByAccelFeedbackRatio, 
                                                     self.minSpeedAfterCompensation, self.DT,
                                                     self.N, self.STOP_TIME)
            

            # ---------------Sungil update---------------230117
            if any(self.motion_planner.cuspPointIdxList):
            # ---------------End update---------------230117
                self.cusp_point = [path_init[i, :] for i in self.motion_planner.cuspPointIdxList]

            self.prev_path = path_data

            # Set the initial steering angle for MPC Constraints
            if self.PARKING_MODE:
                  delta = np.ones((1, self.N - 1)).flatten() * self.INIT_STEER_ANGLE
                  self.PARKING_MODE = False
                  self.PATH_CHANGE = False

            # print("OK")
            # ref_x, ref_y, ref_yaw = path_data[0], path_data[1], path_data[2]
            # path_init = np.array([ref_x, ref_y, ref_yaw]).T
            # smoothed_path = self.path_smoother(self.path_smoother_forward(path_init), self.path_smoother_backward(path_init))
            # smoothed_path = np.array(smoothed_path, dtype = np.float32)
            # ref_x, ref_y, ref_yaw = smoothed_path[0], smoothed_path[1], smoothed_path[2]
            # np.savetxt('/home/vialab/exhibition_ws/src/ackerman_ros_robot_gazebo_simulation/custom_controllers/ackermann-drive-teleop/script_for_mpc_rev/data/smoothed_path_230107_01.txt', smoothed_path, delimiter=',') 
            # # print(smoothed_path)
            # # smoothed_path = np.array(smoothed_path, dtype = np.float32)

        # Obtain the path from the motion planner

        # ---------------Sungil update---------------230112
        if self.prev_oa != None:
            xref = self.motion_planner.step(current_state[0:2], current_state[2], self.prev_oa)
        else:
            xref = self.motion_planner.step(current_state[0:2], current_state[2], 0)
        # ---------------End update---------------230112
        
        # xref = self.motion_planner.step(current_state[0:2], current_state[2])
        traj_order = [0, 1, 3, 2]
        idx = np.empty_like(traj_order)
        idx[traj_order] = np.arange(len(traj_order))
        xref[:] = xref[:, idx]
        xref = xref[0:self.N, :].T

        if xref is not None:
            self.xref_tmp = xref

        print("Stopping count ", self.motion_planner.stopStep)

        # Calculate curvature for the node
        ck = self.calculate_curvature(xref[0, :], xref[1, :])

        # Calculate the goal and stop flag distance
        isstop = (abs(vehicle_state.v) <= self.STOP_SPEED)
        isgoal = self.calc_goal_distance(current_state=current_state)

        # Deciding whether MPC needs to be stopped or not
        mpc_stop = self.stop_mpc_flag(current_state=current_state)

        self.halt_mpc = False
        if mpc_stop:
            if np.abs(current_state[2]) < self.MPC_THRESHOLD_TO_STOP:
                self.halt_mpc = True
            else:
                self.halt_mpc = False

        # check if the vehicle is reached the goal
        if (isstop and isgoal) or xref is None:
            print("############################### GOAL REACHED #####################################")
            self.PARKING_MODE = True
            if self.PARKING_MODE and self.PATH_CHANGE:
                oa, odelta, ov = [0.0], [self.INIT_STEER_ANGLE], [0.0]
            else:
                oa, odelta, ov = [0.0], [0.0], [0.0]

            ox = np.ones((1, self.N)).flatten() * current_state[0]
            oy = np.ones((1, self.N)).flatten() * current_state[1]
            ov = float(ov[0])

            # ---------------Sungil update---------------230117
            self.prev_oa = oa[self.CONTROL_IDX_THROTTLE]
            self.setup_parking_mpc()
            print(self.TARGET_SPEED)
            # ---------------End update---------------230117

            return ox, oy, oa, odelta, ov, isstop, isgoal, xref
            
        elif self.cusp_point is not None and self.motion_planner.stopStep != 0:
            if (self.cusp_point[self.cusp_idx][0] != self.goal[0]) and (self.cusp_point[self.cusp_idx][1] != self.goal[1]):
                stop_at_cusp_point = self.calc_stopping_distance(current_state)
                if stop_at_cusp_point and self.motion_planner.pathIdx + 1 < len(self.motion_planner.pathList):
                    print("############################# STOPPING AT CUSP POINT ################################")
                    # Initialize the steering angle to compensate the change at the segment change
                    print("Steering Init!!!")
                    pathIdx = self.motion_planner.pathIdx
                    post_path = self.motion_planner.pathList[pathIdx]
                    # ck_path = self.calc_curve_rev(ax=post_path[:, 0], ay=post_path[:, 1])
                    # odelta = self.init_steer(ck_path, self.TARGET_SPEED, self.DT, self.N - 1)

                    oa, ov = [0.0], [0.0]
                    ov = float(ov[0])

                    # ---------------Sungil update---------------230112
                    self.prev_oa = oa[self.CONTROL_IDX_THROTTLE]
                    # ---------------End update---------------230112

                    # # Stanley
                    # odelta = self.init_steer_stanley(current_state, post_path, self.DT, self.N - 1)
                    # Curve init
                    ox = np.ones((1, self.N)).flatten() * current_state[0]
                    oy = np.ones((1, self.N)).flatten() * current_state[1]
                    odelta = np.ones((1, self.N - 1)).flatten() * self.INIT_STEER_CUSP
                    if self.cusp_point and self.motion_planner.stopStep == 0:
                        if self.cusp_idx + 1 < len(self.motion_planner.cuspPointIdxList):
                            self.cusp_idx += 1
                            
                    return ox, oy, oa, odelta, ov, isstop, isgoal, xref

        # Calculate the MPC (During normal operation)
        if not self.halt_mpc:
            start = time.time()
            oa, odelta, ox, oy, oyaw, ov = self.calc_linear_mpc_control(xref, current_state, a, delta, ck)
            print(f"Total time for MPC {time.time() - start}")
            # ---------------Sungil update---------------230112
            self.prev_oa = oa[self.CONTROL_IDX_THROTTLE]
            # ---------------End update---------------230112
        # Halt the MPC (if the speed is too low and already inside the mpc stopping area)
        else:
            print("############################# MPC HALTED ################################")
            oa, odelta, ov = [0.0], [0.0], [0.0]
            ox = np.ones((1, self.N)).flatten() * current_state[0]
            oy = np.ones((1, self.N)).flatten() * current_state[1]
            ov = float(ov[0])
            # ---------------Sungil update---------------230112
            self.prev_oa = oa
            # ---------------End update---------------230112

        ov = current_state[2] + self.DT * oa[self.CONTROL_IDX_THROTTLE]

        return ox, oy, oa, odelta, ov, isstop, isgoal, xref

    def calc_goal_distance(self, current_state):
        dx = np.abs(current_state[0] - self.goal[0])
        dy = np.abs(current_state[1] - self.goal[1]) 
        print(dx, dy)
        isgoal_x = (dx <= self.GOAL_DIS_X)
        isgoal_y = (dy <= self.GOAL_DIS_Y)
        isgoal = isgoal_x and isgoal_y
        return isgoal

    def calc_stopping_distance(self, current_state):
        dx = np.abs(current_state[0] - self.cusp_point[self.cusp_idx][0])
        dy = np.abs(current_state[1] - self.cusp_point[self.cusp_idx][1])
        d = math.hypot(dx, dy)
        print(f"Distance Cusp {d}")
        stop_at_cusp_point = (d <= self.CUSP_POINT_DIST)
        return stop_at_cusp_point

    def stop_mpc_flag(self, current_state):
        dx = np.abs(current_state[0] - self.goal[0])
        dy = np.abs(current_state[1] - self.goal[1]) 
        isgoal_x = (dx <= self.STOP_MPC_X)
        isgoal_y = (dy <= self.STOP_MPC_Y)
        stop_mpc = isgoal_x and isgoal_y
        return stop_mpc

    def init_steer(self, ck, speed_target, motion_res, control_horizon):
        time_to_max = self.MAX_STEER / self.MAX_DSTEER
        # TODO: Need to define the speed target
        dist_judge = speed_target * time_to_max
        n_value = int(dist_judge / motion_res)
        curv_avg = sum(ck[0:n_value]) / n_value
        od_init = np.sign(curv_avg) * self.MAX_STEER if np.abs(curv_avg) >= 0.5 * self.MAX_CURVE else 0
        return [od_init] * control_horizon

    def init_steer_stanley(self, cur_pose, path, speed_target, control_horizon):
        # Gain
        k = 0.78
        x_cur, y_cur, yaw_cur = cur_pose[0], cur_pose[1], cur_pose[3]
        # print(path)
        x1, y1 = path[0, 0], path[0, 1]
        x2, y2 = path[1, 0], path[1, 1]
        yaw1 = path[1, 2]

        # Calc the Cross-Track-Error
        e_num = (x2 - x1) * (y1 - y_cur) - (x1 - x_cur) * (y2 - y1)
        e_denom = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        e = e_num / e_denom

        # Find the heading error
        psi_e = yaw1 - yaw_cur

        # Steering correction
        theta_ctrl = math.atan2(speed_target, k * e)
        od_init = psi_e + theta_ctrl
        od_init = np.clip(od_init, self.MAX_STEER, -self.MAX_STEER)
        return [od_init] * control_horizon

    @staticmethod
    def calc_curve_rev(ax, ay):
        ck = []
        # Calculate first derivative
        dx_val = np.diff(ax)
        dy_val = np.diff(ay)
        dx = np.insert(dx_val, 0, dx_val[0]).tolist()
        dy = np.insert(dy_val, 0, dy_val[0]).tolist()
        # Calculate second derivative
        ddx_val = np.diff(dx)
        ddy_val = np.diff(dy)
        ddx = np.insert(ddx_val, 0, ddx_val[0]).tolist()
        ddy = np.insert(ddy_val, 0, ddy_val[0]).tolist()
        for i in range(len(ax)):
            # Calculate yaw and curvature
            if (dx[i] ** 2 + dy[i] ** 2 == 0.0):
                ck.append(0.0)
            else:
                calc_curve = (ddy[i] * dx[i] - ddx[i] * dy[i]) / ((dx[i] ** 2 + dy[i] ** 2) ** (3 / 2))
                ck.append(calc_curve)
        return ck

    @staticmethod
    def calculate_curvature(x, y):
        dx = np.gradient(x)
        dy = np.gradient(y)
        ddx = np.gradient(dx)
        ddy = np.gradient(dy)
        curvature = (ddy * dx - ddx * dy) / np.power(dx ** 2 + dy ** 2, 3/2)
        curvature = np.nan_to_num(curvature, 0)
        return curvature

    @staticmethod
    def smooth_yaw(yaw):
        for i in range(len(yaw) - 1):
            dyaw = yaw[i + 1] - yaw[i]

            while dyaw >= math.pi / 2.0:
                yaw[i + 1] -= math.pi * 2.0
                dyaw = yaw[i + 1] - yaw[i]

            while dyaw <= -math.pi / 2.0:
                yaw[i + 1] += math.pi * 2.0
                dyaw = yaw[i + 1] - yaw[i]

        return yaw

    @staticmethod
    def calculate_cross_tracK_error(cx, cy, cyaw, state):
        dx = [state.x - icx for icx in cx]
        dy = [state.y - icy for icy in cy]

        crosstrack_error = [abs(math.cos(icyaw) * idx + math.sin(icyaw) * idy) for (icyaw, idx, idy) in
                            zip(cyaw, dx, dy)]

        return crosstrack_error

    @staticmethod
    def pi_2_pi(angle):
        while angle > math.pi:
            angle = angle - 2.0 * math.pi

        while angle < -math.pi:
            angle = angle + 2.0 * math.pi

        return angle

    @staticmethod
    def smooth_steering(prev_od, od, pred_hor):
        del_sum = np.sum(od)
        del_avg = (prev_od + del_sum) / (pred_hor + 1)

        # Steering speed
        del_steer = 2 * (del_avg - prev_od) / pred_hor
        new_od = [prev_od + (i + 1) * del_steer for i in range(pred_hor)]

        return new_od

    # ----------------------------------Sungil's Code----------------------------------
    @staticmethod
    def path_smoother_forward(original_path):
    # x, y, heading
    
        forward_path = [[0 for i in range(len(original_path[j]))] for j in range(len(original_path))]

        alpha = 0.98

        forward_path[0] = [original_path[0][0], original_path[0][1], original_path[0][2]]

        for i in range(1, len(original_path)):

            dist = ((original_path[i - 1][0] - original_path[i][0]) ** 2 + (original_path[i - 1][1] - original_path[i][1]) ** 2) ** 0.5
            tempAngle = math.atan2(original_path[i][1] - original_path[i - 1][1], original_path[i][0] - original_path[i - 1][0])

            if i == 1:
                nextAngle = tempAngle
            else:
                nextAngle = alpha * forward_path[i - 1][2] + (1 - alpha) * tempAngle
            
            point = [math.cos(nextAngle) * dist, math.sin(nextAngle) * dist]

            forward_path[i][0] = forward_path[i - 1][0] + point[0]
            forward_path[i][1] = forward_path[i - 1][1] + point[1]
            forward_path[i][2] = nextAngle

        return forward_path

    @staticmethod
    def path_smoother_backward(_original_path):
        # x, y, heading
        original_path = [[_original_path[len(_original_path) - i - 1][j] for j in range(len(_original_path[i]))] for i in range(len(_original_path))]

        backward_path = [[0 for i in range(len(original_path[j]))] for j in range(len(original_path))]

        alpha = 0.98

        backward_path[0] = [original_path[0][0], original_path[0][1], 3.141592]

        for i in range(1, len(original_path)):

            dist = ((original_path[i - 1][0] - original_path[i][0]) ** 2 + (original_path[i - 1][1] - original_path[i][1]) ** 2) ** 0.5
            tempAngle = math.atan2(original_path[i][1] - original_path[i - 1][1], original_path[i][0] - original_path[i - 1][0])
            if abs(backward_path[i - 1][2] - tempAngle) >= 3.141592:
                tempAngle += 2 * 3.141592

            if i == 1:
                nextAngle = 3.141592
            else:
                nextAngle = alpha * backward_path[i - 1][2] + (1 - alpha) * tempAngle
            
            point = [math.cos(nextAngle) * dist, math.sin(nextAngle) * dist]

            backward_path[i][0] = backward_path[i - 1][0] + point[0]
            backward_path[i][1] = backward_path[i - 1][1] + point[1]
            backward_path[i][2] = nextAngle

        return backward_path

    @staticmethod
    def path_smoother(forward_path, _backward_path):
        if len(forward_path) != len(_backward_path):
            print("Not matched path length\n")
            return False

        backward_path = [[_backward_path[len(_backward_path) - i - 1][j] for j in range(len(_backward_path[i]))] for i in range(len(_backward_path))]

        smoothed_path = [[0 for i in range(len(forward_path[j]))] for j in range(len(forward_path))]

        for i in range(len(smoothed_path)):
            x = (forward_path[i][0] + backward_path[i][0]) / 2
            y = (forward_path[i][1] + backward_path[i][1]) / 2

            smoothed_path[i][0:2] = [x, y]

            if i > 0:
                
                smoothed_path[i - 1][2] = math.atan2(smoothed_path[i][1] - smoothed_path[i - 1][1], smoothed_path[i][0] - smoothed_path[i - 1][0])
            
            if i == len(smoothed_path) - 1:
                smoothed_path[i][2] = smoothed_path[i - 1][2]

        return smoothed_path
