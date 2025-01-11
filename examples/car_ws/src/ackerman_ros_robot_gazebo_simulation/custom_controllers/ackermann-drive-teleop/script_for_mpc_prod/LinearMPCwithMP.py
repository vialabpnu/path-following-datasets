import time
import csv
import math
import logging

import numpy as np
import matplotlib.pyplot as plt
import cvxpy as cp


import GeometryHelper
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
    def __init__(self, N=9, type="uniform"):
        self.A = None
        self.B = None
        self.C = None
        self.x0 = None
        self.xref = None

        self.NX = 4
        self.NU = 2
        self.N = N # 1 (Current State) + 10 (Future State/Prediction Horizon)
        print(f"self.N: {self.N}")

        # FIXED PARAMS
        # TESTING 1
        # self.R_curve = np.diag([0.01, 0.00147003173])
        # self.Rd_curve = np.diag([0.01, 0.05])
        # self.Q_curve = np.diag([2.0, 2.0, 12.0, 0.75])
        
        self.R_straight = np.diag([0.01, 0.01])
        self.Rd_straight = np.diag([0.01, 0.00])
        self.Q_straight = np.diag([5.0, 5.0, 1.0, 1.0])
        
        self.R_curve = np.diag([0.01, 0.0])
        self.Rd_curve = np.diag([0.01, 0.05])
        # self.Q_curve = np.diag([2.0, 2.0, 10.0, 0.75])
        self.Q_curve = np.diag([5.0, 5.0, 1.0, 1.0])

        # Set the goal parameter here
        self.GOAL_DIS_X = 0.5
        self.GOAL_DIS_Y = 0.5
        self.STOP_MPC_X = 1.00 # Set this value larger or equal to the X axis Goal Distance
        self.STOP_MPC_Y = 1.00 # Set this value larger or equal to the Y axis Goal Distance
        # self.STOP_SPEED = 0.028 # [m/s]
        self.STOP_SPEED = 0.075 # [m/s]
        self.MAX_TIME = 500.0

        self.MAX_CURVE = 0.2 # [m]

        ####################### Change the speed related parameters here ###################################
        # self.TARGET_SPEED = 0.888 # [m/s]
        self.TARGET_SPEED = 4.4/3.6 #1.111 # [m/s]
        self.ACCEL_REF = 0.6 #1.0 # [m/s^2]
        self.DECEL_REF = 0.6 #1.0 # [m/s^2]
        self.MIN_SPEED_TO_MOVE = 0.555 # [m/s]
        self.EARLY_STOPPING_DIST = 0 #[m]

        # 230117 added
        self.COMPENSE_ACCEL = 1 # [m/s^2]
        self.MIN_SPEED_AFTER_COMP = 0.555 # [m/s]
        self.MPC_THRESHOLD_TO_STOP = 0.1 #0.095 # [m/s]

        # Golf-cart params
        self.WB = 2.48
        self.MAX_STEER = 0.444 # [rad]
        self.MAX_DSTEER = 0.14 # [rad/s]
        self.INIT_STEER_ANGLE = np.deg2rad(-26.0)
        self.INIT_STEER_CUSP = np.deg2rad(6.0)

        self.MAX_ACCEL = 1.5
        self.MAX_DECEL = -1.5
        self.CUR_SPEED = 0.0

        # Time Sampling Parameters
        self.DT = 0.1  # [s] Time Step
        self.DT_array = np.array([self.DT] * (self.N-1))
        self.use_rl_to_determine_dt = False
        
        self.PARKING_MODE = False
        self.PATH_CHANGE = False
        self.VAR_DT = True
        self.CUSP_POINT_DIST = 0.5 #[m]
        
        self.SCALED_ACCEL = 1.5
        self.MAX_SPEED = self.TARGET_SPEED + self.DT * self.SCALED_ACCEL # [m/s]
        self.MIN_SPEED = -self.TARGET_SPEED - self.DT * self.SCALED_ACCEL # [m/s]

        self.prev_oa = None
        self.nextSpeedCompensationByAccelFeedbackRatio = 1
        self.minSpeedAfterCompensation = 0

        self.UMIN = np.array([self.MAX_ACCEL * -1.0, self.MAX_STEER * -1.0])
        self.UMAX = np.array([self.MAX_ACCEL, self.MAX_STEER])
        self.XMIN = np.array([-np.inf, -np.inf, self.MIN_SPEED, -np.inf])
        self.XMAX = np.array([np.inf, np.inf, self.MAX_SPEED, np.inf])

        # Time delay parameters
        self.TIME_DELAY_THROTTLE = 0.0 #0.26
        self.TIME_DELAY_STEER = 0.0 #0.16836
        self.ALPHA = None
        self.BETA = None
        
        # Optimization cost
        self.prev_optim_cost = 0
        self.optim_cost = 0
        
        # Lateral Error
        self.lat_error = 0.0
        
        # Non-uniform time step parameters
        self.type = type
        if self.VAR_DT:
            self.DT_MIN = self.DT
            self.DT_MAX = 0.8
            self.Nn_prev = 1
            self.Nn_current = self.Nn_prev
            self.Nn_min = 1
            self.Nn_max = 8
            self.Nn = self.Nn_max
            self.DT_array = None
            self.DT_array = self.scale_dt(type=self.type)
            # # K-Uniform MPC (FSM-MPC 6)
            # self.K_SCALE = 8
            # self.total_horizon = 8
            # self.sparse_horizon = 2
            # # self.DT_array = np.array([self.DT_MIN * self.K_SCALE] * (self.N - 1))
            # self.DT_array = np.array([self.DT_MIN] * (self.total_horizon - self.sparse_horizon) + [self.DT_MAX] * self.sparse_horizon)
            print(f"DT_array: {self.DT_array}")

        self.x = None
        self.u = None
        self.prev_u = np.zeros((2, self.N - 1))
        self.yaw = None
        self.sp = None

        self.ref_x = None
        self.ref_y = None
        self.ref_yaw = None
        self.cur_path_curve = None
        # Added for weight adaptation
        self.last_horizon_curvature = 0.0

        self.goal = None
        self.goal_reached_from_motion_planner = False
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
        self.prev_heading_error = 0.0
        self.count_wait = 0
        self.stop = False
        self.STOP_TIME = 20
        self.stop_count = 0
        self.send_first_control = False

        self.objective = 0
        self.motion_planner = None
        self.use_motion_planner = True
        self.use_steering_smoother = False
        
        self.log_costs_components = True
        self.lateral_error = 0.0
        self.prev_ema_lateral_error = None
        self.recent_errors = []
        self.prev_lateral_error = 0.0
        self.state_x_cost = 0.0
        self.state_y_cost = 0.0
        self.state_v_cost = 0.0
        self.state_yaw_cost = 0.0
        self.control_accel_cost = 0.0
        self.control_steer_cost = 0.0
        self.control_accel_diff_cost = 0.0
        self.control_steer_diff_cost = 0.0
        
        # Decaying factor for the cost function
        self.decaying_weight = False
        if self.decaying_weight:
            self.decay_constant = 0.25
        else:
            self.decay_constant = 0.0
            
        # Curvature Threshold for Weight Change
        self.max_curvature = 0.2
        self.curvature_threshold = self.max_curvature / 2
        self.activate_weight_change = False

    def setup_mpc(self):
        # Setup MPC
        self.x = cp.Variable((self.NX, self.N))
        self.u = cp.Variable((self.NU, self.N - 1))

    def setup_parking_mpc(self):
        #################################### PARKING STEP PARAMETERS SETTING ################################################
        self.EARLY_STOPPING_DIST = 0
        self.TARGET_SPEED = 0.4 # [m/s]
        self.MIN_SPEED_TO_MOVE = 0.35 # [m/s]

    def get_linear_model(self, v, psi, delta, cur_DT=None):
        if cur_DT is None:
            cur_DT = self.DT
        A = np.array(
            [
                [1, 0, cur_DT * np.cos(psi), - cur_DT * v * np.sin(psi)],
                [0, 1, cur_DT * np.sin(psi), cur_DT * v * np.cos(psi)],
                [0, 0, 1, 0],
                [0, 0, cur_DT * np.tan(delta) / self.WB, 1],
            ]
        )
        B = np.array(
            [
                [0, 0],
                [0, 0],
                [cur_DT, 0],
                [0, cur_DT * v / (self.WB * np.cos(delta) ** 2)],
            ]
        )
        C = np.array(
            [
                cur_DT * v * np.sin(psi) * psi,
                - cur_DT * v * np.cos(psi) * psi,
                0,
                - cur_DT * v * delta / (self.WB * np.cos(delta) ** 2),
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

    def calc_linear_mpc_control(self, xref, x0, oa, od):
        dref = np.zeros((1, self.N - 1))
        if oa is None or od is None:
            oa = [0.0] * (self.N - 1)
            od = [0.0] * (self.N - 1)

        oa, od, ox, oy, oyaw, ov = self.linear_mpc(xref, x0, od, dref)

        return oa, od, ox, oy, oyaw, ov

    def determine_initial_direction(self, cyaw, vehicle_state):
        heading_diff = np.abs(cyaw[0] - vehicle_state[3])
        if (heading_diff > np.pi / 4) and (heading_diff < 3 * np.pi / 2):
            self.direction = -1
        else:
            self.direction = 1
        pass

    # Main MPC function
    def linear_mpc(self, xref, x0, prev_od, dref):
        # print(f"Previous Delta: {prev_od}")
        cost = 0.0
        constraints = []

        # Set the weight based on the path curvature
        if not self.activate_weight_change:
            Q = self.Q_curve
            R = self.R_curve
            Rd = self.Rd_curve
            Qf = Q
        else:
            print(f"Current Path Curvature: {self.last_horizon_curvature}")
            # if abs(self.last_horizon_curvature) < self.curvature_threshold:
            if abs(self.cur_path_curve) < self.curvature_threshold:
                Q = self.Q_straight
                R = self.R_straight
                Rd = self.Rd_straight
                Qf = Q
            else:
                Q = self.Q_curve
                R = self.R_curve
                Rd = self.Rd_curve
                Qf = Q
        
        # Update lateral error (for stopping condition)
        self.lat_error = np.sqrt((x0[0] - xref[0, 0]) ** 2 + (x0[1] - xref[1, 0]) ** 2)

        # Time delay compensation
        self.ALPHA = int(math.ceil(self.TIME_DELAY_THROTTLE / self.DT_MIN))
        self.BETA = int(math.ceil(self.TIME_DELAY_STEER / self.DT_MIN))

        for k in range(self.N - 1):
            # Decaying the weight for the future states (for the cost function)
            if self.decaying_weight:
                Q = self.Q_curve * np.exp(-self.decay_constant * k)
                R = self.R_curve * np.exp(-self.decay_constant * k)
                Rd = self.Rd_curve * np.exp(-self.decay_constant * k)
            
            if self.VAR_DT:
                cur_DT = self.DT_array[k]
            else:
                cur_DT = self.DT
                
            self.A, self.B, self.C = self.get_linear_model(
                xref[2, k], xref[3, k], dref[0, k], cur_DT=cur_DT)
            constraints += [self.x[:, k + 1] == self.A @ self.x[:, k] + self.B @ self.u[:, k] + self.C]
            
            if k < self.ALPHA:
                constraints += [self.u[0, k] == self.prev_u[0, k + 1]]
                logging.info(f"ALPHA IS LESS THAN K: {k}")
            
            if k < self.BETA:
                constraints += [self.u[1, k] == self.prev_u[1, k + 1]]
                logging.info(f"BETA IS LESS THAN K: {k}")
            elif k >= self.BETA and k < (self.N - 2):
                constraints += [cp.abs(self.u[1, k + 1] - self.u[1, k]) <=
            self.MAX_DSTEER * cur_DT]
                
            # k=0 error cost (If there is no delay)
            if k == 0 and (self.ALPHA == 0 and self.BETA == 0):
                cost += cp.quad_form(xref[:, k] - self.x[:, k], Q)
                cost += cp.quad_form(self.u[:, k], R)
                # logging.info(f"ADDING COST FOR K = 0")
                if k < self.N-2:
                    cost += cp.quad_form(self.u[:, k + 1] - self.u[:, k], Rd)

            if k != 0 and k >= min(self.ALPHA, self.BETA):
                cost += cp.quad_form(xref[:, k] - self.x[:, k], Q)
                cost += cp.quad_form(self.u[:, k], R)
                # logging.info(f"ADDING COST FOR K = {k}")
                if k < self.N-2:
                    cost += cp.quad_form(self.u[:, k + 1] - self.u[:, k], Rd)
        # Decaying the weight for the final state (for the cost function)
        if self.decaying_weight:
            Qf = Q * np.exp(-self.decay_constant * (self.N - 1))
            
        # Final horizon state constraints
        cost += cp.quad_form(xref[:, self.N - 1] - self.x[:, self.N - 1], Qf)

        # logging.info(f"Current Speed: {x0[2]}")
        # logging.info(f"Current Reference Speed: {xref[2, :]}")
        # If the speed below the golf cart threshold
        if np.abs(x0[2]) < self.MIN_SPEED_TO_MOVE:
            self.MAX_ACCEL = 1.5
            self.MAX_DECEL = -1.5
        elif np.abs(x0[2]) >= self.MIN_SPEED_TO_MOVE:
            self.MAX_ACCEL = 1.5
            self.MAX_DECEL = -1.5

        # print(f"MAXIMUM ACCELERATION {self.MAX_ACCEL}")
        # print(f"MAXIMUM DECELERATION {self.MAX_DECEL}")

        constraints += [self.x[:, 0] == x0]
        constraints += [self.x[2, self.ALPHA:] <= self.MAX_SPEED]
        constraints += [self.x[2, self.ALPHA:] >= self.MIN_SPEED]
        constraints += [self.u[0, self.ALPHA:] <= self.MAX_ACCEL]
        constraints += [self.u[0, self.ALPHA:] >= self.MAX_DECEL]
        constraints += [cp.abs(self.u[1, self.BETA:]) <= self.MAX_STEER]

        # Previous control inputs flag
        if self.stop:
            prev_od = 0.0
        else:
            # print(f"Previous Steering {prev_od}")
            if not (self.PARKING_MODE and self.PATH_CHANGE):
                prev_od = prev_od[self.BETA]
            else:
                prev_od = prev_od[0]
        
        # Activate this constraint during parking
        constraints += [cp.abs(self.u[1, self.BETA] - prev_od) <= self.MAX_DSTEER * self.DT_MIN]

        prob = cp.Problem(cp.Minimize(cost), constraints)

        # Preventing crash during training or evaluating
        try:
            prob.solve(solver=cp.OSQP, verbose=False, warm_start=True)

            if prob.status == cp.OPTIMAL or prob.status == cp.OPTIMAL_INACCURATE:
                ox, oy, ov, oyaw = self.x.value[0, :].flatten(), self.x.value[1, :].flatten(), \
                                self.x.value[2, :].flatten(), self.x.value[3, :].flatten()
                # logging.info(f"Acceleration: {self.u.value[0, :].flatten()}")
                # Make sure the control input is not out of the limit
                oa, odelta = np.clip(self.u.value[0, :], self.MAX_DECEL, self.MAX_ACCEL), np.clip(
                    self.u.value[1, :], -self.MAX_STEER, self.MAX_STEER)
                # Update optimization cost
                self.prev_optim_cost = self.optim_cost
                self.optim_cost = prob.value
                self.objective_cost = self.optim_cost
                self.control_cost = np.sum(self.u.value[:, :].T ** 2 @ R)
                # self.control_cost_different_way = np.sum(self.u.value[:, :].T @ R * self.u.value[:, :].T)
                self.control_diff_cost = np.sum((self.u.value[:, 1:] - self.u.value[:, :-1]).T ** 2 @ Rd)
                # self.control_diff_cost_different_way = np.sum((self.u.value[:, 1:] - self.u.value[:, :-1]).T @ Rd * (self.u.value[:, 1:] - self.u.value[:, :-1]).T)
                self.state_cost_calc = np.sum((xref - self.x.value[:, :]).T ** 2 @ Q)
                self.objective_cost_calc = self.state_cost_calc + self.control_cost + self.control_diff_cost
                self.state_cost = self.objective_cost - self.control_cost - self.control_diff_cost
                # If Log the cost components
                if self.log_costs_components:
                    self.state_x_cost = np.sum((xref[0, :] - self.x.value[0, :]) ** 2 * Q[0, 0])
                    self.state_y_cost = np.sum((xref[1, :] - self.x.value[1, :]) ** 2 * Q[1, 1])
                    self.state_v_cost = np.sum((xref[2, :] - self.x.value[2, :]) ** 2 * Q[2, 2])
                    self.state_yaw_cost = np.sum((xref[3, :] - self.x.value[3, :]) ** 2 * Q[3, 3])
                    logging.info(f"State Yaw first cost: {np.sum((xref[3, 0] - self.x.value[3, 0]) ** 2 * Q[3, 3])}")
                    self.control_accel_cost = np.sum(self.u.value[0, :] ** 2 * R[0, 0])
                    self.control_steer_cost = np.sum(self.u.value[1, :] ** 2 * R[1, 1])
                    self.control_accel_diff_cost = np.sum((self.u.value[0, 1:] - self.u.value[0, :-1]) ** 2 * Rd[0, 0])
                    self.control_steer_diff_cost = np.sum((self.u.value[1, 1:] - self.u.value[1, :-1]) ** 2 * Rd[1, 1])
                # Steering smoother
                # print(f"Acceleration {oa}")
                # print(f"Steering before smoothing {odelta}")
                if self.use_steering_smoother:
                    odelta = self.smooth_steering_shift(prev_od, odelta, self.N - 1, self.BETA)
                # print(f"Steering after smoothing {odelta}")
                self.prev_u[0,:] = oa
                self.prev_u[1,:] = odelta           
            else:
                print("MPC Solution is not Feasible")
                oa, odelta, ox, oy, oyaw, ov = None, None, None, None, None, None
        except Exception as e:
            print("Exception Occurred ", e)
            print("MPC Solving Failed!")
            oa, odelta, ox, oy, oyaw, ov = None, None, None, None, None, None
        return oa, odelta, ox, oy, oyaw, ov

    def solve_mpc(self, vehicle_state, path_data, oa, od, dt_array=None, goal_reached_from_motion_planner=False, receive_curvature=False):
        # Obtain prev control input
        if oa is None or od is None:
            a = np.zeros((1, self.N - 1)).flatten()
            delta = np.zeros((1, self.N - 1)).flatten()
        else:
            a = oa
            delta = np.array(od, dtype=np.float32)

        current_state = [vehicle_state.x, vehicle_state.y, vehicle_state.v, vehicle_state.yaw]

        same_path = np.array_equal(path_data, self.prev_path)
        if not self.use_motion_planner:
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
                

                if any(self.motion_planner.cuspPointIdxList):
                    self.cusp_point = [path_init[i, :] for i in self.motion_planner.cuspPointIdxList]

                self.prev_path = path_data

                # Set the initial steering angle for MPC Constraints
                if self.PARKING_MODE:
                    delta = np.ones((1, self.N - 1)).flatten() * self.INIT_STEER_ANGLE
                    self.PARKING_MODE = False
                    self.PATH_CHANGE = False
                    
            # Obtain the path from the motion planner
            if self.prev_oa is not None:
                xref = self.motion_planner.step(current_state[0:2], current_state[2], self.prev_oa)
            else:
                xref = self.motion_planner.step(current_state[0:2], current_state[2], 0)
            
            # xref = self.motion_planner.step(current_state[0:2], current_state[2])
            traj_order = [0, 1, 3, 2]
            idx = np.empty_like(traj_order)
            idx[traj_order] = np.arange(len(traj_order))
            xref[:] = xref[:, idx]
            xref = xref[0:self.N, :].T

            if xref is not None:
                self.xref_tmp = xref

            print("Stopping count ", self.motion_planner.stopStep)
            
        else:
            if receive_curvature:
                path_curve = path_data[4]
            # Get Sample Time Array
            if self.use_rl_to_determine_dt:
                self.DT_array = dt_array
                self.last_idx = 0
                logging.info("DT Array from RL %s", dt_array)
            else:
                # If the motion planner path is updated reset the index
                print("Same path", same_path)
                if not same_path:
                    self.last_idx = 0
                    self.prev_path = path_data
                elif (not goal_reached_from_motion_planner or not self.halt_mpc) and same_path:
                    self.last_idx += 1
                    # self.last_idx = self.find_closest_point(current_state, path_data)
                print("Last index", self.last_idx)
                if receive_curvature:
                    self.DT_array = self.scale_dt(path_curve=path_curve[self.last_idx:], type=self.type)
                    self.cur_path_curve = path_curve[self.last_idx]
                else:
                    self.DT_array = self.scale_dt(type=self.type)
            # Obtain the reference path according to the sample time
            ## Test consistency of sampling time from DRL (K-Uniform MPC)
            # self.DT_array = np.array([self.DT_MIN * self.K_SCALE] * (self.N - 1))
            # use_sample_time = False if self.type == "uniform" else True
            # Apply FSM-MPC
            # self.DT_array = np.array([self.DT_MIN] * (self.total_horizon - self.sparse_horizon) + [self.DT_MAX] * self.sparse_horizon)
            use_sample_time = True
            if use_sample_time:
                DT_idx_sum = np.cumsum(self.DT_array)
                # print("DT Index sum", DT_idx_sum)
                # Divide the path into N segments based on the minimum sample time
                DT_idx = np.divide(DT_idx_sum, self.DT)
                # print("DT Index before", DT_idx)
                DT_idx = np.round(DT_idx).astype(int) + self.last_idx
                # Insert the last index to the first index
                DT_idx = np.insert(DT_idx, 0, self.last_idx)
                # print("DT Index", DT_idx)
                path_idx_range = DT_idx
            else:
                path_idx_range = range(self.last_idx, self.last_idx + self.N - 1)
            #print("reference path", path_data)
            #print("Path index range", path_idx_range)
            xref = np.array([path_data[0][path_idx_range], path_data[1][path_idx_range], path_data[2][path_idx_range], path_data[3][path_idx_range]])            
            #print("reference path", xref)
            # Match curvature data to the last horizon reference path
            if receive_curvature:
                self.last_horizon_curvature = path_curve[path_idx_range[-1]]
        
        # Calculate curvature for the node
        # ck = self.calculate_curvature(xref[0, :], xref[1, :])

        # Calculate the goal and stop flag distance
        isstop = (abs(vehicle_state.v) <= self.STOP_SPEED)
        isgoal = self.calc_goal_distance(current_state=current_state)
        
        # Constraints imposed to MPC
        if isstop and isgoal:
            self.stop = True
        else:
            self.stop = False

        # Deciding whether MPC needs to be stopped or not
        mpc_stop = self.stop_mpc_flag(current_state=current_state, isgoal=isgoal)

        self.halt_mpc = False
        if mpc_stop:
            if np.abs(current_state[2]) < self.MPC_THRESHOLD_TO_STOP:
                self.halt_mpc = True
            else:
                self.halt_mpc = False

        # check if the vehicle is reached the goal
        if ((isstop and isgoal) or xref is None and not self.use_motion_planner) or ((isstop and isgoal) or goal_reached_from_motion_planner):
            print("############################### GOAL REACHED #####################################")
            isstop, isgoal = True, True
            self.PARKING_MODE = True
            if self.PARKING_MODE and self.PATH_CHANGE:
                oa, odelta, ov = [0.0], [self.INIT_STEER_ANGLE], [0.0]
            else:
                oa, odelta, ov = [0.0], [0.0], [0.0]

            ox = np.ones((1, self.N)).flatten() * current_state[0]
            oy = np.ones((1, self.N)).flatten() * current_state[1]
            oyaw = np.ones((1, self.N)).flatten() * current_state[3]
            ov = float(ov[0])

            # ---------------Sungil update---------------230117
            self.prev_oa = oa[0]
            # self.setup_parking_mpc()
            # ---------------End update---------------230117

            return ox, oy, oyaw, oa, odelta, ov, isstop, isgoal, xref
            
        elif self.cusp_point is not None and self.motion_planner.stopStep != 0 and not self.use_motion_planner:
            if (self.cusp_point[self.cusp_idx][0] != self.goal[0]) and (self.cusp_point[self.cusp_idx][1] != self.goal[1]):
                stop_at_cusp_point = self.calc_stopping_distance(current_state)
                if stop_at_cusp_point and self.motion_planner.pathIdx + 1 < len(self.motion_planner.pathList):
                    print("############################# STOPPING AT CUSP POINT ################################")
                    # Initialize the steering angle to compensate the change at the segment change
                    print("Steering Init!!!")
                    pathIdx = self.motion_planner.pathIdx
                    post_path = self.motion_planner.pathList[pathIdx]
                  
                    oa, ov = [0.0], [0.0]
                    ov = float(ov[0])

                    self.prev_oa = oa[self.ALPHA]

                    # # Stanley
                    # odelta = self.init_steer_stanley(current_state, post_path, self.DT, self.N - 1)
                    # Curve init
                    ox = np.ones((1, self.N)).flatten() * current_state[0]
                    oy = np.ones((1, self.N)).flatten() * current_state[1]
                    oyaw = np.ones((1, self.N)).flatten() * current_state[3]
                    odelta = np.ones((1, self.N - 1)).flatten() * self.INIT_STEER_CUSP
                    if self.cusp_point and self.motion_planner.stopStep == 0:
                        if self.cusp_idx + 1 < len(self.motion_planner.cuspPointIdxList):
                            self.cusp_idx += 1
                            
                    return ox, oy, oyaw, oa, odelta, ov, isstop, isgoal, xref

        # Calculate the MPC (During normal operation)
        if not self.halt_mpc:
            start = time.time()
            # logging.warn("First 8 Reference Yaw inside of MPC before transformation %s", xref[3, :8])
            # Copy reference yaw for transformation
            yaw_ref = xref[3, :].copy()
            heading_error, transformed_heading, transformed_heading_ref = self.adjust_heading_error(current_state[3], yaw_ref)
            logging.warn("Heading Error %s", heading_error)
            transformed_xref = np.array([xref[0, :], xref[1, :], xref[2, :], transformed_heading_ref])
            transformed_current_state = [current_state[0], current_state[1], current_state[2], transformed_heading]
            # logging.warn("First 8 Reference Yaw inside of MPC after transformation %s", transformed_xref[3, :8])
            oa, odelta, ox, oy, oyaw, ov = self.calc_linear_mpc_control(transformed_xref, transformed_current_state, a, delta)
            print(f"Total time for MPC {time.time() - start}")
            if oa is None or odelta is None:
                oa = [0.0]
                oa_send = oa[0]
                odelta = [0.0]
                self.prev_oa = oa_send
            else:
                self.prev_oa = oa[self.ALPHA]
                oa_send = oa[self.ALPHA]
        # Halt the MPC (if the speed is too low and already inside the mpc stopping area)
        else:
            print("############################# MPC HALTED ################################")
            oa, odelta, ov = [0.0], [0.0], [0.0]
            ox = np.ones((1, self.N)).flatten() * current_state[0]
            oy = np.ones((1, self.N)).flatten() * current_state[1]
            oyaw = np.ones((1, self.N)).flatten() * current_state[3]
            ov = float(ov[0])
            isstop = True
            isgoal = True
            self.prev_oa = oa[0]
            # self.PARKING_MODE = True
            oa_send = oa[0]
            
        ov = current_state[2] + self.DT * oa_send

        return ox, oy, oyaw, oa, odelta, ov, isstop, isgoal, xref

    def calc_goal_distance(self, current_state):
        dx = np.abs(current_state[0] - self.goal[0])
        dy = np.abs(current_state[1] - self.goal[1]) 
        # print(dx, dy)
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

    def stop_mpc_flag(self, current_state, isgoal):
        dx = np.abs(current_state[0] - self.goal[0])
        dy = np.abs(current_state[1] - self.goal[1]) 
        isgoal_x = (dx <= self.STOP_MPC_X)
        isgoal_y = (dy <= self.STOP_MPC_Y)
        stop_mpc = isgoal_x and isgoal_y and not isgoal
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

    def scale_dt(self, type: str ="exp", path_curve: list=None, path_speed: list=None, cur_speed: list=None, cost: list=None) -> np.array:
        if type == "sigmoid":
            hor_arr = np.arange(self.N)
            scale = 21
            exp_input_scale = 0.2 
            shifting = self.DT_MIN * scale / 2
            return np.clip(shifting / (1 + np.exp(-exp_input_scale * hor_arr)) - (shifting/2 - self.DT_MIN), self.DT_MIN, self.DT_MAX)
        elif type == "uniform":
            return np.ones(self.N-1) * self.DT_MIN
        elif type == "exp":
            scale_exp = lambda la, tau, k: la * np.exp(tau * k)
            la = 1.0 #0.2 #1.0
            tau = 0.2 #0.5 #0.2
            scaled_dt = [scale_exp(la, tau, k) * self.DT_MIN for k in range(self.N)] 
            return np.clip(scaled_dt, self.DT_MIN, self.DT_MAX)
        elif type == "linear":
            scale_linear = lambda la, tau, k: la * k + tau
            la = 0.7 # self.DT_MAX - self.DT_MIN
            tau = self.DT_MIN * 10
            scaled_dt = [scale_linear(la, tau, k) * self.DT_MIN for k in range(self.N)]
            # print("Scaled DT", scaled_dt)
            return np.clip(scaled_dt, self.DT_MIN, self.DT_MAX)
        elif type == "scaled":
            scale_1 = self.DT_MIN * (self.N - 1)
            scale_2 = 3
            scale_3 = self.DT_MAX * (self.N - 1)
            if self.N < 4:
                if self.N == 2:
                    return np.array([self.DT * scale_1, self.DT * scale_2])
                elif self.N == 3:
                    return np.array([self.DT * scale_1, self.DT * scale_2, self.DT * scale_3])
                else:
                    raise ValueError("The number of horizon should be greater than 1")
            else:
                scaling_term = []
                # Dense Horizon
                prop_1 = int(0.3 * self.N) / self.N
                # Sparse Horizon
                prop_2 = int(0.4 * self.N) / self.N
                prop_3 = 1.0 - prop_1 - prop_2
                scaling_term.extend([scale_1] * int(prop_1 * self.N))
                scaling_term.extend([scale_2] * int(prop_2 * self.N))
                scaling_term.extend([scale_3] * int(prop_3 * self.N))
                scaling = np.multiply(scaling_term, self.DT_MIN)
                # Clip the scaling term
                scaled_dt = np.clip(scaling, self.DT_MIN, self.DT_MAX)
                # # Experimental
                # scaling = np.multiply(self.DT, np.ones(self.N))
            return scaled_dt
        elif type == "quadratic":
            fitting = False
            if fitting:
                x = np.arange(self.N)
                y = np.array([self.DT, self.DT + 0.05, self.DT + 0.1, self.DT + 0.15, 0.3, 0.35, 0.65, 0.8, 0.95, 1.1])
                z = np.polyfit(x[:-1], y, 2)
                p = np.poly1d(z)
            else:
                scale_quad = lambda la, tau, eps, k: la * k ** 2 + tau * k + eps
                la = self.DT_MIN#0.8 #self.DT_MIN
                tau = self.DT_MIN#-1.7 #self.DT_MIN
                eps = self.DT_MIN * 10#1.0 #self.DT_MIN * 10
                scaled_dt = [scale_quad(la, tau, eps, k) * self.DT_MIN for k in range(self.N)]
            return np.clip(scaled_dt, self.DT_MIN, self.DT_MAX)
        elif type == "classical":
            return np.ones(self.N) * self.DT_MIN
        # Dual MPC
        elif type == "dual":
            scale_1 = 1
            scale_2 = self.DT_MAX*(self.N - 1)
            if self.N < 2:
                raise ValueError("The number of horizon should be greater than 1")
            else:
                prop_1 = int(0.3 * self.N) / self.N
                prop_2 = 1.0 - prop_1
                scaling_term = []
                scaling_term.extend([scale_1] * int(prop_1 * self.N))
                scaling_term.extend([scale_2] * int(prop_2 * self.N))
                scaling = np.multiply(scaling_term, self.DT_MIN)
                scaling = np.clip(scaling, self.DT_MIN, self.DT_MAX)
            return scaling
        #TODO: Implement the Sensor Papers scaling method
        elif type == "fixed_adapt":
            pass
        #TODO: Implement the Variable Sampling Model Non-Uniform sample time in the Sparse Part
        elif type == "nonuni_sparse_var":
            return self.non_uniform_adapt(path_curve)
        elif type == "fsmpc":
            sparse_horizon = 4  
            return [self.DT_MIN] * (self.N - 1 - sparse_horizon) + [self.DT_MAX] * sparse_horizon
        #TODO: Proposed approach
        elif type == "proposed":
            pass
        else:
            return self.DT * np.ones(self.N)
        
    def non_uniform_adapt(self, path_curve: list,
                          gamma_threshold: float=0.01,
                          curve_threshold: float=0.01, N_sparse: int=4,
                          Nn_min: int=1, Nn_max: int=8, 
                          use_dt_from_class: bool=True) -> np.array:
        
        ''' Implementation of Variable Sampling Model for the Sparse Part from
        "Model Predictive Control Method for Autonomous Vehicles Using Time-Varying and
        Non-Uniformly Spaced Horizon" M. Kim et al. 2021. Total prediction horizon is N = (Nd + NsNn)dt_min, 
        where Nd is the dense part and Ns is the sparse part (Algorithm 1).
        - For instance: 
        In our method (with Np=8) the DRL max sampling time for the sparse is 0.8sx4 = 3.2s, min is 0.1sx4 = 0.4s 
        in total max is 3.6s (3.2s from Dense and 0.4s from Sparse), min is 0.8s (0.4s from Dense and 0.4s from Sparse). 
        To have the same number of horizon in this method, we need to have 4 horizon in the sparse part
        with Nn_min = 1 and Nn_max = 8, so that the Max prediction interval will be (4(dense)+ 4*8(sparse))*0.1(dt_min) = 3.6s.
        Min prediction interval will be (4(dense)+ 4*1(sparse))*0.1(dt_min) = 0.8s'''
        
        if use_dt_from_class:
            DT_min = self.DT_MIN
            DT_max = self.DT_MAX
            Nn_max = self.Nn_max
            Nn_min = self.Nn_min
        
        Nn = self.Nn
        
        # Current 
        prev_cost = self.prev_optim_cost
        current_cost = self.optim_cost
        # Avoid division by zero
        if prev_cost == 0:
            cost_rate = 0
        else:
            cost_rate = (prev_cost - current_cost) / prev_cost
        gamma_threshold = 0.01 #0.1
        curve_threshold = 0.01 #0.1
        print(f"Cost Rate {cost_rate}")
        if self.Nn_current == self.Nn_prev:
            if cost_rate >= gamma_threshold:
                if np.abs(path_curve[0]) <= curve_threshold and Nn < Nn_max:
                    Nn += 1
            elif cost_rate <= -gamma_threshold:
                if np.abs(path_curve[0]) >= curve_threshold and Nn > Nn_min:
                    Nn -= 1
        print(f"Path Curve {path_curve[0]}")
        # Update the horizon
        self.Nn = Nn
        print(f"Current Nn {Nn}")
        self.Nn_prev = self.Nn_current
        self.Nn_current = Nn
        # DT array
        DT_array = np.ones(self.N-1) * DT_min
        DT_array[-N_sparse:] = Nn * DT_min
        return DT_array
    
    def fixed_adapt(self, DT: list, steering_angle: list, lateral_speed: list) -> np.array:
        prev_DT = DT[0]
        cur_DT = DT[1]
        prev_lat_speed = lateral_speed[0]
        cur_lat_speed = lateral_speed[1]
        prev_steer = steering_angle[0]
        cur_steer = steering_angle[1]
        la = 0.2
        C = -0.01
        Z = -la * ((cur_steer * cur_lat_speed/cur_DT) - (prev_steer * prev_lat_speed/prev_DT))
        next_DT = cur_DT + np.sign(Z-C)*np.min([Z, C])
        return self.bounding_min_max(next_DT, self.DT_MIN, self.DT_MAX)
        
    # Weight scaling    
    def scale_weight(self, Q: np.array, R: np.array, k: int, decay: str ='exp', path_curve: list =None, path_speed: list =None):
        if decay == 'exp':
            a = 0.2
            for i in range(self.NX):
                Q[i, i] = Q[i, i] * math.exp(-a * k)
                if i < self.NU:
                    R[i, i] = R[i, i] * math.exp(-a * k)
        else:
            return Q, R
        
    def find_closest_point(self, state, path):
        ref_x, ref_y = path[0].tolist(), path[1].tolist()
        dx = [state[0] - icx for icx in ref_x]
        dy = [state[1] - icy for icy in ref_y]
        d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]
        min_dist = min(d)
        ind = d.index(min_dist)
        if self.last_idx >= ind:
            ind = self.last_idx
        else:
            self.last_idx = ind
        return ind
    
    def adjust_heading_error(self, heading: float, heading_ref: list) -> float:
        # Adjust the heading error to be within the range [-pi, pi]
        # Given current heading and reference heading transform the heading to avoid the discontinuity
        heading_error = heading_ref[0] - heading
        logging.info(f"Heading Errror Before {heading_error}")
        if heading_error > np.pi or heading_error < -np.pi:
            # Convert heading and heading ref to the range to avoid confusion in the controller
            if heading > np.pi/2 and (heading_ref < -np.pi/2).any():
                heading = heading 
                for i in range(len(heading_ref)):
                    heading_ref[i] = heading_ref[i] + 2*np.pi
            elif heading < -np.pi/2 and (heading_ref > np.pi/2).any():
                heading = heading
                for i in range(len(heading_ref)):
                    heading_ref[i] = heading_ref[i] - 2*np.pi
            # Calculate the heading error and adjust it to be within the range [-pi, pi]
            heading_error = heading_ref[0] - heading
            # heading_error = np.mod(heading_error + np.pi, 2 * np.pi) - np.pi
        else:
            heading_error = heading_error
            
        delta_error = heading_error - self.prev_heading_error
        # Check if delta error (current error - previous error) is within the range
        if delta_error > np.pi/2:
            heading_error = heading_error - 2*np.pi
        elif delta_error < -np.pi/2:
            heading_error = heading_error + 2*np.pi
            
        self.prev_heading_error = heading_error
            
        return heading_error, heading, heading_ref
    
    @staticmethod
    def heading_error_bound(heading_error: float, previous_error: float = 0.0) -> float:
        if heading_error > math.pi:
            heading_error = heading_error - 2 * math.pi
        elif heading_error < -math.pi:
            heading_error = heading_error + 2 * math.pi
        else:
            heading_error = heading_error
            
        delta_error = heading_error - previous_error
        # Check if delta error (current error - previous error) is within the range
        if delta_error > math.pi / 2:
            heading_error = heading_error - 2 * math.pi
        elif delta_error < -math.pi / 2:
            heading_error = heading_error + 2 * math.pi
        return heading_error
    
    @staticmethod
    def bounding_min_max(value, min_val, max_val):
        return max(min(value, max_val), min_val)
    
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
    def smooth_steering(prev_od, od, pred_hor, beta):
        del_sum = np.sum(od)
        del_avg = (prev_od + del_sum) / (pred_hor + 1)

        # Steering speed
        del_steer = 2 * (del_avg - prev_od) / (pred_hor)
        new_od = [prev_od + (i + 1) * del_steer for i in range(pred_hor)]
        return new_od

    @staticmethod
    def smooth_steering_shift(prev_od, od, pred_hor, beta):
        del_sum = np.sum(od[beta:])
        del_avg = (prev_od + del_sum) / (pred_hor - beta + 1)

        # Steering speed
        del_steer = 2 * (del_avg - prev_od) / (pred_hor - beta)
        new_od = [prev_od + (i + 1) * del_steer for i in range(pred_hor)]
        new_od[:beta] = od[:beta]
        new_od = [arr.item() for arr in new_od]
        return new_od
    
