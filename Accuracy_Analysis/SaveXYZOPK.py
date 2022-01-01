import csv
import os
import math
import numpy as np

RootFolder = "E:\\Map\\Guiren_0907\\Analysis\\Guiren_Noon\\"

SLAMFile = RootFolder + "KFTraj_TUM.txt"
SfMFile = RootFolder + "SfM_Traj.txt"
RotFile = RootFolder + "Traj_MTA\\Precision_Report_Traj_0.txt"

Output = RootFolder + "SfM_XYZOPK.csv"

SLAMRotList = []
SLAMTimeList = []
SfMRotList = []
KFList = []
OmegaDiffList = []
PhiDiffList = []
KappaDiffList = []

Rot7Para = np.zeros(2) #Just for Initialize

def cos(a):
	return math.cos(a)
	
def sin(a):
	return math.sin(a)

def RotationMatrix(omega, phi, kappa):
	OmegaMatrix = np.array([[1, 0, 0], 
	[0, cos(omega), sin(omega)], 
	[0, -sin(omega), cos(omega)]])
	PhiMatrix = np.array([[cos(phi), 0, -sin(phi)], [ 0, 1, 0], [sin(phi), 0, cos(phi)]])
	KappaMatrix = np.array([[cos(kappa), sin(kappa), 0], [-sin(kappa), cos(kappa), 0 ], [0, 0, 1]])

	return KappaMatrix.dot(PhiMatrix.dot(OmegaMatrix))

def RotationMatrixCV(omega, phi, kappa):
	OmegaMatrix = np.array([[1, 0, 0],[0, cos(omega), -sin(omega)],[0, sin(omega), cos(omega)]])
	PhiMatrix = np.array([[cos(phi), 0, sin(phi)],[0, 1, 0],[-sin(phi), 0, cos(phi)]])
	KappaMatrix = np.array([[cos(kappa), -sin(kappa), 0],[sin(kappa), cos(kappa), 0],[0, 0, 1]])
	
	return KappaMatrix.dot(PhiMatrix.dot(OmegaMatrix))

with open(RotFile,encoding="utf-8") as csvfile:	

	count = -5

	for row in csv.reader(csvfile, delimiter=' '):
		if count < 0:
			count += 1
			pass
		else:
			Rot7Para = RotationMatrix(float(row[2]), float(row[5]), float(row[8]))
			break

SLAMoutX = []
SLAMoutY = []
SLAMoutZ = []
SLAMoutO = []
SLAMoutP = []
SLAMoutK = []

with open(SLAMFile, newline='') as csvfile:  
	
	count = 0

	for row in csv.reader(csvfile, delimiter=' '):
		if count == 0:
			count += 1
			pass
		else:
			KF = int(row[-1])
			time = float(row[1])
			
			R = np.array([
			[float(row[5]),float(row[6]),float(row[7])],
			[float(row[8]),float(row[9]),float(row[10])],
			[float(row[11]),float(row[12]),float(row[13])]])
			
			KFList.append(KF)
			SLAMRotList.append(Rot7Para.dot(R))
			SLAMTimeList.append(time)
			
			phi = math.asin(Rot7Para.dot(R)[0, 2])
			kappa = math.asin(-Rot7Para.dot(R)[0, 1]/cos(phi))
			omega = math.asin(-Rot7Para.dot(R)[1, 2]/cos(phi))
			
			SLAMoutX.append(float(row[2]))
			SLAMoutY.append(float(row[3]))
			SLAMoutZ.append(float(row[4]))
			SLAMoutO.append(omega)
			SLAMoutP.append(phi)
			SLAMoutK.append(kappa)
			
SfMoutX = []
SfMoutY = []
SfMoutZ = []
SfMoutO = []
SfMoutP = []
SfMoutK = []

with open(SfMFile, newline='') as csvfile:  
	
	count = -2

	for row in csv.reader(csvfile, delimiter='\t'):
		if count < 0:
			count += 1
			pass
		else:
			for i, element in enumerate(KFList):
				if element == int(row[0][0:-4]):
					R = np.array([
					[float(row[7]),float(row[8]),float(row[9])],
					[float(row[10]),float(row[11]),float(row[12])],
					[float(row[13]),float(row[14]),float(row[15])]])
			
					SfMRotList.append(R)
		
					SfMoutX.append(float(row[1]))
					SfMoutY.append(float(row[2]))
					SfMoutZ.append(float(row[3]))
					SfMoutO.append(float(row[4]))
					SfMoutP.append(float(row[5]))
					SfMoutK.append(float(row[6]))

with open(Output, 'w', newline='') as csvfile3:
		writer = csv.writer(csvfile3)
		writer.writerow(['X Y Z Omega Phi Kappa'])
    	
		#for i in range(len(SLAMoutX)):
		#	ToWrite = str('{:.6f}'.format(SLAMoutX[i])) + ' ' + str('{:.6f}'.format(SLAMoutY[i])) + ' ' + str('{:.6f}'.format(SLAMoutZ[i])) + ' ' 
		#	ToWrite = ToWrite + str('{:.6f}'.format(SLAMoutO[i]*180/math.pi)) + ' ' + str('{:.6f}'.format(SLAMoutP[i]*180/math.pi)) + ' ' + str('{:.6f}'.format(SLAMoutK[i]*180/math.pi))
			
		#	writer.writerow([ToWrite])
		for i in range(len(SfMoutX)):
			ToWrite = str('{:.6f}'.format(SfMoutX[i])) + ' ' + str('{:.6f}'.format(SfMoutY[i])) + ' ' + str('{:.6f}'.format(SfMoutZ[i])) + ' ' 
			ToWrite = ToWrite + str('{:.6f}'.format(SfMoutO[i])) + ' ' + str('{:.6f}'.format(SfMoutP[i])) + ' ' + str('{:.6f}'.format(SfMoutK[i]))
			
			writer.writerow([ToWrite])
	



	
	
	
	
	
	
	

