import csv
import os
import math
import numpy as np

#RootFolder = "E:\\Map\\Guiren_0907\\Analysis\\Guiren_Cloudy\\"
RootFolder = "E:\\Map\\Side\\Analysis\\Side_Cloudy\\"

SLAMFile = RootFolder + "KFTraj_TUM.txt"
SfMFile = RootFolder + "SfM_Traj.txt"
#RotFile = RootFolder + "Traj_MTA\\Precision_Report_Traj_0.txt"
RotFile = RootFolder + "CP_MTA\\Precision_Report_Traj_0.txt"

Outfile = RootFolder + "Rotation_Report_SLAM_" + RotFile[len(RootFolder):len(RootFolder) + 2] + "_" + RotFile[-5:-4]+"2_ABS.txt"

SLAMRotList = []
SLAMTimeList = []
SfMRotList = []
SfMRotListCV = []
KFList = []
OmegaDiffList = []
PhiDiffList = []
KappaDiffList = []

Rot7Para = np.zeros(2) #Just for Initialize

def cos(a):
	return math.cos(a)
	
def sin(a):
	return math.sin(a)

def SSE(List):
	
	sse = 0
	
	for item in List:
		sse += item*item
		
	return sse

def RotationMatrix(omega, phi, kappa):
	OmegaMatrix = np.array([[1, 0, 0], [0, cos(omega), sin(omega)], [0, -sin(omega), cos(omega)]])
	PhiMatrix = np.array([[cos(phi), 0, -sin(phi)], [ 0, 1, 0], [sin(phi), 0, cos(phi)]])
	KappaMatrix = np.array([[cos(kappa), sin(kappa), 0], [-sin(kappa), cos(kappa), 0 ], [0, 0, 1]])

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
			
			R = Rot7Para.dot(R)
			
			KFList.append(KF)
			SLAMRotList.append(R)
			SLAMTimeList.append(time)
			
			#ISPRS Transpose
			#phi = math.asin(R[0, 2])
			#kappa = math.asin(R[0, 1]/cos(phi))
			#omega = math.asin(R[1, 2]/cos(phi))
			
			#ISPRS
			#phi = math.asin(R[2, 0])
			#kappa = math.asin(-R[1, 0]/cos(phi))
			#omega = math.asin(-R[2, 1]/cos(phi))
			#print(omega * 180/math.pi, phi * 180/math.pi, kappa * 180/math.pi)
			

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
			
					#SfMRotList.append(R)
	
					#Transform to OpenCV Definition
					omega = float(row[4])-180
					if omega<-180:
						omega = omega + 360
					phi = -float(row[5])
					kappa = -float(row[6])
					
					SfMRotList.append(RotationMatrix(omega*math.pi/180, phi*math.pi/180, kappa*math.pi/180))

OmegaDiffListAbs = []
PhiDiffListAbs = []
KappaDiffListAbs = []
					
count = -1
for RotM in SfMRotList:
	count += 1	
	Rdiff = SLAMRotList[count].dot(RotM)
	
	phi = math.asin(Rdiff[0, 2])
	
	try:
		omega = math.asin( -Rdiff[1, 2] / cos(phi))		
	except ValueError:
		if -Rdiff[1, 2] / cos(phi) > 1:
			omega = math.asin(1)
		else:
			omega = math.asin(-1)
			
	try:
		kappa = math.asin( -Rdiff[0, 1] / cos(phi))		
	except ValueError:
		if -Rdiff[0, 1] / cos(phi) > 1:
			kappa = math.asin(1)
		else:
			kappa = math.asin(-1)		
	
	#print(omega * 180/math.pi, phi * 180/math.pi, kappa * 180/math.pi)
	
	OmegaDiffList.append(omega * 180/math.pi)
	PhiDiffList.append(phi * 180/math.pi)
	KappaDiffList.append(kappa * 180/math.pi)
	
	OmegaDiffListAbs.append(abs(omega * 180/math.pi))
	PhiDiffListAbs.append(abs(phi * 180/math.pi))
	KappaDiffListAbs.append(abs(kappa * 180/math.pi))
	
	
#Statistic
Mean_DiffOmega = '{:.3f}'.format(np.mean(OmegaDiffListAbs))
STD_DiffOmega = '{:.3f}'.format(np.std(OmegaDiffList, ddof = 1))
Median_DiffOmega = '{:.3f}'.format(np.median(OmegaDiffListAbs))
Max_DiffOmega = '{:.3f}'.format(max(OmegaDiffListAbs))
Min_DiffOmega = '{:.3f}'.format(min(OmegaDiffListAbs))
SSE_DiffOmega = '{:.3f}'.format(SSE(OmegaDiffList))
RMSE_DiffOmega = '{:.3f}'.format(math.sqrt(SSE(OmegaDiffList)/len(OmegaDiffList)))

Statistic_Output1 = 'STD = ' + str(STD_DiffOmega) + ', RMSE = ' + str(RMSE_DiffOmega) +', Mean = ' + str(Mean_DiffOmega) + ', Med = ' + str(Median_DiffOmega)
Statistic_Output2 = ', Max = ' + str(Max_DiffOmega) + ', Min = ' + str(Min_DiffOmega) + ', SSE = ' + str(SSE_DiffOmega)

Mean_DiffPhi = '{:.3f}'.format(np.mean(PhiDiffListAbs))
STD_DiffPhi = '{:.3f}'.format(np.std(PhiDiffList, ddof = 1))
Median_DiffPhi = '{:.3f}'.format(np.median(PhiDiffListAbs))
Max_DiffPhi = '{:.3f}'.format(max(PhiDiffListAbs))
Min_DiffPhi = '{:.3f}'.format(min(PhiDiffListAbs))
SSE_DiffPhi = '{:.3f}'.format(SSE(PhiDiffList))
RMSE_DiffPhi = '{:.3f}'.format(math.sqrt(SSE(PhiDiffList)/len(PhiDiffList)))

Statistic_Output3 = 'STD = ' + str(STD_DiffPhi) + ', RMSE = ' + str(RMSE_DiffPhi) +', Mean = ' + str(Mean_DiffPhi) + ', Med = ' + str(Median_DiffPhi)
Statistic_Output4 = ', Max = ' + str(Max_DiffPhi) + ', Min = ' + str(Min_DiffPhi) + ', SSE = ' + str(SSE_DiffPhi)

Mean_DiffKappa = '{:.3f}'.format(np.mean(KappaDiffListAbs))
STD_DiffKappa = '{:.3f}'.format(np.std(KappaDiffList, ddof = 1))
Median_DiffKappa = '{:.3f}'.format(np.median(KappaDiffListAbs))
Max_DiffKappa = '{:.3f}'.format(max(KappaDiffListAbs))
Min_DiffKappa = '{:.3f}'.format(min(KappaDiffListAbs))
SSE_DiffKappa = '{:.3f}'.format(SSE(KappaDiffList))
RMSE_DiffKappa = '{:.3f}'.format(math.sqrt(SSE(KappaDiffList)/len(KappaDiffList)))

Statistic_Output5 = 'STD = ' + str(STD_DiffKappa) + ', RMSE = ' + str(RMSE_DiffKappa) +', Mean = ' + str(Mean_DiffKappa) + ', Med = ' + str(Median_DiffKappa)
Statistic_Output6 = ', Max = ' + str(Max_DiffKappa) + ', Min = ' + str(Min_DiffKappa) + ', SSE = ' + str(SSE_DiffKappa)	

with open(Outfile, 'w', newline='') as csvfile3:
		writer = csv.writer(csvfile3)
		writer.writerow(["Omega"])
		writer.writerow([Statistic_Output1 + Statistic_Output2])
		writer.writerow(' ')
		writer.writerow(["Phi"])
		writer.writerow([Statistic_Output3 + Statistic_Output4])
		writer.writerow(' ')
		writer.writerow(["Kappa"])
		writer.writerow([Statistic_Output5 + Statistic_Output6])
		writer.writerow(' ')
		writer.writerow(' ')
		writer.writerow(' ')
		writer.writerow(['Time dOmega dPhi dKappa'])
    	
		for i in range(len(SLAMTimeList)):
			ToWrite = [str(SLAMTimeList[i]) + ' ' + str('{:.6f}'.format(OmegaDiffList[i])) + ' ' + str('{:.6f}'.format(PhiDiffList[i])) + ' ' + str('{:.6f}'.format(KappaDiffList[i]))]
			
			writer.writerow(ToWrite)
			
print(' ')			
print("       " + Outfile + " Successfully Saved !!!")