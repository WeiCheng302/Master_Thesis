import csv
import os
import math
import numpy as np

#Where the following data belongs to
Root_Folder = 'E:\\Map\\Guiren_0907\\Analysis\\Guiren_Morning\\Localization_Mode\\Localization_Mode_VM_97\\'

#The Reference Data
Reference = Root_Folder + 'SfM_Traj_TUM.txt'

#The Data to be Analyzed
#FileToAnalyze = Root_Folder + 'Localization_Mode_CP97_VC_MA_Clip.csv'
FileToAnalyze = Root_Folder + 'Localization_Mode_CP97_2_VM_MC_Clip.csv'
#The Report of the Analysis
Output = FileToAnalyze[0:-4] + '_Report_OutlierRM.txt'

def SSE(List):
	
	sse = 0
	
	for item in List:
		sse += item*item
		
	return sse

skip = 0
FormerRowX = 0
Time = []
ReferenceCoordinate = []
LocalizationCoordinate = []
CoordinateDifference = []
PointDifference = []

with open(Reference, newline='') as csvfile:  
	for row in csv.reader(csvfile, delimiter=' '):
		Time.append(row[0])
		ReferenceCoordinate.extend([float(row[1]), float(row[2]), float(row[3])])

with open(FileToAnalyze, newline='') as csvfile:
	for row in csv.reader(csvfile, delimiter=' '):
			
		if float(row[1]) == FormerRowX:
			skip += 1

		LocalizationCoordinate.extend(['{:.6f}'.format(float(row[1])), '{:.6f}'.format(float(row[2])), '{:.6f}'.format(float(row[3]))])
		FormerRowX = float(row[1])

if skip != 1 :
	skip += 1

for i in range(3 * skip, len(LocalizationCoordinate)):
	#print(float(ReferenceCoordinate[i]))
	CoordinateDifference.append(float(ReferenceCoordinate[i]) - float(LocalizationCoordinate[i]))

for i in range(len(CoordinateDifference)):
	if i%3 == 0:
		#print(i)
		#print(len(LocalizationCoordinate))
		#print(len(CoordinateDifference))
		pointdiff= math.sqrt(CoordinateDifference[i]**2 + CoordinateDifference[i + 1]**2 + CoordinateDifference[i + 2]**2)
		PointDifference.append(pointdiff)
#print(PointDifference)
while max(PointDifference)>1:
	maxIndex = PointDifference.index(max(PointDifference))
	PointDifference.pop(maxIndex)
	Time.pop(maxIndex + 1)
	CoordinateDifference.pop(3 * maxIndex + 0)
	CoordinateDifference.pop(3 * maxIndex + 1)
	CoordinateDifference.pop(3 * maxIndex + 2)

	print(len(Time))
	print(len(PointDifference))
	print(len(CoordinateDifference))
	print(maxIndex)

Mean_PointDifference = '{:.3f}'.format(np.mean(PointDifference))
STD_PointDifference = '{:.3f}'.format(np.std(PointDifference, ddof = 1))
Median_PointDifference = '{:.3f}'.format(np.median(PointDifference))
Max_PointDifference = '{:.3f}'.format(max(PointDifference))
Min_PointDifference = '{:.3f}'.format(min(PointDifference))
SSE_PointDifference = '{:.3f}'.format(SSE(PointDifference))
RMSE_PointDifference = '{:.3f}'.format(math.sqrt(SSE(PointDifference)/len(PointDifference)))

Statistic_Output1 = 'STD = ' + str(STD_PointDifference) + ', RMSE = ' + str(RMSE_PointDifference) +', Mean = ' + str(Mean_PointDifference) + ', Med = ' + str(Median_PointDifference)
Statistic_Output2 = ', Max = ' + str(Max_PointDifference) + ', Min = ' + str(Min_PointDifference) + ', SSE = ' + str(SSE_PointDifference)

with open(Output, 'w', newline='') as csvfile3:
		writer = csv.writer(csvfile3)
		writer.writerow([Statistic_Output1 + Statistic_Output2])
		writer.writerow(' ')
		writer.writerow(' ')
		writer.writerow(' ')
		writer.writerow(['CPID Vx Vy Vz Vxyz V97x V97y V97z V97xyz E N H'])
    	
		for i in range(skip, len(Time)):
			#print('======')
			#print(i)
			#print(len(PointDifference))
			basic = str(Time[i]) + ' 0 0 0 0 '
			VxVyVz = str('{:.6f}'.format(CoordinateDifference[3 * (i - skip) + 0])) + ' ' + str('{:.6f}'.format(CoordinateDifference[3 * (i - skip)+ 1])) + ' ' + str('{:.6f}'.format(CoordinateDifference[3 * (i - skip) + 2]))
			ENH = str('{:.6f}'.format(ReferenceCoordinate[3 * (i - skip) + 0])) + ' ' + str('{:.6f}'.format(ReferenceCoordinate[3 * (i - skip) + 1])) + ' ' + str('{:.6f}'.format(ReferenceCoordinate[3 * (i - skip) + 2]))
			ToWrite = [basic + VxVyVz + ' ' + str('{:.6f}'.format(float(PointDifference[i - skip]))) + ' ' + ENH]
			
			writer.writerow(ToWrite)

    		









