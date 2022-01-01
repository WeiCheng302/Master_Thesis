import csv
import os
from scipy.spatial.transform import Rotation as R

#Where the Reference KFTrajTUM is
ReferenceFolder = 'E:\\Map\\Guiren_0907\\Analysis\\Guiren_Morning\\'

#Where Localization Mode Data belong
Root_Folder = 'E:\\Map\\Guiren_0907\\Analysis\\Guiren_Morning\\Localization_Mode\\Localization_Mode_VM_97\\'

#Reference TUM File
Reference = ReferenceFolder + 'KFTraj_TUM.txt'

#Localization Mode Trajectory
LocalizationModeFile = Root_Folder + 'Localization_Mode_Traj97_2_VM_MC.txt'
#LocalizationModeFile = Root_Folder + 'Localization_Mode_Traj97_VC_MN.txt'
LocalizationOutput = LocalizationModeFile[0:-4] + '_Clip.csv'
#FrameTrajectory = Root_Folder + 'Frame_Traj_TUM.txt'
#FrameTrajectoryOutput = Root_Folder + 'Frame_Traj_TUM_Clip.csv'

def IsKeyFrame(LocalizationModeTime, KFTime):
	check = False	
	for time in KFTime:
		if LocalizationModeTime == time:
			check = True
	return check

def TUM_Output(count, Output, Time, Translation, Quat):
    Output[count][0] = str(Time[count])
    Output[count][1] = float(Translation[count][0])
    Output[count][2] = float(Translation[count][1])
    Output[count][3] = float(Translation[count][2])
    # #Output[count][4] = float(Quat[count][0])
    # #Output[count][5] = float(Quat[count][1])
    # #Output[count][6] = float(Quat[count][2])
    # #Output[count][7] = float(Quat[count][3])

def LocalizationModeClip(reference, LoadFile, outfile):
	Translation = []
	Quat = []
	KFTime = []
    
	with open(reference, newline='') as csvfile:  
		KFTime = [row[1] for row in csv.reader(csvfile, delimiter=' ') if row[0][0]!='K']
    
	with open(LoadFile, newline='') as csvfile2:
    
		rows = csv.reader(csvfile2, delimiter=' ')
		
		for row in rows:
			if IsKeyFrame(row[0], KFTime):
				t = [float(row[1]),float(row[2]),float(row[3])]
				#r = R.from_matrix([[row[5], row[6], row[7]],[row[8], row[9],row[10]],[row[11],row[12],row[13]]])
				# #q = [float(row[4]),float(row[5]),float(row[6]),float(row[7])]
				# #Quat.append(q)
				Translation.append(t)
    
	#Output = [[0 for row in range(8)] for column in range(len(KFTime))]
	Output = [[0 for row in range(4)] for column in range(len(KFTime))]
	
	with open(outfile, 'w', newline='') as csvfile3:
		writer = csv.writer(csvfile3, delimiter=' ')
		count = 0
    	
		for i in range(len(KFTime)):
			TUM_Output(count, Output, KFTime, Translation, Quat)		
			count+=1
    		
		writer.writerows(Output)

LocalizationModeClip(Reference, LocalizationModeFile, LocalizationOutput)
print(" ")
print("     Localization Mode Trajectory successfully saved.")