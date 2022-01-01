from scipy.spatial.transform import Rotation as R
import csv

def TUM_Output(count, Output, Time, Translation, Quat):
    Output[count][0] = str(Time[count])
    Output[count][1] = float(Translation[count][0])
    Output[count][2] = float(Translation[count][1])
    Output[count][3] = float(Translation[count][2])
    Output[count][4] = float(Quat[count][0])
    Output[count][5] = float(Quat[count][1])
    Output[count][6] = float(Quat[count][2])
    Output[count][7] = float(Quat[count][3])

reference = 'E:\\Map\\0601_RS\\KFTraj_TUM.txt'
outfile = 'E:\\Map\\0601_RS\\SLAM_Traj_TUM.txt'

Translation = []
Quat = []
Time = []

with open(reference, newline='') as csvfile:  
    Time = [row[1] for row in csv.reader(csvfile, delimiter=' ') if row[0][0]!='K']

with open(reference, newline='') as csvfile2:

  rows = csv.reader(csvfile2, delimiter=' ')

  for row in rows:
        if row[0][0] != 'K':
            t = [float(row[2]),float(row[3]),float(row[4])]
            r = R.from_matrix([[row[5], row[6], row[7]],[row[8], row[9],row[10]],[row[11],row[12],row[13]]])
            Quat.append(r.as_quat())
            Translation.append(t)

Output = [[0 for row in range(8)] for column in range(len(Time))]

with open(outfile, 'w', newline='') as csvfile3:
    writer = csv.writer(csvfile3, delimiter=' ')
    count = 0
	
    for i in range(len(Time)):
        TUM_Output(count, Output, Time, Translation, Quat)		
        count+=1
		
    writer.writerows(Output)
	
