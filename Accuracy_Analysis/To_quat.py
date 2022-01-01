from scipy.spatial.transform import Rotation as R
import csv

readfile = 'E:\\Map\\FUJI_AFTN_Intrinsic\\FUJI_0304_Cloudy_AFTN_CW_Walk\\Metashape_Cam_FUJI_0304_Cloudy_AFTN_CW_Walk.txt'
reference = 'E:\\Map\\FUJI_AFTN_Intrinsic\\FUJI_0304_Cloudy_AFTN_CW_Walk\\Wall_KFTraj_TUM.csv'
outfile = 'Metashape_Cam_FUJI_0304_Cloudy_AFTN_CW_Walk_TUM.csv'

Translation = []
Quat = []
Time = []


with open(reference, newline='') as csvfile:
    rows = csv.reader(csvfile, delimiter=' ')
    for row in rows:    
        Time.append(row[0])

with open(readfile, newline='') as csvfile2:

  rows = csv.reader(csvfile2, delimiter=',')

  for row in rows:
    if row[0][0] == '#':
	    pass
    else:
        t = [float(row[1]),float(row[2]),float(row[3])]
        r = R.from_matrix([[row[7], row[8], row[9]],[row[10], row[11],row[12]],[row[13],row[14],row[15]]])
        mat = r.as_quat()
        Quat.append(mat)
        Translation.append(t)
print(Quat)
Output = [[0 for _ in range(8)] for _ in range(len(Time))]

with open(outfile, 'w', newline='') as csvfile3:
    writer = csv.writer(csvfile3, delimiter=' ')
    count = 0
    for i in range(len(Time)):
        print(count)
        print(Time[count])
        Output[count][0] = float(Time[count])
        Output[count][1] = float(Translation[count][0])
        Output[count][2] = float(Translation[count][1])
        Output[count][3] = float(Translation[count][2])
        Output[count][4] = float(Quat[count][0])
        Output[count][5] = float(Quat[count][1])
        Output[count][6] = float(Quat[count][2])
        Output[count][7] = float(Quat[count][3])
        count+=1
        print(Output)
    writer.writerows(Output)