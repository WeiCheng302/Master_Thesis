#This program is to transform the data format to the necessary format for further analysis
#Including (1) The Original dirty SLAM output to TUM format
#          (2) Ground Truth Trajectory from Metashape to TUM format
#          (3) GCPs information (2D/3D) Measurement
import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation as R
import csv
import os

Root_Folder = 'E:\\Map\\Side\\Analysis\\Side_Afternoon\\'

#Markers
Tree = ET.parse( Root_Folder + 'Markers.xml') # Metashape output to get GCPs information

#SfM Trajectory
Readfile = Root_Folder + 'SfM_Traj.txt' # SfM Ground Truth Trajectory in Metashape Format
Reference = Root_Folder + 'KFTraj_TUM.txt' # SLAM Trajectory in a Dirty format

#SLAM Trajectory and Outputs
OutFile_SfM = Root_Folder + 'SfM_Traj_TUM.txt' # GroundTruth Trajectory in TUM format
CsvOut = open(Root_Folder + 'Image Measurement.csv', 'w', newline='') # Image Measurements of GCPs and its XYZ
CsvOut2 = open(Root_Folder + 'Image Measurement XYZ.csv', 'w', newline='') # XYZ of GCPs
OutFile_SLAM = Root_Folder + 'SLAM_Traj_TUM.txt' # SLAM Trajectory in TUM format

#Redundency Inages
Redundent = Root_Folder + 'Redundency.txt'

def IsRedundency(camID, redundentcamID):
	check = False	
	for element in redundentcamID:
		if camID == element:
			check = True
	return check	
	
def Getname(num):

	if float(num)<10:
	    strRes = '00000' + str(num)
	elif float(num)<100:
	    strRes = '0000'+ str(num)
	elif float(num)<1000:
	    strRes = '000'+ str(num)
	elif float(num)<10000:
	    strRes = '00'+ str(num)
	return strRes

def GetRedundent(Redudent, Reference):
    Redundent_List = []
    Reference_FrameID = []
    count = -1

    with open(Reference, newline='') as csvfile0:
        for row in csv.reader(csvfile0, delimiter = ' '):
            if count == -1:
                count += 1
                continue
            Reference_FrameID.append(Getname(row[-1]) + '.png')

    with open(Redundent, newline='') as csvfile:  
        for row in csv.reader(csvfile, delimiter=','):
            for element in row:
                begin = 2
                if element[0] != " ":
                    begin = 1
                if not IsRedundency(element[begin:-1], Reference_FrameID):
                    Redundent_List.append(element[begin:-1])
                count += 1

    return Redundent_List

def TUM_Output(count, Output, Time, Translation, Quat):
    Output[count][0] = str(Time[count])
    Output[count][1] = float(Translation[count][0])
    Output[count][2] = float(Translation[count][1])
    Output[count][3] = float(Translation[count][2])
    Output[count][4] = float(Quat[count][0])
    Output[count][5] = float(Quat[count][1])
    Output[count][6] = float(Quat[count][2])
    Output[count][7] = float(Quat[count][3])
	
def GetMarkers(tree, CsvOut, CsvOut2):
    
    writer = csv.writer(CsvOut, delimiter=' ')
    writer2 = csv.writer(CsvOut2, delimiter=' ')
    
    writer.writerow(['CPID', 'CP_X', 'CP_Y', 'CP_Z', 'KFID', 'KF_X', 'KF_Y'])
    writer2.writerow(['CPID', 'CP_X', 'CP_Y', 'CP_Z'])
    
    RedundencyList = GetRedundent(Redundent, Reference)
	
    root = tree.getroot()		
    cameras = root[0][1]
    markers = root[0][2]
    frames = root[0][9]
    
    for marker in markers:
    	
    	#Get the information of Control Points 
    	marker_label = marker.get('label')
    	marker_id = marker.get('id')
    	marker_x = marker[0].get('x')
    	marker_y = marker[0].get('y')
    	marker_z = marker[0].get('z')

    	x = []
    	y = []
    	kfID = []
    	KFinfo = []
    	redundentcamID = []
    	camID = []
    	count = 0
		
		#Get the Camera ID of the Redundency Image
    	for camera in cameras:
    		for i, element in enumerate(RedundencyList):
    			if camera.get('label') == element:
    				redundentcamID.append(camera.get('id'))
					
		#Get the image measurements of the Control Points on Each Image if not Redundent
    	for i in frames[0][0]:
    		if i.get('marker_id') == marker_id:
    			x = [item.get('x') for item in i if (item.get('x') != None and not IsRedundency(item.get('camera_id'), redundentcamID))]
    			y = [item.get('y') for item in i if (item.get('y') != None and not IsRedundency(item.get('camera_id'), redundentcamID))]
    			kfID = [item.get('camera_id') for item in i if (item.get('camera_id') != None and not IsRedundency(item.get('camera_id'), redundentcamID))]
    	
    	for i in kfID:
    		for camera in cameras :
    			if camera.get('id')==i :
    				camID.append(camera.get('label'))
    		
		#Write image measurements of the Control Points into output files
    	for i in range(len(x)):
    		KFinfo.append(int(camID[count][0:-4]))
    		KFinfo.append(x[count])
    		KFinfo.append(y[count])		
    		count += 1
			
    	writer.writerow([marker_label,'{:.3f}'.format(float(marker_x)),'{:.3f}'.format(float(marker_y)),'{:.3f}'.format(float(marker_z))] + [item for item in KFinfo] )	
    	writer2.writerow([marker_label,'{:.3f}'.format(float(marker_x)),'{:.3f}'.format(float(marker_y)),'{:.3f}'.format(float(marker_z))])
    
    CsvOut.close()
    CsvOut2.close()
	
def SfMTraj_toTUM(readfile, reference, outfile):
    Translation = []
    Quat = []
    Time = []
    RedundentList = GetRedundent(Redundent, Reference)
    
    with open(reference, newline='') as csvfile:  
        Time = [row[1] for row in csv.reader(csvfile, delimiter=' ')  if row[0][0]!='K']
    
    with open(readfile, newline='') as csvfile2:
    
        rows = csv.reader(csvfile2, delimiter='\t')
    
        for row in rows:
            if row[0][0] != '#' and not IsRedundency(row[0], RedundentList):
                t = [float(row[1]),float(row[2]),float(row[3])]
                r = R.from_matrix([[row[7], row[8], row[9]],[row[10], row[11],row[12]],[row[13],row[14],row[15]]])
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
		
def SLAMTraj_toTUM(reference, outfile):
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
		
		
GetMarkers(Tree, CsvOut, CsvOut2)
SfMTraj_toTUM(Readfile, Reference, OutFile_SfM)
SLAMTraj_toTUM(Reference, OutFile_SLAM)

print(" ")
print("     Markers, SfM Trajectory and SLAM Trajectory are saved.")