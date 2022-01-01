import xml.etree.ElementTree as ET
import csv

tree = ET.parse('E://Map//0601_RS//Markers.xml')
CSVout = open('E://Map//0601_RS//Image Measurement.csv', 'w', newline='')
writer = csv.writer(CSVout, delimiter=' ')

writer.writerow(['CPID', 'CP_X', 'CP_Y', 'CP_Z', 'KFID', 'KF_X', 'KF_Y'])
root = tree.getroot()		
cameras = root[0][1]
markers = root[0][2]
frames = root[0][8]

for marker in markers:
	
	marker_label = marker.get('label')
	marker_id = marker.get('id')
	marker_x = marker[0].get('x')
	marker_y = marker[0].get('y')
	marker_z = marker[0].get('z')
	
	x = []
	y = []
	kfID = []
	KFinfo = []
	camID = []
	count = 0

	for i in frames[0][0]:
		if i.get('marker_id') == marker_id:
			x = [item.get('x') for item in i if item.get('x') != None]
			y = [item.get('y') for item in i if item.get('y') != None]
			kfID = [item.get('camera_id') for item in i if item.get('camera_id') != None]
	
	for i in kfID:
		for camera in cameras :
			if camera.get('id')==i :
				camID.append(camera.get('label'))
		
	for i in range(len(x)):
		KFinfo.append(int(camID[count][0:-4]))
		KFinfo.append(x[count])
		KFinfo.append(y[count])		
		count += 1
		
	writer.writerow([marker_label,'{:.3f}'.format(float(marker_x)),'{:.3f}'.format(float(marker_y)),'{:.3f}'.format(float(marker_z))] + [item for item in KFinfo] )	

CSVout.close()