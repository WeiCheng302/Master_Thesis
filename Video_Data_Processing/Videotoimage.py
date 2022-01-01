#Python code to transfer the video to image
#Involve the generation of needed file for TUM and KITTI
#coding:utf-8
#python 2

import os
import cv2

def getname(num):
	if num<10:
	    strRes = '00000' + str(num)
	elif num<100:
	    strRes = '0000'+ str(num)
	elif num<1000:
	    strRes = '000'+ str(num)
	elif num<10000:
	    strRes = '00'+ str(num)
	return '0' + strRes 
	
#Input Video and Folder Names
cap = cv2.VideoCapture('Afternoon/FUJI_0304_Cloudy_AFTN_Cali.MOV')

outterfoldername = 'Afternoon'
foldername = 'Calibration'

img_format = 'png'

#Build the Folders
homedir = os.getcwd()

try:
    os.mkdir(outterfoldername)
    os.chdir(outterfoldername)
except OSError:
    os.chdir(outterfoldername)

try:
    os.mkdir(foldername)
    os.chdir(foldername)
except OSError:
    os.chdir(foldername)

os.chdir(homedir)

#Get the FPS and Size
fps = cap.get(5)
size = (int(cap.get(3)), int(cap.get(4)))
framecounts = cap.get(7)

#ID_Sort = [34,232,430,601,745,916,1096,1267,1447,1582,1735,1915,2077,2257,2509,3238,3481,3625,3778,3931,4102,4264,4462,4579,4723,4849,4984,5128,5272,5425,5578]

print (' ')
print ('FPS =' ,fps)
print ('Size = ', size)
print ('Num of Frames = ', framecounts)

#Frame Reading

#success, frame = cap.read()
idx = 0

while cap.isOpened:
	#frame = cv2.resize(frame, (960, 540),interpolation = cv2.INTER_CUBIC)
	success, frame = cap.read()
	#for i, element in enumerate(ID_Sort):
	#if idx == element:
	#if idx%15 == 0:
	cv2.imwrite(outterfoldername + '/' + foldername + '/' + getname(idx) + '.' + img_format, frame)
	
	idx += 1	
	processed_percent = float(idx)/framecounts
	if idx%100 == 0:
		print (str(processed_percent) + '% of frames has been processed' )

	if idx >= framecounts:
	    print( img_format + ' Saved finished !')
	    break
	
	#cv2.imshow('frame', frame)
	#cv2.waitKey(10)

