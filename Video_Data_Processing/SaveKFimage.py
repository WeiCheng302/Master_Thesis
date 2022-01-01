#Python code to transfer the video to image
#Involve the generation of needed file for TUM and KITTI
#coding:utf-8
#python 2

import os
import cv2
import pandas as pd

def getname(num):
	if num<10:
	    strRes = '00000' + str(num)
	elif num<100:
	    strRes = '0000'+ str(num)
	elif num<1000:
	    strRes = '000'+ str(num)
	elif num<10000:
	    strRes = '00'+ str(num)
	return strRes
	
#Input Video and Folder Names
InputVideo = 'FUJI_0304_Cloudy_AFTN_CW_Walk.MOV'
cap = cv2.VideoCapture(InputVideo)
f = pd.read_csv("E:\Map\FUJI_AFTN_Intrinsic\FUJI_0304_Cloudy_AFTN_CW_Walk\Wall_KF.txt" header = 0  delim_whitespace = True)

outterfoldername = InputVideo[0:-4]
foldername = 'Keyframe'

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
size = (int(cap.get(3))  int(cap.get(4)))
framecounts = cap.get(7)

ID_Sort = f["FrameID"]

print (' ')
print ('FPS ='  fps)
print ('Size = '  size)
print ('Num of Frames = '  framecounts)

#Frame Reading

#success  frame = cap.read()
idx = 0
FrameIDcount = 0

while cap.isOpened:
	success  frame = cap.read()
	for i  element in enumerate(ID_Sort):
		if idx == element:
		#frame = cv2.resize(frame  (960  540) interpolation = cv2.INTER_CUBIC)
			try:
				cv2.imwrite(outterfoldername + '/' + foldername + '/' + getname(idx) + '.png'  frame)
			except cv2.error:
				print(123)
				continue
			print(idx)
			print(FrameIDcount)
			print("==============")
        
	FrameIDcount += 1
	idx += 1	

	processed_percent = float(idx)/framecounts
		
	if idx%100 == 0:
		print(str(processed_percent) + '% of frames has been processed')

	if idx >= framecounts:
	    print('PNG Saved finished !')
	    break
	
	#cv2.imshow('frame'  frame)
	#cv2.waitKey(10)
