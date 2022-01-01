#coding:utf-8
#python 2

import os
import cv2
import moviepy.editor as mpy
import moviepy.video.fx.all as vfx
import os
import imageio
os.chdir('Video')

Name = ['Corr', 'Mix', 'Bright', 'Dark', 'Round', 'Round', 'Board']
start = [0, 14, 0, 14, 11, 5, 3]
end = [4, 20, 27,  20, 24, 7, 6]

def getname(num):
	if num<10:
	    strRes = str(num)
	elif num<100:
	    strRes = str(num)
	elif num<1000:
	    strRes = str(num)
	elif num<10000:
	    strRes = str(num)
	return strRes

for i in range(len(Name)):
    Input_Video = Name[i]
    Output_GIF = Name[i]
    if i==5:
        Output_GIF = Name[i] + '_Straight'
    FPS_SET = 30
    foldername = Name[i]

    start_sec = start[i]
    end_sec = end[i]
    start_min = 00
    end_min = 00

    try:
        os.mkdir(foldername)
    except OSError:
        if i == 5:
	    print('I == 5')
	else:
            print(foldername + " : Folder already exist")
	    continue

    clip = mpy.VideoFileClip(Input_Video + '.mp4') 
    content = clip.subclip((start_min, start_sec),(end_min, end_sec))
    clip = content.resize(newsize=(480,270))
    content = vfx.blackwhite(clip, RGB = None, preserve_luminosity = True)
    #content.write_gif(Output_GIF + '.gif', fps = FPS_SET)
    content.write_videofile(Output_GIF+ '_cut' + '.mp4', audio = None, fps = FPS_SET)

    cap = cv2.VideoCapture(Input_Video + '_cut' + '.mp4')
    if i == 5:
	cap = cv2.VideoCapture(Output_GIF + '_cut' + '.mp4')
	print(Output_GIF)
    success, frame = cap.read()
    idx = 0
    framecounts = cap.get(7)

    while cap.isOpened:
	cv2.imwrite(foldername + '/' + getname(idx) + '.png', frame)
	success, frame = cap.read()
	idx += 1	

        processed_percent = float(idx)/framecounts
        
        if idx%100 == 0:
	    print str(processed_percent) + '% of frames has been processed' 

	if idx >= framecounts:
	    print('PNG Saved finished !')
	    break

    dirs = os.listdir(foldername)
    dirs.sort(key = lambda x:int(x[:-4]))

    FrameDB = []
    count = 0

    for file in dirs:
	filename = file
        if count%2 != 0:
	    FrameDB.append(filename)
	count += 1
    
    frames = []
    for image_name in FrameDB:
	im = imageio.imread(foldername + '/' + image_name)
	frames.append(im)

    imageio.mimsave(Output_GIF + '.gif' , frames, 'GIF', duration = 0.03)#90/count)
    print('GIF Saved finished !')
    
    command = "rmdir /s /q %s "
    command = command % foldername
    result = os.system(command)

    if result == 0:
	print("Folder Deleted !")
    else:
	print("Folder Delete Failed !")
