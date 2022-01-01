from moviepy.editor import *
import moviepy.video.fx.all as vfx
import cv2
from moviepy.editor import ImageSequenceClip
import moviepy.editor as mpy
import os

#videoname2 = ['Init_with_turn','LPS_1Round','LPS_cloudier','LPS_cloudy','LPS_dark','LPS_multi_close']
#videoname2 = ['bikenextday01','bikenextday02','geomatics','wallnextday02','wallnextday01']
#videoname2 = ['Alley','Lab2','Lab3','Lab4','Round_clockwise','Round_side_counterclockwise','Side_Back','Side_Front','Straight_Back','Straight_Front','Straight_sidebuilding']
videoname2 = ['Quiren_4K_1','Quiren_4K_2',]

homedir = os.getcwd()
for name in videoname2:
    os.chdir('Video')
    os.chdir('Quiren')
    Video_Name = name

    Video_Name_in = Video_Name + '.MOV'
    Video_Name_out = Video_Name + '.mp4'

    video = VideoFileClip(Video_Name_in)
    video = video.without_audio()
    #os.chdir('wall')
    video.write_videofile(Video_Name_out)
    
    print('finish processing :')
    print(name)
    os.chdir(homedir)

videoname3 = ['Quiren_4K_1','Quiren_4K_1','Quiren_4K_2','Quiren_4K_2',]
start_min = [0,1,0,1]
start_sec = [0,43,0,15]
end_min = [1,3,1,3]
end_sec = [53,3,50,30]
a = 0
for i in range(len(videoname3)):
    print("Clipping")
    os.chdir('Video/Quiren')
    Input_Video = videoname3[a]
    Output_GIF = videoname3[a]+str(a)
    
    start_mins = start_min[a]
    start_secs = start_sec[a]
    end_mins = end_min[a]
    end_secs = end_sec[a]
    a += 1

    clip = mpy.VideoFileClip(Input_Video + '.mp4') 
    content = clip.subclip((start_mins, start_secs),(end_mins, end_secs))
    content = vfx.blackwhite(clip, RGB = None, preserve_luminosity = True)

    content.write_videofile(Output_GIF + '.mp4')
    os.chdir(homedir)
