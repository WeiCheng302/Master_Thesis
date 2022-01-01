import moviepy.editor as mpy
import moviepy.video.fx.all as vfx
import os
os.chdir('Video')

Name = ['Mix', 'Bright', 'Corr', 'Dark', 'Round', 'Round', 'Board']
start = [14, 0, 0, 14, 11, 5, 3]
end = [20, 27, 4, 20, 24, 7, 6]

for i in range(len(Name)):
    Input_Video = Name[i]
    Output_GIF = Name[i]
    if i==6:
        Output_GIF = Name[i] + '_Straight'
    FPS_SET = 90

    start_sec = start[i]
    end_sec = end[i]
    start_min = 00
    end_min = 00

    clip = mpy.VideoFileClip(Input_Video + '.mp4') 
    content = clip.subclip((start_min, start_sec),(end_min, end_sec))
    clip = content.resize(newsize=(480,270))
    content = vfx.blackwhite(clip, RGB = None, preserve_luminosity = True)

    content.write_gif(Output_GIF + '.gif', fps = FPS_SET)
