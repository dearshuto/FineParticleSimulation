#!/usr/bin/env python
# -*- coding: utf-8 -*-

import glob
import sys
import subprocess
import os
import commands
from subprocess import check_call

# 作業フォルダを取得
current = os.getcwd()

argv = sys.argv
argc = len(argv)
step = 1
if argc > 1:
    step = argv[1]

# 実行ファイルを確認
povray = "python povray_renderer.py " + str(step)
subprocess.check_call(povray.split(" "))

# 粘性を変えてもう一回
#povray = "python povray_renderer.py " + str(step) + ' ' + " "
#subprocess.check_call(povray.split(" "))

profile = "python profile_distribution.py -step " + str(step) + " -distribusion -collapse_curve 0"
subprocess.check_call(profile.split(" "))

mergedMovie = "ffmpeg -i distribution.mp4 -i mohr_stress_circle0.mp4 -i povray.mp4 -i povray.mp4 -filter_complex 'nullsrc=size=1920x1440 [base]; [0:v] setpts=PTS-STARTPTS, scale=1920x1440 [frame0];[1:v] setpts=PTS-STARTPTS, scale=640x480 [frame1]; [base] [frame0] overlay=shortest=1:x=1920:y=1440 [tmp1]; [tmp1] [frame1] overlay=shortest=1:x=640:y=480' comp.mp4"
subprocess.check_call(mergedMovie.split(" "))

# povFile = glob.glob('./*.pov')
# if len(povFile) > 0:
#     for file in povFile:
#         os.system('povray ' + file)
#
# #    distributionFile = glob.glob('distribution*')
#
#     # distributionの動画作成
#     movieName = 'povray.mp4'
#
#     if os.path.exists(movieName):#すでにあったら消しといて
#         os.remove(movieName)
#     os.system('ffmpeg -i %d.png -an -vcodec libx264 -pix_fmt yuv420p ' + movieName)
#
# #不要なファイルは消しておく
# for file in glob.glob('*.pov'):
#     os.remove(file)
# for file in glob.glob('*.png'):
#     os.remove(file)
