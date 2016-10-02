#!/usr/bin/env python
# -*- coding: utf-8 -*-

import glob
import sys
import subprocess
import os
import commands
from subprocess import check_call

import povray_renderer
import profile


# 実行ファイルを確認
# povray = "python povray_renderer.py " + str(step) + ' ' + str(1)
# subprocess.check_call(povray.split(" "))
#
# # 粘性を変えてもう一回
# povray = "python povray_renderer.py " + str(step) + ' ' + str(10)
# subprocess.check_call(povray.split(" "))
#
# profile = "python profile_distribution.py -step " + str(step) + " -distribusion -collapse_curve 0"
# subprocess.check_call(profile.split(" "))
#
# #mergedMovie = "ffmpeg -i distribution.mp4 -i mohr_stress_circle0.mp4 -i povray.mp4 -i povray.mp4 -filter_complex 'nullsrc=size=1920x1440 [base]; [0:v] setpts=PTS-STARTPTS, scale=1920x1440 [frame0];[1:v] setpts=PTS-STARTPTS, scale=640x480 [frame1]; [base] [frame0] overlay=shortest=1:x=1920:y=1440 [tmp1]; [tmp1] [frame1] overlay=shortest=1:x=640:y=480' comp.mp4"
# mergeMovie = "./merge_movie.sh"
# subprocess.check_call(mergedMovie.split(" "))

if __name__ == '__main__':
    # 作業フォルダを取得
    current = os.getcwd()

    argv = sys.argv
    argc = len(argv)
    step = 1
    if argc > 1:
        step = argv[1]

    #実行
    povray_renderer.executeCommand()
    profile.executeCommand('-distribution -collapse_curve 0 ' + '-step ' + str(step) +  ' -output ' + current)

    #画像をレンダリング
    povray_renderer.render(False)
    profile.renderStressCircle(0, False)
    profile.renderDistribusion(False)

    #余分なファイルを削除
    povray_renderer.removeTempFiles()
    profile.removeTempFiles()