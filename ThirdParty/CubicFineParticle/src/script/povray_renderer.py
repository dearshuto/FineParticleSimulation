#!/usr/bin/env python
# -*- coding: utf-8 -*-

import glob
import sys
import subprocess
import os
import commands
from subprocess import check_call


def executeCommand():
    # 作業フォルダを取得
    current = os.getcwd()

    # 実行ファイルを確認
    profiler = current + "/../bin/Release/povray_example"

    argv = sys.argv
    argc = len(argv)
    frame = 0
    dashpod = 1
    if argc > 1:
        frame = argv[1]
    if argc > 2:
        dashpod = argv[2]

    cmd = str(profiler) + ' ' + str(frame) + ' ' + str(dashpod)
    print cmd
    subprocess.check_call(cmd.split(" "))

def render(withMovie = True):
    povFile = glob.glob('./*.pov')
    if len(povFile) > 0:
        for file in povFile:
            os.system('povray +W400 +H300 ' + file)

        # distributionの動画作成
        if withMovie:
            movieName = 'povray.mp4'
            if os.path.exists(movieName):
                movieName = "_" + movieName
            os.system('ffmpeg -i %d.png -an -vcodec libx264 -pix_fmt yuv420p ' + movieName)

def removeTempFiles():
    for file in glob.glob('*.pov'):
        os.remove(file)


if __name__ == '__main__':
    executeCommand()
    render()
    removeTempFiles()

    #不要なファイルは消しておく
    for file in glob.glob('*.png'):
        os.remove(file)
