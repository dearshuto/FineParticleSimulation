#!/usr/bin/env python
# -*- coding: utf-8 -*-

import fnmatch
import glob
import sys
import subprocess
import os
import commands
from subprocess import check_call

def executeCommand(commandArguments):
    # 作業フォルダを取得
    current = os.getcwd()

    # 実行ファイルを確認
    profiler = current + "/../bin/Release/profiling_example"
    if not os.path.exists(profiler):
        print 'Build source before using me'
        exit();

    # 外部コマンド実行
    cmd = profiler + ' ' + commandArguments
    subprocess.check_call(cmd.split(" "))

## ----  distribution profile ------------------------------------------------------------------------------------------
def renderDistribusion(withMovie = True):
    distributionFile = glob.glob('./distribution_*.gnuplot')
    if len(distributionFile) > 0:
        for file in distributionFile:
            os.system('gnuplot -c ' + file)

        # distributionの動画作成
        if withMovie :
            movieName = 'distribution.mp4'
            if os.path.exists(movieName):#すでにあったら消しといて
                os.remove(movieName)
            os.system('ffmpeg -i distribution_%d.png -an -vcodec libx264 -pix_fmt yuv420p ' + movieName)


## ----  Mohr's Stress Circle Profile ----------------------------------------------------------------------------------
def renderStressCircle(target, withMovie = True):
    mohr = glob.glob('./particle*.gnuplot')
    if len(mohr) > 0:
        for file in mohr:
            os.system('gnuplot -c ' + file)

        # distributionの動画作成
        if withMovie :
            movieName = 'mohr_stress_circle' + str(target) + '.mp4'

            if os.path.exists(movieName):#すでにあったら消しといて
                os.remove(movieName)
            os.system('ffmpeg -i particle' + str(target) + '_%d.png -an -vcodec libx264 -pix_fmt yuv420p ' + movieName)


def removeTempFiles():
    for file in glob.glob('*.gnuplot'):
        os.remove(file)
    for file in glob.glob('*.data'):
       os.remove(file)

if __name__ == '__main__':
    argv = sys.argv
    argc = len(argv)

    if argc < 2:
        print 'arguments are...'
        print '-min_max_average_time'
        print '-distribusion'
        print '-collapse_curve'

    i = 1
    target = 0
    commandOption = ''
    while i < argc:
        commandOption = commandOption + ' ' + argv[i]
        if argv[i] == '-collapse_curve':
            target = argv[i+1]
        i += 1
    commandOption = commandOption + ' -output ' + os.getcwd()

    executeCommand(commandOption)
    renderDistribusion()
    renderStressCircle(target)

    #不要なファイルは消しておく
    removeTempFiles()
    for file in glob.glob('*.png'):
        os.remove(file)
