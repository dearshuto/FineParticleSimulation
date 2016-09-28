#!/usr/bin/env python
# -*- coding: utf-8 -*-

import fnmatch
import glob
import sys
import subprocess
import os
import commands
from subprocess import check_call

# 作業フォルダを取得
current = os.getcwd()

# 実行ファイルを確認
profiler = current + "/../bin/Release/profiling_example"
if not os.path.exists(profiler):
    print 'Build source before using me'
    exit();

argv = sys.argv
argc = len(argv)

if argc < 2:
    print 'arguments are...'
    print '-min_max_average_time'
    print '-distribusion'
    print '-collapse_curve'

i = 1
commandOption = ''
while i < argc:
    commandOption = commandOption + ' ' + argv[i]
    i += 1
commandOption = commandOption + ' -output ' + current

# 外部コマンド実行
cmd = profiler + commandOption
subprocess.check_call(cmd.split(" "))

## ----  distribution profile ------------------------------------------------------------------------------------------
distributionFile = glob.glob('./distribution_*.gnuplot')
if len(distributionFile) > 0:
    for file in distributionFile:
        os.system('gnuplot -c ' + file)

#    distributionFile = glob.glob('distribution*')

    # distributionの動画作成
    movieName = 'distribution.mp4'

    if os.path.exists(movieName):#すでにあったら消しといて
        os.remove(movieName)
    os.system('ffmpeg -i distribution_%d.png -an -vcodec libx264 -pix_fmt yuv420p ' + movieName)


## ----  Mohr's Stress Circle Profile ----------------------------------------------------------------------------------
mohr = glob.glob('./particle*.gnuplot')
if len(mohr) > 0:
    for file in mohr:
        os.system('gnuplot -c ' + file)

#    distributionFile = glob.glob('distribution*')

    n = 0
    while i < argc:
        if argv[i] == -collapse_curve:
            n = argv[i+1]
            break
    i += 1
    # distributionの動画作成
    movieName = 'mohr_stress_circle' + str(n) + '.mp4'

    if os.path.exists(movieName):#すでにあったら消しといて
        os.remove(movieName)
    os.system('ffmpeg -i particle' + str(n) + '_%d.png -an -vcodec libx264 -pix_fmt yuv420p ' + movieName)



#不要なファイルは消しておく
for file in glob.glob('*.png'):
    os.remove(file)
for file in glob.glob('*.gnuplot'):
    os.remove(file)
for file in glob.glob('*.data'):
   os.remove(file)