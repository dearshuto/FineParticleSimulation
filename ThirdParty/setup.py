#!/usr/bin/env python
# -*- coding: utf-8 -*-

import shutil
import os
import glob


# my code
path = 'CubicFineParticle/src/'
target = os.path.join(os.getcwd(), path + 'include/')
at = os.path.join(os.getcwd(), 'CubicFineParticle/Includes')

# コードを階層を維持したままコピー
try:
    shutil.copytree(target, at)
except OSError:
    shutil.rmtree(at)
    shutil.copytree(target, at)

# .libをコピー
libpath = os.path.join(os.getcwd(), path + 'lib/Release')

shutil.rmtree('CubicFineParticle/Libraries')
shutil.copytree(libpath, 'CubicFineParticle/Libraries')


# Bullet Physics
shutil.rmtree('BulletPhysics/Libraries')
shutil.copytree(libpath, 'BulletPhysics/Libraries')

bpath = 'CubicFineParticle/src/vendor/bullet/src'
btarget = os.path.join(os.getcwd(), bpath)
bat = os.path.join(os.getcwd(), 'BulletPhysics/Includes')

# ソースコードをディレクトリごとコピー
try:
    shutil.copytree(btarget, bat)
except OSError:
    shutil.rmtree(bat)
    shutil.copytree(btarget, bat)

# ヘッダー以外消去
for root, dirs, files in os.walk(bat):
    for file_ in files:
        [os.remove(f) for f in glob.glob(root + '/*.cpp')]
        [os.remove(f) for f in glob.glob(root + '/*.c')]
        [os.remove(f) for f in glob.glob(root + '/*.txt')]
        [os.remove(f) for f in glob.glob(root + '/*.lua')]
