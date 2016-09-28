import sys
import os
from subprocess import check_call

# フォルダの存在をテェック. なければ作る
directory = "./distribution"
if not os.path.exists(directory):
    os.mkdir(directory)

# 実行ファイルの存在をチェック
profiler = "../bin/Release/profiling_example"
if not os.path.exists(profiler):
    print 'Build source before using me'
    exit();

# コマンドライン引数を取得
argv = sys.argv
argc = len(argv)
i = 1
commandOption = ''
while i < argc:
    commandOption = argv[i] + ' '
    i += 1

# 外部コマンドを実行
os.system('ls ' + commandOption)

