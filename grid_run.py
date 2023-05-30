#!/usr/bin/env python3
# coding: utf-8

### import modules ###
import os
import numpy as np
import matplotlib.pyplot as plt

#回数
times = 2
num = 0   #初期値
ns = []
file_list = []
infile = "回目"
dir = './result(csv)'    #15、16行目のどちらを使うかによって保存されるフォルダが変わる
#dir = './fake_result(csv)'   #15、16行目のどちらを使うかによって保存されるフォルダが変わる

#実行
while(True):   #Trueである間繰り返す
    print(str(num + 1) + "回目")   #num+1と表示
    file_list.append(str(num + 1) + infile)    #file_listにnum+1回目という要素を追加
    a = np.random.randint(12345,123456)   #「全ての面で出る確率が等しいサイコロ」を投げたときの動作  12345,123455までの値
    while(True):   #Trueである限り繰り返す
        if a not in ns:    #もしaがnsというリストの要素に含まれない場合True
            ns.append(a)   #nsというリストにaという要素を追加
            os.system("python grid_simulator.py " + str(a))    # os.systemとはunixコマンドをpython上で記述する為にある、従来のモジュール
            num += 1
            break    #ループ終了
        else:   #aがnsというリストにすでにある場合
            a = np.random.randint(12345,123456)     #aを12345,123455までの値にする
    if len(os.listdir(dir)) == 50:   #もしファイル名・ディレクトリ名の一覧をリストで取得した時の長さが50ならば### 50回目終了 ###と出力する
        print("### 50回目終了 ###")
    if len(os.listdir(dir)) == times:      #もしファイル名・ディレクトリ名の一覧をリストで取得した時の長さがtimesと同じなら100回目終了と出力して終了
        print("### 100回目終了 ###")
        break
    #os.system("python argv.py " + str(a))

#print(file_list)
