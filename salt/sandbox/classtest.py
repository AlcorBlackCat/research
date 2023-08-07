#!/usr/bin/env python3
# -*- coding: utf-8 -*-

class human:
    def __init__(self):
        print("A");
    def pr(self):
        print("AA")

class wolf(human):
    def __init__(self):
        print("B");
    def pr(self):
        print("BB")

box=[]

box.append(human())
box.append(wolf())

if type(box[0])==human:
    print("C")
else:print("D")

for i in box:
    i.pr()

print(box)
