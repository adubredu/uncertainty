#!/usr/bin/env python3

f = open('init_coordinates.txt','r')
content = f.read()
stages = content.split('*')
coords = stages[0].split('\n')
xyz = []
for coord in coords:
	abc = coord.split(' ')
	xyz.append(abc)
xyz= xyz[1:]
xyz = xyz[:-1]


for name, x,y,z in xyz:
	print(name)