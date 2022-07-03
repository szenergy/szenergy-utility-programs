#!/usr/bin/env python

import sys
import pandas as pd
import matplotlib.pyplot as plt 
import math
import utm

df = pd.read_csv(sys.argv[1])

f = open('output.csv','w')
x_cords = []
y_cords = []
yaw_values = []
yaw = None
base_x = utm.from_latlon(df.iloc[0]['Lat [deg]'],df.iloc[0]['Lon [deg]'])[0]
base_y = utm.from_latlon(df.iloc[0]['Lat [deg]'],df.iloc[0]['Lon [deg]'])[1]
f.write('x,y,z,yaw,velocity,change_flag\n')
for i in range(0,len(df)):
  if df.iloc[i]['Pos Mode'] != 0:
    x = utm.from_latlon(df.iloc[i]['Lat [deg]'],df.iloc[i]['Lon [deg]'])[0]
    y = utm.from_latlon(df.iloc[i]['Lat [deg]'],df.iloc[i]['Lon [deg]'])[1]
    euc_dist = math.sqrt(math.pow(x-base_x,2)+math.pow(y-base_y,2))
    x_cords.append(x)
    y_cords.append(y)
    z = 0 #df.iloc[i]['Alt Ellips [m]'] 
    if i <len(df)-1:
      if df.iloc[i+1]['Pos Mode'] != 0:
        next_x = utm.from_latlon(df.iloc[i+1]['Lat [deg]'],df.iloc[i+1]['Lon [deg]'])[0]
        next_y = utm.from_latlon(df.iloc[i+1]['Lat [deg]'],df.iloc[i+1]['Lon [deg]'])[1]
        yaw =  math.atan2(next_y-y,next_x-x)
    if yaw >=1.2 and yaw<=1.7 and euc_dist > 0.2:
      base_x = x
      base_y = y
      yaw_values.append(yaw)
    velocity = 20.0 #df.iloc[i]['Horiz Vel [m/s]']**2+df.iloc[i]['Vert Vel [m/s]']**2
    change_flag = df.iloc[i]['Corr. Age [s]']
    f.write(str(x)+','+str(y)+','+str(z)+','+str(yaw)+','+str(velocity)+','+str(change_flag)+'\n')
f.close()

plt.plot(yaw_values)
plt.ylabel('Angle')
plt.title('Yaw angle 0.2m-es lépésközzel')
plt.show()