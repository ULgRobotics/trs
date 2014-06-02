#!/usr/bin/env python

#[0]
#shape: box
#color: 0 255 0
#colorname: green
#picture: pictures/clock.png

s = 'inst = []; '

import ConfigParser

config = ConfigParser.ConfigParser()
config.read('instructions.cfg')
for i in range(0,10):
  shape = config.get(str(i), "shape")
  color = config.get(str(i), "color")
  colorname = config.get(str(i), "colorname")
  picture = config.get(str(i), "picture")
  s += "inst = [ inst struct('shape', '{shape}', 'color', [{color}], 'colorname', '{colorname}', 'picture', '{picture}') ]; ".format(shape=shape, color=color, colorname=colorname, picture=picture)

print s
  
