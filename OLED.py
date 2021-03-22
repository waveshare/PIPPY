#!/usr/bin/env/python3
# File name   : OLED.py
# Description : for OLED functions

import time

from waveshare_OLED import OLED_0in91
from PIL import Image,ImageDraw,ImageFont

import threading

# Raspberry Pi pin configuration:
disp = OLED_0in91.OLED_0in91()
disp.Init()
disp.clear()

# Create blank image for drawing.
# Make sure to create image with mode '1' for 1-bit color.
width = disp.width
height = disp.height
image = Image.new('1', (width, height), "BLACK")

# Get drawing object to draw on image.
draw = ImageDraw.Draw(image)

# Draw some shapes.
# First define some constants to allow easy resizing of shapes.
padding = -2
top = padding
bottom = height-padding
# Move left to right keeping track of the current x position for drawing shapes.
x = 0

# Load default font.
font = ImageFont.load_default()

text_1 = 'IP:'
text_2 = 'VOLTAGE:'
text_3 = 'WIFI MODE:'
text_4 = 'WaveShare PIPPY'


class OLED_ctrl(threading.Thread):
	def __init__(self, *args, **kwargs):
		super(OLED_ctrl, self).__init__(*args, **kwargs)
		self.__flag = threading.Event()	 
		self.__flag.set()	   
		self.__running = threading.Event()	  
		self.__running.set()

	def run(self):
		while self.__running.isSet():
			self.__flag.wait()
			try:
				draw.rectangle((-1,-1,width,height), outline=0, fill=255)
				draw.text((x, top),       text_1,  font=font, fill=0)
				draw.text((x, top+8),     text_2,  font=font, fill=0)
				draw.text((x, top+16),    text_3,  font=font, fill=0)
				draw.text((x, top+25),    text_4,  font=font, fill=0)

				# Display image.
				disp.ShowImage(disp.getbuffer(image))
				time.sleep(.1)
			except:
				pass

			print('loop')
			self.pause()

	def pause(self):
		self.__flag.clear()	 

	def resume(self):
		self.__flag.set()	

	def stop(self):
		self.__flag.set()	   
		self.__running.clear()		  

	def screen_show(self, position, text):
		global text_1, text_2, text_3, text_4, text_5, text_6
		if position == 1:
			text_1 = text
		elif position == 2:
			text_2 = text
		elif position == 3:
			text_3 = text
		elif position == 4:
			text_4 = text
		self.resume()

if __name__ == '__main__':
	screen = OLED_ctrl()
	screen.start()
	screen.screen_show(1, 'IP:192.168.12.1')
	while 1:
		time.sleep(3)
		screen.screen_show(1, 'IP:192.168.4.1')
		screen.screen_show(2, 'VOLTAGE:8.4v')
		pass