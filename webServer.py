#!/usr/bin/env/python
# File name   : server.py
# Production  : PIPPY
# Author	  : WaveShare

import time
import threading
import os
import socket
import info
import OLED

#websocket
import asyncio
import websockets

import json
import app

screen = OLED.OLED_ctrl()
screen.start()


def ap_thread():
	os.system("sudo create_ap wlan0 eth0 PIPPY 12345678")


def wifi_check():
	time.sleep(5)
	try:
		s =socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
		s.connect(("1.1.1.1",80))
		ipaddr_check=s.getsockname()[0]
		s.close()
		print(ipaddr_check)
		screen.screen_show(1, 'IP:'+str(ipaddr_check))
		screen.screen_show(3, 'WIFI MODE: STA')
	except:
		ap_threading=threading.Thread(target=ap_thread)   
		ap_threading.setDaemon(True)                     
		ap_threading.start()                             
		screen.screen_show(1, 'IP:192.168.12.1')
		screen.screen_show(3, 'AP STARTING 10%')
		time.sleep(1)
		screen.screen_show(3, 'AP STARTING 20%')
		time.sleep(1)
		screen.screen_show(3, 'AP STARTING 30%')
		time.sleep(1)
		screen.screen_show(3, 'AP STARTING 40%')
		time.sleep(1)
		screen.screen_show(3, 'AP STARTING 50%')
		time.sleep(1)
		screen.screen_show(3, 'AP STARTING 60%')
		time.sleep(1)
		screen.screen_show(3, 'AP STARTING 70%')
		time.sleep(1)
		screen.screen_show(3, 'AP STARTING 80%')
		time.sleep(1)
		screen.screen_show(3, 'AP STARTING 90%')
		time.sleep(1)
		screen.screen_show(3, 'AP STARTING 100%')
		time.sleep(1)
		screen.screen_show(3, 'WIFI MODE: AP')


def batteryStatus():
	while 1:
		batteryV = app.camera_opencv.robot.getVoltage()
		screen.screen_show(2, 'VOLTAGE: {:6.2f}V'.format(batteryV))
		time.sleep(5)


async def check_permit(websocket):
	while True:
		recv_str = await websocket.recv()
		cred_dict = recv_str.split(":")
		if cred_dict[0] == "admin" and cred_dict[1] == "123456":
			response_str = "Connected!"
			await websocket.send(response_str)
			return True
		else:
			response_str = "sorry, the username or password is wrong, please submit again"
			await websocket.send(response_str)


async def recv_msg(websocket):
	while True: 
		response = {
			'status' : 'ok',
			'title' : '',
			'data' : None
		}

		data = ''
		data = await websocket.recv()
		try:
			data = json.loads(data)
		except Exception as e:
			print('not A JSON')

		if not data:
			continue

		if isinstance(data,str):
			flask_app.commandInput(data)

			if 'get_info' == data:
				response['title'] = 'get_info'
				response['data'] = [info.get_cpu_tempfunc(), info.get_cpu_use(), info.get_ram_info()]

			if 'findColor' == data:
				flask_app.modeselect('findColor')
				print('set mode as findColor')

			elif 'scan' == data:
				print('scanning')
				ds = app.camera_opencv.ultra.checkdist()
				print(ds)
				radar_send = [[3,60],[ds,70],[ds,80],[ds,90],[ds,100],[ds,110],[3,120]]
				# radar_send = []
				# for i in range(1,150):
				# 	radar_send.append[ds]
				response['title'] = 'scanResult'
				response['data'] = radar_send
				time.sleep(0.3)
				pass

			elif 'motionGet' == data:
				flask_app.modeselect('watchDog')
				print('set mode as watchDog')

			elif 'stopCV' == data:
				flask_app.modeselect('none')

			#CVFL
			elif 'CVFL' == data:
				flask_app.modeselect('findlineCV')
				print('set mode as findlineCV')

			elif 'CVFLColorSet' in data:
				color = int(data.split()[1])
				flask_app.camera.colorSet(color)

			elif 'CVFLL1' in data:
				pos = int(data.split()[1])
				flask_app.camera.linePosSet_1(pos)

			elif 'CVFLL2' in data:
				pos = int(data.split()[1])
				flask_app.camera.linePosSet_2(pos)

			elif 'CVFLSP' in data:
				err = int(data.split()[1])
				flask_app.camera.errorSet(err)

			elif 'defEC' in data:#Z
				fpv.defaultExpCom()


		elif(isinstance(data,dict)):
			if data['title'] == "findColorSet":
				color = data['data']
				flask_app.colorFindSet(color[0],color[1],color[2])

		print(data)
		response = json.dumps(response)
		await websocket.send(response)


async def main_logic(websocket, path):
	await check_permit(websocket)
	await recv_msg(websocket)


if __name__ == '__main__':
	global flask_app

	wifi_check()
	flask_app = app.webapp()
	flask_app.startthread()

	bs_threading=threading.Thread(target=batteryStatus)   
	bs_threading.setDaemon(True)                     
	bs_threading.start()
	while  1:
		try:				  #Start server,waiting for client
			start_server = websockets.serve(main_logic, '0.0.0.0', 8888)
			asyncio.get_event_loop().run_until_complete(start_server)
			print('waiting for connection...')
			# print('...connected from :', addr)
			break
		except Exception as e:
			print(e)

	try:
		asyncio.get_event_loop().run_forever()
	except Exception as e:
		print(e)
