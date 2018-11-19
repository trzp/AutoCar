# coding:utf-8
__author__ = 'mr tang'
__date__ = '2014-08-31'

import struct,os
import numpy as np
from math import fabs,ceil

CODE_BASE = 9150 #old value = 7800
 
bydparams = {	'PORT_computer':	9002,				# mcu -> computer; 上位机监听该端口捕获数据
				'PORT_ecu':			8002,				# computer -> mcu; 下位机监听该端口捕获数据
				'IP_computer':		'192.168.0.111',	# 协议上位机IP地址，以便下位机接收数据
				'IP_ecu':			'192.168.0.254',	# 协议下位机IP地址，以便上位机接受数据
				'ratio_l':			2698 / 13.96,
				'ratio_r':			2545 / 13.25,
				'Kp':				150,
				'Ki':				10,
				'Kd':				0,
				'ForwardVellmt':	7,					# m/s 约25km/h
				'BackwardVellmt':	-0.6,				# m/s 约2km/h 
				'gas_ctr_lmt':		800,
				'brake_ctr_lmt':	-643,
				'u_gas':			10,
				'u_brake':			0,
				'throttle_threshold':40, 
				'slope_gas':		0.1,
				'slope_brake':		-35,
				'brake_init':		10000,
			}

def sgn(num):
	if num > 0:		return 1 
	elif num < 0:	return -1
	else:			return 0 

def steeringK(vel):
	vel = abs(vel)
	a = 2/3.
	k = (a-1)*vel/3.+3.5-2*a
	if k < 0.2:k = 0.2
	return k

def steeringlmt(vel):
	vel = abs(vel)
	a = 12
	y = (a-25)*abs(vel)/4.+37.5-0.5*a
	return y

class BydVelCtr():
	# 增量式PID计算油门量和刹车量。
	# 计算的原始量为Uaction，Uaction>0为加速，应当加油，Uaction<0则为减速，应当刹车
	# 分别依据Uaction的正负来换算出实际给的油门量和刹车量
	def __init__(self):
		self.ek1 = 0 
		self.ek2 = 0 
		self.lst_U = 0 

	def update(self,deVel,acVel):	# m/s 
		# 限速
		if deVel > bydparams['ForwardVellmt']:		deVel = bydparams['ForwardVellmt']
		if deVel < bydparams['BackwardVellmt']:		deVel = bydparams['BackwardVellmt']

		# pid计算出油门量
		ek0 = abs(deVel) - acVel
		Uaction = self.lst_U + bydparams['Kp'] * (ek0 - self.ek1) + bydparams['Ki'] * ek0 + bydparams['Kd'] * (ek0 - 2*self.ek1 + self.ek2)

		if Uaction > bydparams['gas_ctr_lmt']:		Uaction = bydparams['gas_ctr_lmt']# 限值
		elif Uaction < bydparams['brake_ctr_lmt']:	Uaction = bydparams['brake_ctr_lmt']

		self.ek2 = self.ek1 
		self.ek1 = ek0
		self.lst_U = Uaction

		# 期望停车
		if deVel == 0 and acVel < 0.7:
			Uaction = bydparams['brake_ctr_lmt']
			self.__init__()		# 初始化，消除累计误差

		# Uaction转换成油门量和刹车量
		if Uaction > bydparams['u_gas']:											#加速阶段
			brake = bydparams['brake_init']
			throttle = bydparams['slope_gas'] * Uaction
		elif Uaction < bydparams['u_brake']:										# 减速段
			brake = bydparams['brake_init'] + bydparams['slope_brake'] * Uaction
			throttle = 0 
		else:
			brake = bydparams['brake_init']
			throttle = 0

		return throttle,brake 

class BydState():
	# 调用类的Parsing函数解析从网络获取的原始数据，数据校验成功，返回1，否则返回0
	# 数据校验成功，将解析的数据依据协议赋予类的成员变量
	def __init__(self):
		self.SteeringAngle = 0			# 方向盘角度，单位：度。左转为正
		self.ForwardVel = 0				# 速度，km/h
		self.ShiftStr = '=PRNDMS+-' + '='*247	# 档位列表,数值与档位对应：1-P;2-R;3-N;4-D;5-M;6-S;7-+;8--
		self.Shift = '='				# 当前档位。'='无效档位
		self.EStop = 0					# 紧急制动状态,1-制动，0-未制动
		self.LeftLamp = 0				# 左转向灯状态，1-亮，0-灭
		self.RightLamp = 0				# 右转向灯状态，1-亮，0-灭
		self.LongituteEnabled = 0		# 纵向控制使能状态，1-使能，0-未使能
		self.LateralEnabled = 0			# 横向控制使能状态，1-使能，0-未使能
		self.BrakeEnabled = 0			# 制动踏板信号，1-踩下，0-未踩下

	def __PreProcessing(self,BUF):		# 数据解码、校验
		ascDATA = []					# ascii码列表
		ascDATA = [ord(ch) for ch in BUF]
		check = 0
		if BUF[0:4] == '$BYD' and BUF[46] == '*':
			for item in ascDATA[0:-2]:	check ^= item
			Hcheck = (check >> 4) & 0x0f
			Lcheck = (check & 0x0f)
			if Hcheck < 10:		Hcheck += 48
			else:				Hcheck += 55
			if Lcheck < 10:		Lcheck += 48
			else:				Lcheck += 55
			if Hcheck == ascDATA[47] and Lcheck == ascDATA[48]:	return 1
			else:	return 0
		else:	return 0

	def __Code2Angle(self,code,ratio_l = bydparams['ratio_l'],ratio_r = bydparams['ratio_r']):
		Angle = code - CODE_BASE
		if Angle > 0:	Angle = Angle / ratio_l
		else:			Angle = Angle / ratio_r
		return Angle

	def Parsing(self,BUF):
		if self.__PreProcessing(BUF):
			self.SteeringAngle = self.__Code2Angle(int(BUF[5:10]))#因更换转向机，参数进行调整
			self.ForwardVel = int(BUF[11:15])/100.	# m/s
			self.Shift = self.ShiftStr[int(BUF[26])]
			self.EStop = int(BUF[30])
			self.LeftLamp = int(BUF[32])
			self.RightLamp = int(BUF[34])
			self.LongituteEnabled = int(BUF[36])
			self.LateralEnabled = int(BUF[38])
			self.BrakeEnabled = int(BUF[40])
			return 1
		return 0
# the end

class BydCmd():
	# 定义了一系列函数设置对车辆的期望操作，完成了缓冲命令的书写
	# 在将命令下发至车辆之前必须调用update函数，命令才能生效

	def __init__(self):
		self.buffer = np.zeros(13,np.uint8)						# 创建一段单字节整数序列
		self.buffer[0] = self.buffer[1] = self.buffer[2] = 0xff	# 包头
		self.setLateral(0)
		self.setLongitute(0)
		self.setLeftLamp(0)
		self.setRightLamp(0)
		self.setShift('N')
		self.setThrottle(0)
		self.setSteeringAngle(0)

	def setLeftLamp(self,flag):
		if flag:	self.buffer[3] |= 0x40
		else:		self.buffer[3] &= 0xbf

	def setRightLamp(self,flag):
		if flag:	self.buffer[3] |= 0x20
		else:		self.buffer[3] &= 0xdf

	def setLongitute(self,flag):
		if flag:	self.buffer[3] |= 0x10
		else:		self.buffer[3] &= 0xEF

	def setLateral(self,flag):
		if flag:	self.buffer[3] |= 0x08
		else:		self.buffer[3] &= 0xf7

	def setShift(self,flag):	# N/D/R
		if flag == 'd' or flag == 'D':		# 'D'
			self.buffer[3] |= 0x01
			self.buffer[3] &= 0xfd
		elif flag == 'r' or flag == 'R':	# 'R'
			self.buffer[3] |= 0x02
			self.buffer[3] &= 0xfe
		else:								# 'N'
			self.buffer[3] &= 0xfc

	def __Angle2Code(self,angle,ratio_l = bydparams['ratio_l'],ratio_r = bydparams['ratio_r']):	# -25 < angle < 25     left turn > 0
		if angle > 35:	angle = 35
		if angle < -28:	angle = -28
		if angle > 0:
			code = int(ceil(ratio_l * angle +CODE_BASE))
		else:
			code = int(ceil(ratio_r * angle +CODE_BASE))
		codeH = (code >> 8) & 0xff
		codeL = (code & 0xff)
		return codeH,codeL

	def setSteeringAngle(self,angle):
		angle = angle#因更换转向机，参数进行调整
		if angle > 0:	self.buffer[3] |= 0x04
		else:			self.buffer[3] &= 0xFB
		self.buffer[4],self.buffer[5] = self.__Angle2Code(angle)

	def setThrottle(self,gas):
		gas = int(gas)
		gasH = (gas >> 8) & 0xff 
		gasL = gas & 0xff 
		self.buffer[6] = gasH
		self.buffer[7] = gasL 

	def setBrake(self,num):
		num = int(num)
		braH = (num >> 8) & 0xff 
		braL = num & 0xff 
		self.buffer[8] = braH
		self.buffer[9] = braL 
	
	def setEstop(self,es):
		pass

	def update(self):
		self.buffer[12] = 0
		for item in self.buffer[1:12]:	self.buffer[12] ^= item	# 异或校验
		self.buffer[12] &= 0x7F									# 校验位生成
# the end 

class BCIcmd():
	# 定义的bci2000平台与控制电脑之间的通讯协议。
	# BCI2000平台依赖该类实现命令的生成（下发前需要调用update方法）
	# 控制计算机依赖该类实现对从网络获取的命令的解析
	def __init__(self):
		self.buffer = np.zeros(9,np.float32)
		self.Mode = 0			# 0-无操作，1-纵向操作，2-转向操作，3-纵横操作
		self.deVel = 0			# 期望速度，正为向前，负为向后,m/s
		self.Turn = 0			# 增量式控制，左转为正。数值为每个周期增加的角度。0则是保持当前角度。
		self.steeringReturn = 0	# 方向盘回正。0-不操作，非零-回正，按照给定数值为增量控制回正方向盘。
		self.Brake = 0			# 刹车
		self.Stop = 0 			# 驻车

	def update(self):			# 作为发送端数据的更新和校验位写入
		self.buffer[0] = 255.0 
		self.buffer[1] = 254.0 
		self.buffer[2] = self.Mode
		self.buffer[3] = self.deVel
		self.buffer[4] = self.Turn
		self.buffer[5] = self.steeringReturn
		self.buffer[6] = self.Brake
		self.buffer[7] = self.Stop
		self.buffer[8] = 0
		for item in self.buffer[0:8]:	self.buffer[8] += item	# 校验

	def Parsing(self,BUF):		# 作为接受端缓冲数据的解码
		temp = []
		for i in range(0,len(BUF),4):
			s,= struct.unpack('f',BUF[i:i+4])
			temp.append(s)
		check = 0 
		for item in temp[0:8]:	check += item
		# 校验包头和校验位
		if fabs(temp[0] - 255) < 0.01  and fabs(temp[1] - 254.0) < 0.01 and fabs(check - temp[8]) < 0.01:
			self.Mode = int(temp[2])
			self.deVel = temp[3]
			self.Turn = temp[4]
			self.steeringReturn = temp[5]
			self.Brake = int(temp[6])
			self.Stop = int(temp[7])
			return 1
		else:	return 0 
# the end 

if __name__ == '__main__':
	s = MotorCtr()
	print s.BDang(5,1)