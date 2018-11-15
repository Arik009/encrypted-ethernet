from PIL import Image

import serial.tools.list_ports
import sys
import time
from time import sleep

def get_usb_port():
	usb_port = list(serial.tools.list_ports.grep('USB-Serial Controller'))
	if len(usb_port) == 1:
		print('Automatically found USB-Serial Controller: {}'.format(usb_port[0].description))
		return usb_port[0].device
	else:
		ports = list(serial.tools.list_ports.comports())
		port_dict = {i:[ports[i],ports[i].vid] for i in range(len(ports))}
		usb_id = None
		for p in port_dict:
			print('{}:   {} (Vendor ID: {})'.format(p,port_dict[p][0],port_dict[p][1]))
			if port_dict[p][1]==1027:
				usb_id = p
		if usb_id == None:
			return None
		else:
			print('USB-Serial Controller: Device {}'.format(p))
			return port_dict[usb_id][0].device

serial_port = get_usb_port()
if serial_port is None:
	raise Exception('USB-Serial Controller Not Found')

IMAGE_WIDTH = 32
IMAGE_HEIGHT = 32

def image_to_bytestream(fin_name):
	res = []
	im = Image.open(fin_name)
	im = im.resize((IMAGE_WIDTH, IMAGE_HEIGHT))
	for i in range(IMAGE_HEIGHT):
		for j in range(IMAGE_WIDTH//2):
			r1, g1, b1 = im.getpixel((j*2, i))
			r2, g2, b2 = im.getpixel((j*2+1, i))
			r1 >>= 4
			g1 >>= 4
			b1 >>= 4
			r2 >>= 4
			g2 >>= 4
			b2 >>= 4
			res += [(r1 << 4) | g1, (b1 << 4) | r2, (g2 << 4) | b2]
	return bytes(res)

def send_image(ser, fin_name):
	bytestream = image_to_bytestream(fin_name)
	num_written = ser.write(bytestream)
	ser.flush()
	print("%d bytes written" % num_written)

with serial.Serial(port = serial_port, 
		baudrate=12000000,
		parity=serial.PARITY_NONE, 
		stopbits=serial.STOPBITS_ONE, 
		bytesize=serial.EIGHTBITS,
		timeout=0) as ser:

	print(ser)
	print('Serial Connected!')

	if ser.isOpen():
		print(ser.name + ' is open...')

	send_image(ser, 'images/nyan.jpg')
