import sys
import os
import os.path
from time import sleep
sys.path.append('../lib/')
import eth
import image_bytes
import fpga_serial

STOP_EARLY = False
image_dir = 'images/nyan/'
IMAGE_WIDTH = 128
IMAGE_HEIGHT = 128

def send_cycle(ser):
	# Only cycle a few times for testing
	cnt = 0
	images = sorted(os.listdir(image_dir))
	while True:
		if STOP_EARLY and cnt == 5:
			break
		for fin_name in images:
			im = image_bytes.image_to_colors(
				os.path.join(image_dir, fin_name),
				IMAGE_WIDTH, IMAGE_HEIGHT)
			for i in range(len(im)//512):
				num_written = ser.write(
					eth.gen_eth_fgp_payload(i*512, im[i*512:(i+1)*512]))
				ser.flush()
				print("%d bytes written" % num_written)
				sleep(0.02)
		cnt = cnt + 1

fpga_serial.do_serial(send_cycle)
