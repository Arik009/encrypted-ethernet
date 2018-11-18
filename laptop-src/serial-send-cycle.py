import sys
import os
import os.path
from time import sleep
sys.path.append('../lib/')
import eth
import image_bytes
import fpga_serial

STOP_EARLY = True
image_dir = 'images/nyan/'
IMAGE_WIDTH = 32
IMAGE_HEIGHT = 32

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
					eth.gen_eth_fgp(i*512, im[i*512:(i+1)*512]))
				ser.flush()
				print("%d bytes written" % num_written)
				sleep(0.01)
			cnt = cnt + 1

send_cycle(None)

fpga_serial.do_serial(send_cycle)
