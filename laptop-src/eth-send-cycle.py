import sys
from time import sleep
sys.path.append('../lib/')
import eth
import image_bytes

STOP_EARLY = True
fin_name = 'images/nyan.jpg'
IMAGE_WIDTH = 32
IMAGE_HEIGHT = 32
im = image_bytes.image_to_colors(
	fin_name, IMAGE_WIDTH, IMAGE_HEIGHT)
# Only cycle a few times for testing
cnt = 0
while True:
	if STOP_EARLY and cnt == 5:
		break
	for i in range(len(im)//512):
		eth.sendeth(eth.gen_eth_fgp(i*512, im[i*512:(i+1)*512]))
		sleep(0.01)
	cnt = cnt + 1
