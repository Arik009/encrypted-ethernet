import sys
sys.path.append('../lib/')
import eth
import image_bytes

fin_name = 'images/nyan.jpg'
IMAGE_WIDTH = 128
IMAGE_HEIGHT = 128
eth.sendeth(eth.gen_eth_fgp(512,
	image_bytes.image_to_colors(
		fin_name, IMAGE_WIDTH, IMAGE_HEIGHT)[:512]))
