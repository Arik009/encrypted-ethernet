import sys
sys.path.append('../lib/')
import image_bytes
import fpga_serial
import eth

fin_name = 'images/nyan.jpg'
IMAGE_WIDTH = 32
IMAGE_HEIGHT = 32

def send_single(ser):
	bytestream = eth.gen_eth_fgp_payload(512,
		image_bytes.image_to_colors(
			fin_name, IMAGE_WIDTH, IMAGE_HEIGHT)[512:2*512])
	num_written = ser.write(bytestream)
	ser.flush()
	print("%d bytes written" % num_written)

fpga_serial.do_serial(send_single)
