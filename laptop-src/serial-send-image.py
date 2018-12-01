import sys
sys.path.append('../lib/')
import image_bytes
import fpga_serial

IMAGE_WIDTH = 128
IMAGE_HEIGHT = 128

def send_image(ser):
	fin_name = 'images/nyan.jpg'
	bytestream = image_bytes.image_to_bytestream(
		fin_name, IMAGE_WIDTH, IMAGE_HEIGHT)
	num_written = ser.write(bytestream)
	ser.flush()
	print("%d bytes written" % num_written)

fpga_serial.do_serial(send_image)
