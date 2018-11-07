# sample taken from StackOverflow:
# https://stackoverflow.com/questions/40017293/check-fcs-ethernet-frame-crc-32-online-tools

sample_frame_str = 'FFFFFFFFFFFF001234567890080045000030B3FE0000801172BA0A0000030A00000204000400001C894D000102030405060708090A0B0C0D0E0F10111213'
sample_frame = bytes.fromhex(sample_frame_str)

def reflect(x):
	res = 0
	for i in range(32):
		res = (res << 1) | ((x >> i) & 1)
	return res

def reflect_bytes(x):
	res = 0
	for i in range(4):
		res = (res << 8) | ((x >> (i*8)) & 0xff)
	return res

poly = reflect(0x04c11db7)
init = reflect(0xffffffff)
expected = 0xf9065ed2
mask = 0xffffffff

curr = init
for i in range(len(sample_frame) * 8):
	curr_bit = (sample_frame[i//8] >> (i%8)) & 1
	curr ^= curr_bit
	multiple = poly if (curr & 1) == 1 else 0
	curr = ((curr >> 1) ^ multiple) & mask

print('poly: %08x' % poly)
print('out: %08x' % reflect_bytes((~curr) & mask))
print('expected: %08x' % expected)
