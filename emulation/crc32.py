# sample taken from StackOverflow:
# https://stackoverflow.com/questions/40017293/check-fcs-ethernet-frame-crc-32-online-tools

sample_frame_str = '000AE6F005A3001234567890080045000030B3FE0000801172BA0A0000030A00000204000400001C894D000102030405060708090A0B0C0D0E0F10111213'
sample_frame = bytes.fromhex(sample_frame_str)

def reflect(x):
	res = 0
	for i in range(32):
		res = (res << 1) | ((x >> i) & 1)
	return res

poly = reflect(0x04c11db7)
init = reflect(0xffffffff)
expected = 0xb36bd57a
mask = 0xffffffff

curr = init
for i in range(len(sample_frame) * 8):
	curr_bit = (sample_frame[i//8] >> (i%8)) & 1
	curr ^= curr_bit
	multiple = poly if (curr & 1) == 1 else 0
	curr = ((curr >> 1) ^ multiple) & mask

print('poly: 0x%08x' % poly)
print('out: %08x' % ((~curr) & mask))
print('expected: %08x' % expected)
