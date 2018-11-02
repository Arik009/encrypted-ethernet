# sample taken from StackOverflow:
# https://stackoverflow.com/questions/40017293/check-fcs-ethernet-frame-crc-32-online-tools

sample_frame_str = '000AE6F005A3001234567890080045000030B3FE0000801172BA0A0000030A00000204000400001C894D000102030405060708090A0B0C0D0E0F10111213'
sample_frame = bytes.fromhex(sample_frame_str)

NUM_ELEMENTS = 4096;
arr = list(range(NUM_ELEMENTS))

print('packet_synth: sample_frame offset = 0')
print('packet_synth: sample_frame len = %d' % len(sample_frame))

for i in range(len(sample_frame)):
	arr[i] = sample_frame[i]

with open('packet_synth.coe', 'w') as f:
	f.write('memory_initialization_radix=16;\n')
	f.write('memory_initialization_vector=\n')
	for i in range(NUM_ELEMENTS):
		f.write('%02x%c\n' % (arr[i] % 2**8,
			',' if i != NUM_ELEMENTS - 1 else ';'))
