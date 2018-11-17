import os

# sample taken from StackOverflow:
# https://stackoverflow.com/questions/40017293/check-fcs-ethernet-frame-crc-32-online-tools

mac_send = bytes.fromhex('DEADBEEFCAFE')
mac_recv = bytes.fromhex('C0FFEEDAD101')

sample_payload_str = '080045000030B3FE0000801172BA0A0000030A00000204000400001C894D000102030405060708090A0B0C0D0E0F10111213'
sample_payload = bytes.fromhex(sample_payload_str)

sample_frame_str = 'FFFFFFFFFFFF001234567890080045000030B3FE0000801172BA0A0000030A00000204000400001C894D000102030405060708090A0B0C0D0E0F10111213'
# add preamble manually for testing
sample_frame_str = '555555555555D5' + sample_frame_str
# add crc manually for testing
sample_frame_str += 'F9065ED2'
sample_frame = bytes.fromhex(sample_frame_str)

class Memory:
	def __init__(self):
		self.curr_bytes = []
		self.curr_index = 0

	def append(self, val):
		off = self.curr_index
		self.curr_bytes += val
		self.curr_index += len(val)
		return off

mem = Memory()
mac_send_off = mem.append(mac_send)
mac_recv_off = mem.append(mac_recv)
sample_frame_off = mem.append(sample_frame)
sample_payload_off = mem.append(sample_payload)

NUM_ELEMENTS = 4096;
arr = list(range(NUM_ELEMENTS))

def write_localparam(f, key, val):
	f.write('localparam %s = %d;\n' % (key, val))

rom_layout_filename = os.path.join(
	os.path.dirname(__file__), '../hdl/inc/packet_synth_rom_layout.vh')
with open(rom_layout_filename, 'w') as f:
	f.write('// This is a generated file. DO NOT edit this directly.\n\n')
	f.write('// sender and receiver MAC addresses\n')
	write_localparam(f, 'MAC_SEND_OFF', mac_send_off)
	write_localparam(f, 'MAC_RECV_OFF', mac_recv_off)
	f.write('\n')
	write_localparam(f, 'SAMPLE_PAYLOAD_OFF', sample_payload_off)
	write_localparam(f, 'SAMPLE_PAYLOAD_LEN', len(sample_payload))
	write_localparam(f, 'SAMPLE_FRAME_OFF', sample_frame_off)
	write_localparam(f, 'SAMPLE_FRAME_LEN', len(sample_frame))

for i in range(len(mem.curr_bytes)):
	arr[i] = mem.curr_bytes[i]

with open('packet_synth.coe', 'w') as f:
	f.write('memory_initialization_radix=16;\n')
	f.write('memory_initialization_vector=\n')
	for i in range(NUM_ELEMENTS):
		f.write('%02x%c\n' % (arr[i] % 2**8,
			',' if i != NUM_ELEMENTS - 1 else ';'))
