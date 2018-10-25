from socket import *
def sendeth(src, dst, eth_type, payload, interface = "enp2s0"):
	assert(len(src) == len(dst) == 6) # 48-bit ethernet addresses
	assert(len(eth_type) == 2) # 16-bit ethernet type
	s = socket(AF_PACKET, SOCK_RAW)
	s.bind((interface, 0))
	return s.send(src + dst + eth_type + payload)

if __name__ == "__main__":
	print("Sent %d-byte Ethernet packet on enp2s0" %
	sendeth(b"\xFE\xED\xFA\xCE\xBE\xEF",
					b"\xFE\xED\xFA\xCE\xBE\xEF",
					b"\x7A\x05",
					b"hello"))
