import sys
import socket
import random
sys.path.append('../lib/')
import eth
import ip
import dhcp
import arp
import time

ETH_P_ALL = 3
INTERFACE = 'enp2s0'
BUF_SIZE = 4096

with open('test_server.txt') as f:
	server_data = f.read().strip().split(':')
	server_port = int(server_data[1])
	server_ip_str = server_data[0].split('.')
	server_ip = bytes([
		int(server_ip_str[0]),
		int(server_ip_str[1]),
		int(server_ip_str[2]),
		int(server_ip_str[3])
	])

sock = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(ETH_P_ALL))
sock.bind((INTERFACE, ETH_P_ALL))
# sock.setblocking(0)

dhcp_xid = random.getrandbits(32)

eth.sendeth(eth.gen_eth(eth.MAC_BROADCAST, eth.MAC_SEND, eth.ETHERTYPE_IP,
	dhcp.gen_dhcp_discover(dhcp_xid, 0)
))

def filter_dhcp_reply(packet, dhcp_xid):
	if eth.get_ethertype(packet) != eth.ETHERTYPE_IP:
		return None
	packet = packet[eth.HEADER_LEN:]
	if (ip.ip_get_version(packet) != ip.IP_VERSION_4 or
		ip.ip_get_prot(packet) != ip.IP_PROT_UDP or
		ip.ip_get_dst_addr(packet) != ip.IPADDR_BROADCAST):
		return None
	packet = packet[ip.IP_HEADER_LEN_DEFAULT*4:]
	if (ip.udp_get_dst_port(packet) != dhcp.PORT_CLIENT):
		return None
	packet = packet[ip.UDP_HEADER_LEN:]
	if (dhcp.get_op(packet) != dhcp.OP_REPLY or
		dhcp.get_xid(packet) != dhcp_xid):
		return None
	return packet

while True:
	packet = sock.recv(BUF_SIZE)
	packet = filter_dhcp_reply(packet, dhcp_xid)
	if packet is None:
		continue
	client_ip = dhcp.get_ip(packet)
	dhcp_opts = dhcp.get_opts(packet)
	upstream_ip = dhcp_opts[dhcp.OPT_ROUTER]
	netmask = dhcp_opts[dhcp.OPT_MASK]
	dns_ip = dhcp_opts[dhcp.OPT_DNS]
	break

while True:
	eth.sendeth(eth.gen_eth(eth.MAC_BROADCAST, eth.MAC_SEND,
		eth.ETHERTYPE_IP,
		dhcp.gen_dhcp_request(dhcp_xid, 0, client_ip)
	))
	start_time = time.time()
	success = False

	while True:
		if time.time() - start_time > 5.0:
			print('No reply found, sending another DHCP request')
			break
		packet = sock.recv(BUF_SIZE)
		packet = filter_dhcp_reply(packet, dhcp_xid)
		if packet is None:
			continue
		success = True
		break
	if success:
		break

eth.sendeth(eth.gen_eth(eth.MAC_BROADCAST, eth.MAC_SEND, eth.ETHERTYPE_ARP,
	arp.gen_arp(client_ip, upstream_ip)
))

while True:
	packet = sock.recv(BUF_SIZE)
	if eth.get_ethertype(packet) != eth.ETHERTYPE_ARP:
		continue
	packet = packet[eth.HEADER_LEN:]
	if (arp.get_op(packet) != arp.OP_REPLY or
		arp.get_target_addr(packet) != client_ip):
		continue
	upstream_mac = arp.get_sender_mac(packet)
	break

time.sleep(1)

eth.sendeth(eth.gen_eth(upstream_mac,
	eth.MAC_SEND,
	eth.ETHERTYPE_IP,
	ip.gen_ip_udp(
		client_ip,
		server_ip,
		58099, server_port,
		b'hello')
))
