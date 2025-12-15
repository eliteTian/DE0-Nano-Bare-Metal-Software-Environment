from scapy.all import sendp, Ether

payload = bytes([i & 0xff for i in range(64)])   # example payload

frame = Ether(
    dst="00:07:ed:42:9a:48",   # board MAC
    src="02:11:22:33:44:56",   # host MAC
    type=0x88b5
) / payload

sendp(frame, iface="enp6s0")  # replace interface name
