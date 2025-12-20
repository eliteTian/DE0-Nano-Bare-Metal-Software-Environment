from scapy.all import sendp, Ether
import argparse
import sys

ETHERTYPE_TEST = 0x88B5
DEFAULT_IFACE  = "enp6s0"

BOARD_MAC = "00:07:ed:42:9a:48"
HOST_MAC  = "02:11:22:33:44:56"


MIXED_PATTERN = [
     0, 24, 20, 0, -4, 19, 43, 39, 18, 14,
     37, 60, 55, 34, 29, 51, 73, 67, 45, 39,
     60, 81, 74, 51, 43, 63, 83, 75, 50, 41,
     60, 79, 69, 43, 33, 51, 69, 58, 31, 20,
     37, 54, 42, 15, 3, 19, 36, 24, -4, -16,
     0, 16, 4, -24, -36, -19, -3, -15, -42, -54,
    -37, -20, -31, -58, -69, -51, -33, -43, -69, -79,
    -60, -41, -50, -75, -83, -63, -43, -51, -74, -81,
    -60, -39, -45, -67, -73, -51, -29, -34, -55, -60,
    -37, -14, -18, -39, -43, -19, 4, 0, -20, -24
]

ALL_ONES_PATTERN = [0xFF] * 64



def build_payload(byte):
    return bytes((byte & 0xFF) for _ in range(64))

def send_frame(iface, payload):
    frame = Ether(
        dst=BOARD_MAC,
        src=HOST_MAC,
        type=ETHERTYPE_TEST
    ) / payload

    sendp(frame, iface=iface, verbose=False)


def main():
    parser = argparse.ArgumentParser(
        description="Send raw Ethernet test frames using Scapy"
    )


    parser.add_argument(
        "--iface",
        default=DEFAULT_IFACE,
        help="Network interface to use"
    )

    parser.add_argument(
        "--byte",
        type=lambda x: int(x, 0),
        default=0x00,
        help="User content"
    )

    args = parser.parse_args()

    payload = build_payload(args.byte)
    send_frame(args.iface, payload)


if __name__ == "__main__":
    main()
