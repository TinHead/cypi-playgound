from board import GP16, GP17, GP18, GP19, GP21, GP22, LED
from busio import SPI
from digitalio import DigitalInOut, Direction
from adafruit_rfm69 import RFM69
from os import getenv
from time import monotonic
from asyncio import gather, sleep, sleep_ms, run, create_task

MY_NAME = getenv("NODE_NAME")
MY_ID = int(getenv("NODE_ID"))
MY_STATE = "OFF"
RADIO_FREQ_MHZ = 433.0

# Define pins connected to the chip, use these if wiring up the breakout according to the guide:
CS = DigitalInOut(GP17)
RESET = DigitalInOut(GP21)
RELAY = DigitalInOut(GP22)
RELAY.direction = Direction.OUTPUT
RELAY.value = True
GW_UP = True

# Define the onboard LED
LED = DigitalInOut(LED)
LED.direction = Direction.OUTPUT


class gwState():
    def __init__(self):
        self.state = True
        self.ping_time = ""


class rfmMsg():
    def __init__(self, i, n, t, nt, p):
        self.id = i
        self.name = n
        self.msg_type = t
        self.node_type = nt
        self.payload = p

    def gen_msg(self):
        return bytes(self.id + ";"+self.name+";"+self.msg_type+";"+self.node_type+";"+self.payload, "ascii")


def init_rfm():
    # init SPI
    spi = SPI(GP18, MOSI=GP19, MISO=GP16)
    rfm69 = RFM69(spi, CS, RESET, RADIO_FREQ_MHZ)
    rfm69.node = MY_ID
    rfm69.destination = 0  # gw is 0 
    rfm69.encryption_key = (
        b"\x01\x02\x03\x04\x05\x06\x07\x08\x01\x02\x03\x04\x05\x06\x07\x08"
    )
    return rfm69


async def present_me(rfm, msg):
    # send a presentation message every 60 seconds in case gw dies
    while True:
        rfm.send(msg)
        await sleep(60)


async def recv_gw(rfm, state, ping, gw_state):
    while True:
        pkg = rfm.receive()
        if pkg is None:
            print("Nothing received ... moving on...")
        else:
            if pkg == "ON":
                print("Got ON message")
                state.payload = "ON"
                LED.value = True
                RELAY.value = False
                print("Sending state with ack")
                rfm.send_with_ack(state.gen_msg())
            elif pkg == "OFF":
                print("Got OFF message")
                state.payload = "OFF"
                LED.value = False
                RELAY.value = True
                print("Sending state with ack")
                rfm.send_with_ack(state.gen_msg())
            elif pkg == "ping":
                gw_state.state = True
                gw_state.ping_time = monotonic()
                rfm.send(ping)
        await sleep_ms(1)


async def check_uptime(gw_state, node_state):
    while True:
        now = monotonic()
        if now >= gw_state.ping_time+120:
            print("GW connection lost .. turning everything off")
            gw_state.state = False
            node_state.payload = "OFF"
            RELAY.value = True
            LED.value = False
        await sleep(60)


async def main():
    present = rfmMsg(str(MY_ID), MY_NAME, "0", "switch", "OFF")
    state = rfmMsg(str(MY_ID), MY_NAME, "1", "switch", "OFF")
    ping = rfmMsg(str(MY_ID), MY_NAME, "2", "pong", "OFF")
    rfm = init_rfm()
    gw_state = gwState()
    gw_state.ping_time = 0
    recv_task = recv_gw(rfm, state, ping.gen_msg(), gw_state)
    present_task = present_me(rfm, present.gen_msg())
    uptime_task = check_uptime(gw_state, state)
    await gather(present_task, recv_task, uptime_task)


run(main())

# # send an initial presentation
# presentation = str(MY_ID)+";"+MY_NAME+";0;switch;OFF"
# state = str(MY_ID)+";"+MY_NAME+";1;switch;"+MY_STATE
# ping = str(MY_ID)+";"+MY_NAME+";2;switch;"+MY_STATE

# rfm69.send(bytes(presentation, "ascii"))
# print("Sent presentation message!")

# rfm69.send(bytes(state, "ascii"))
# print("Waiting for packets...")
# while True:
#     now = time.monotonic()
#     if now >= PING_TIME+120:
#         print("lost gateway connection! turning off")
#         RELAY.value = True
#         MY_STATE = "OFF"
#         GW_UP = False
#     if now >= PRES_TIME+20:
#         rfm69.send(bytes(presentation, "ascii"))
#         rfm69.send(bytes(str(MY_ID)+";"+MY_NAME+";1;switch;"+MY_STATE, "ascii"))
#         PRES_TIME = time.monotonic()
#     packet = rfm69.receive()
#     if packet is None:
#         # Packet has not been received
#         LED.value = False
#         print("Received nothing! Listening again...")
#     else:
#         # Received a packet!
#         LED.value = True
#         if packet == "ON":
#             RELAY.value = False
#             print("Sending ON")
#             MY_STATE = "ON"
#             rfm69.send_with_ack(str(MY_ID)+";"+MY_NAME+";1;switch;"+MY_STATE)
#         elif packet == "OFF":
#             RELAY.value = True
#             MY_STATE = "OFF"
#             print("Sending OFF")
#             rfm69.send_with_ack(str(MY_ID)+";"+MY_NAME+";1;switch;"+MY_STATE)
#         elif packet == "ping":
#             print("got a ping from GW")
#             rfm69.send(str(MY_ID)+";"+MY_NAME+";2;pong;OFF")
#             PING_TIME = now
#             if not GW_UP:
#                 print("resending presentation message!")
#                 rfm69.send(bytes(presentation, "ascii"))
#                 rfm69.send(bytes(str(MY_ID)+";"+MY_NAME+";1;switch;"+MY_STATE, "ascii"))
#                 GW_UP = True
#         # elif packet == "present":
#         #     print("Got presentation request!")
#         #     rfm69.send(bytes(presentation, "ascii"))
#         #     rfm69.send(bytes(state, "ascii"))
#         packet_text = str(packet, "ascii")
#         print("Received (ASCII): {0}".format(packet_text))
