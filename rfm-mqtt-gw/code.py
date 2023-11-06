# SPDX-FileCopyrightText: 2018 Tony DiCola for Adafruit Industries
# SPDX-License-Identifier: MIT

# Simple example to send a message and then wait indefinitely for messages
# to be received.  This uses the default RadioHead compatible GFSK_Rb250_Fd250
# modulation and packet format for the radio.
import board
import busio
import digitalio
import wifi
import adafruit_rfm69
import os
import socketpool
import ssl
import adafruit_minimqtt.adafruit_minimqtt as MQTT
import json
import time

print(f"Connecting to {os.getenv('CIRCUITPY_WIFI_SSID')}")
wifi.radio.connect(
    os.getenv("CIRCUITPY_WIFI_SSID"), os.getenv("CIRCUITPY_WIFI_PASSWORD")
)

print(f"Connected to {os.getenv('CIRCUITPY_WIFI_SSID')}!")
print(f"My IP address: {wifi.radio.ipv4_address}")

# load saved nodes from nodes.txt

class NodeMsg():
    def __init__(self, msg, client, rfm):
        self.id = str(msg, "ascii").split(';')[0]  # node id
        self.name = str(msg, "ascii").split(';')[1]  # 0 - presentation, 1 - state
        self.type = str(msg, "ascii").split(';')[2]  # payload
        self.device = str(msg, "ascii").split(';')[3]
        self.payload = str(msg, "ascii").split(';')[4]
        self.client = client
        self.topic_base = "homeassistant/"+self.device+"/"+self.name
        self.topic_state = self.topic_base+"/state"
        self.topic_cmd = self.topic_base+"/set"
        self.rfm = rfm
        self.msg = msg
        self.handle = self.gen_handler()

    def process(self):
        if self.type == "0":
            # create a presentation dict
            jmsg = {
                    'name': self.name,
                    'device_class': self.device,
                    'state_topic': self.topic_state,
                    'command_topic': self.topic_cmd,
                    'unique_id': self.id,
                    'payload_on': "ON",
                    'device': {'identifiers': [self.id], 'name': self.id}
                    }
            # return topic_base+"/config", json.dumps(jmsg)
            self.client.publish(self.topic_base+"/config", json.dumps(jmsg))
            print(self.topic_cmd)
            self.client.subscribe(self.topic_cmd)
            self.client.on_message = self.handle
        elif self.type == "1":
            jmsg = self.payload
            self.client.publish(self.topic_state, jmsg)
            # self.client.publish(topic_base, jmsg)
        # elif self.type == "2":
        #     print("got ping from: ", self.id)
        #     self.rfm.send(bytes("pong", "ascii"), destination=int(self.id))

    def gen_handler(self):
        def handle_message(client, topic, message):
            print("Got message: "+message)
            rfm69.send(bytes(message, "ascii"), destination=int(self.id))
        return handle_message


gw_topic_base = "homeassistant/binary_sensor/rfgw"

gw_presentation_dict = {
    'name': "RFGw",
    'device_class': "running",
    'state_topic': gw_topic_base + "/state",
    'command_topic': gw_topic_base + "/set",
    'unique_id': "rfgw01",
    'device': {'identifiers': ["rfgw01"], 'name': "rfgw01"},
}

gw_payload = json.dumps(gw_presentation_dict)
print(gw_payload)


pool = socketpool.SocketPool(wifi.radio)
ssl_context = ssl.create_default_context()

mqtt_client = MQTT.MQTT(
    broker="192.168.1.8",
    port=1883,
    socket_pool=pool,
    username="rfgw",
    password="StrNgRF",
    ssl_context=ssl_context,
)

mqtt_client.connect()
mqtt_client.publish(gw_topic_base+"/config", gw_payload)
mqtt_client.publish(gw_topic_base+"/state", "ON")
mqtt_client.subscribe(gw_topic_base+"/set")
# Define radio parameters.
RADIO_FREQ_MHZ = 433.0  # Frequency of the radio in Mhz. Must match your
# module! Can be a value like 915.0, 433.0, etc.

# Define pins connected to the chip, use these if wiring up the breakout according to the guide:
CS = digitalio.DigitalInOut(board.GP17)
RESET = digitalio.DigitalInOut(board.GP15)
# Or uncomment and instead use these if using a Feather M0 RFM69 board
# and the appropriate CircuitPython build:
# CS = digitalio.DigitalInOut(board.RFM69_CS)
# RESET = digitalio.digitalioitalInOut(board.RFM69_RST)
PING_TIME = -1
# Define the onboard LED
LED = digitalio.DigitalInOut(board.LED)
LED.direction = digitalio.Direction.OUTPUT

# Initialize SPI bus.
spi = busio.SPI(board.GP18, MOSI=board.GP19, MISO=board.GP16)

# Initialze RFM radio
rfm69 = adafruit_rfm69.RFM69(spi, CS, RESET, RADIO_FREQ_MHZ)
rfm69.node = 0

# Optionally set an encryption key (16 byte AES key). MUST match both
# on the transmitter and receiver (or be set to None to disable/the default).
rfm69.encryption_key = (
    b"\x01\x02\x03\x04\x05\x06\x07\x08\x01\x02\x03\x04\x05\x06\x07\x08"
)

# Print out some chip state:
print("Temperature: {0}C".format(rfm69.temperature))
print("Frequency: {0}mhz".format(rfm69.frequency_mhz))
print("Bit rate: {0}kbit/s".format(rfm69.bitrate / 1000))
print("Frequency deviation: {0}hz".format(rfm69.frequency_deviation))

# Send a packet.  Note you can only send a packet up to 60 bytes in length.
# This is a limitation of the radio packet size, so if you need to send larger
# amounts of data you will need to break it into smaller send calls.  Each send
# call will wait for the previous one to finish before continuing.
# rfm69.send(bytes("Hello world!\r\n", "utf-8"))
# print("Sent hello world message!")

# Wait to receive packets.  Note that this library can't receive data at a fast
# rate, in fact it can only receive and process one 60 byte packet at a time.
# This means you should only use this for low bandwidth scenarios, like sending
# and receiving a single message at a time.
print("Waiting for packets...")
while True:
    # send a ping too all
    now = time.monotonic()
    if now >= PING_TIME+30:
        rfm69.send_with_ack("ping")
        PING_TIME = now
    mqtt_client.loop()
    packet = rfm69.receive()
    # Optionally change the receive timeout from its default of 0.5 seconds:
    # packet = rfm69.receive(timeout=5.0)
    # If no packet was received during the timeout then None is returned.
    if packet is None:
        # Packet has not been received
        LED.value = False
        print("Received nothing! Listening again...")
    else:
        # Received a packet!
        LED.value = True
        mesg = NodeMsg(packet, mqtt_client, rfm69)
        mesg.process()
        # print("Received (raw bytes): {0}".format(packet))
        # print("Got a presentation packet from id: "+msg.id)
        # mqtt_client.publish(topic, payload)
        # print("sent mqtt discovery")
        # And decode to ASCII text and print it too.  Note that you always
        # receive raw bytes and need to convert to a text format like ASCII
        # if you intend to do string processing on your data.  Make sure the
        # sending side is sending ASCII data before you try to decode!
        packet_text = str(packet, "ascii")
        print("Received (ASCII): {0}".format(packet_text))
