from board import GP17, GP15, GP18, GP19, GP16, LED
from digitalio import Direction, DigitalInOut
from busio import SPI
from wifi import radio
from adafruit_rfm69 import RFM69
from os import getenv
from socketpool import SocketPool
from ssl import create_default_context
from adafruit_minimqtt import adafruit_minimqtt as MQTT
from json import dumps
from asyncio import gather, sleep, sleep_ms, run, create_task

# constants

RADIO_FREQ_MHZ = 433.0
PING_TIME = -1

# init pins
CS = DigitalInOut(GP17)
RESET = DigitalInOut(GP15)
MY_LED = DigitalInOut(LED)
MY_LED.direction = Direction.OUTPUT
NODES = []


class NodeMsg:
    def __init__(self, msg, client, rfm):
        fields = str(msg, "ascii").split(';')
        self.id = fields[0]  # node id
        self.name = fields[1]  # 0 - presentation, 1 - state
        self.type = fields[2]  # payload
        self.device = fields[3]
        self.payload = fields[4]
        self.client = client
        self.topic_base = "homeassistant/"+self.device+"/"+self.name
        self.topic_state = self.topic_base+"/state"
        self.topic_cmd = self.topic_base+"/set"
        self.rfm = rfm
        self.msg = msg
        self.handle = self.gen_handler()
        print("processing message ", self.msg)

    def process(self):
        if self.type == "0":
            # create a presentation dict
            if self.id not in NODES:
                jmsg = {
                        'name': self.name,
                        'device_class': self.device,
                        'state_topic': self.topic_state,
                        'command_topic': self.topic_cmd,
                        'unique_id': self.name,
                        'payload_on': "ON",
                        'device': {'identifiers': [self.id], 'name': self.name}
                        }
                try:
                    # setup the node in mqtt
                    self.client.publish(self.topic_base+"/config", dumps(jmsg))
                    # set the initial state
                    self.client.publish(self.topic_state, self.payload)
                    self.client.subscribe(self.topic_cmd)
                    self.client.add_topic_callback(self.topic_cmd, self.handle)
                    NODES.append(self.id)
                except MQTT.MMQTTException as err:
                    print("MQTT error:", err)
            else:
                print("Node ", self.id, " already known.")
                # self.client.add_topic_callback(self.topic_cmd, self.handle)

        elif self.type == "1":
            jmsg = self.payload
            try:
                self.client.publish(self.topic_state, jmsg)
            except MQTT.MMQTTException as err:
                print("MQTT Error:", err)

    def gen_handler(self):
        def handle_message(client, topic, message):
            print("Got message: "+message+" from node "+self.name, self.id, topic)
            self.rfm.send(bytes(message, "ascii"), destination=int(self.id))
            self.rfm.send(bytes(message, "ascii"), destination=int(self.id))
            # __name__= self.name
        return handle_message


def init_network():
    print("Connecting to wifi ..")
    while not radio.connected:
        try:
            radio.connect(getenv("CIRCUITPY_WIFI_SSID"),
                          getenv("CIRCUIT_WIFI_PASSWORD"))
        except:
            continue
    print("Connected with ip", radio.ipv4_address)
    sock_pool = SocketPool(radio)
    ssl_context = create_default_context()
    return sock_pool, ssl_context


def init_mqtt(sock_pool, ssl_cotext):
    client = MQTT.MQTT(
        broker=getenv("MQTT_IP"),
        port=1883,
        socket_pool=sock_pool,
        ssl_context=ssl_cotext,
        username=getenv("MQTT_USER"),
        password=getenv("MQTT_PASS"),
    )
    client.connect()
    return client


def init_rfm69(cs, reset, sck, mosi, miso):
    spi = SPI(sck, mosi, miso)
    rfm = RFM69(spi, cs, reset, RADIO_FREQ_MHZ)
    rfm.node = 0
    rfm.encryption_key = (
        b"\x01\x02\x03\x04\x05\x06\x07\x08\x01\x02\x03\x04\x05\x06\x07\x08"
    )
    return rfm


def init_gw(client):
    gw_topic_base = "homeassistant/binary_sensor/rfgw"

    gw_presentation_dict = {
        'name': "RFGw",
        'device_class': "running",
        'state_topic': gw_topic_base + "/state",
        'command_topic': gw_topic_base + "/set",
        'unique_id': "rfgw01",
        'device': {'identifiers': ["rfgw01"], 'name': "rfgw01"},
    }

    gw_payload = dumps(gw_presentation_dict)

    client.publish(gw_topic_base+"/config", gw_payload)
    client.publish(gw_topic_base+"/state", "Running")
    return gw_topic_base, gw_payload


async def send_ping(rfm):
    while True:
        print("Sending ping to all nodes ..")
        rfm.send_with_ack("ping")
        await sleep(30)


async def handle_rfm_receive(rfm, mqtt):
    while True:
        pkt = rfm.receive()
        if pkt is None:
            MY_LED.value = False
        else:
            print("Got packet ..")
            MY_LED.value = True
            msg = NodeMsg(pkt, mqtt, rfm)
            msg.process()
        await sleep_ms(1)


async def handle_mqtt_loop(mqtt):
    while True:
        mqtt.loop()
        await sleep_ms(1)


async def represent(client, topic, payload):
    while True:
        client.publish(topic+"/config", payload)
        client.publish(topic+"/state", "Running")
        await sleep(60)


async def main():
    # connect to wifi
    print("Main ..")
    pool, ssl_con = init_network()
    mqtt_client = init_mqtt(pool, ssl_con)
    topic, payload = init_gw(mqtt_client)
    rfm69 = init_rfm69(CS, RESET, GP18, GP19, GP16)
    ping_task = create_task(send_ping(rfm69))
    rfm_recv_task = create_task(handle_rfm_receive(rfm69, mqtt_client))
    mqtt_loop_task = create_task(handle_mqtt_loop(mqtt_client))
    repr_task = create_task(represent(mqtt_client, topic, payload))
    await gather(ping_task, rfm_recv_task, mqtt_loop_task, repr_task)

run(main())
