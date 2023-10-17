import asyncio
import json
import argparse
import websockets
from pythonosc import osc_message_builder, udp_client, dispatcher, osc_server
from websockets import connect, WebSocketServerProtocol, serve
from pythonosc.udp_client import SimpleUDPClient


def send_osc_to_ws(address, *args):
    json_data = {"address": address, "args": args}
    ws_server.send_message_to_all(json.dumps(json_data))

def send_ws_to_osc(ws, path):
    json_data = json.loads(path)
    osc_client.send_message(json_data["address"], json_data["args"])

async def bridge(ws_url, osc_address, osc_port):
    osc_client = SimpleUDPClient(osc_address, osc_port)
    osc_dispatcher = dispatcher.Dispatcher()
    osc_dispatcher.map("*", send_osc_to_ws)
    osc_server.AsyncIOOSCUDPServer((osc_address, osc_port), osc_dispatcher, asyncio.get_event_loop())

    transport, protocol = await osc_server.create_serve_endpoint()  # Create datagram endpoint and start serving    
        
    async with websockets.serve(send_ws_to_osc, ws_url, 8765) as ws_server:
        await asyncio.Future()
    transport.close()


async def connect_ws_to_osc(ws_url, osc_address, osc_port):
    async with connect(ws_url) as websocket:
        while True:
            message = await websocket.recv()
            json_data = json.loads(message)

            print("Ws recv",json_data)
            osc_client = udp_client.SimpleUDPClient(osc_address, osc_port)
            osc_client.send_message(json_data, json_data)

    

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ws_address", default="ws://localhost", help="The websocket URL to connect to.")
    parser.add_argument("--ws_port", default=9090, help="The websocket Port to connect to.", type=int)
    parser.add_argument("--osc_address", default="localhost",help="The OSC address to send messages to.")
    parser.add_argument("--osc_port", default=9999, help="The OSC port to send messages to.", type=int)
    args = parser.parse_args()

    osc_dispatcher = dispatcher.Dispatcher()

    asyncio.get_event_loop().run_until_complete(bridge(args.ws_address+':'+str(args.ws_port), args.osc_address, args.osc_port))