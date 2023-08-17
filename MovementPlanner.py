#!/usr/bin/env python
import signal
import sys
import time
from copy import copy
from time import sleep

from AbstractVirtualCapability import AbstractVirtualCapability, VirtualCapabilityServer, formatPrint


class MovementPlanner(AbstractVirtualCapability):
    def __init__(self, server):
        super().__init__(server)
        self.position = [0., 0., 0.]
        self.uri = "MovementPlanner"
        #self.funtionality = {"set_pos_viz": None, "get_name": None, "set_name": None, "get_pos": None}
        self.current_block_id = None

    def plan_movement(self, params: dict):
        start = params["StartPoint"]
        end = params["EndPoint"]
        blocks = self.invoke_sync("get_all_blocks", {})["ListOfPoints"]
        point = self.invoke_sync("GetPosition", {})["Position3D"]
        print(f"GOING HAYWIRE {start} -> {end} with blocking: {blocks} and point {point}")
        return {"ListOfPoints": blocks}

    def loop(self):
        sleep(.0001)


if __name__ == '__main__':
    # Needed for properly closing when process is being stopped with SIGTERM signal
    def handler(signum, frame):
        print("[Main] Received SIGTERM signal")
        listener.kill()
        quit(1)


    try:
        port = None
        if len(sys.argv[1:]) > 0:
            port = int(sys.argv[1])
        server = VirtualCapabilityServer(port)
        listener = MovementPlanner(server)
        listener.start()
        signal.signal(signal.SIGTERM, handler)
        listener.join()
    # Needed for properly closing, when program is being stopped wit a Keyboard Interrupt
    except KeyboardInterrupt:
        print("[Main] Received KeyboardInterrupt")
        server.kill()
        listener.kill()
