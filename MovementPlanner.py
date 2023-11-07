#!/usr/bin/env python
import signal
import sys
import time
from copy import copy

import numpy as np
from time import sleep

from AbstractVirtualCapability import AbstractVirtualCapability, VirtualCapabilityServer, formatPrint


class MovementPlanner(AbstractVirtualCapability):
    def __init__(self, server):
        super().__init__(server)
        self.position = [0., 0., 0.]
        self.uri = "MovementPlanner"
        #self.funtionality = {"set_pos_viz": None, "get_name": None, "set_name": None, "get_pos": None}
        self.current_block_id = None
        self.trajectories = []

    def plan_movement(self, params: dict):
        start = params["StartPoint"]
        end = params["EndPoint"]
        norm_want_dir = (np.linalg.norm(np.array(end) - np.array(start)))
        if norm_want_dir == 0:
            return {"ListOfPoints": [end]}
        want_dir = [1. if x > 0.0 or x < 0.0 else 0. for x in np.abs((np.array(end) - np.array(start)) / norm_want_dir)]
        norm_current_dir = np.linalg.norm(np.array(params["Vector3"]))
        if norm_current_dir == 0:
            raise ValueError(f"No Direction set! Direction: " + params["Vector3"])
        current_dir = [1. if x > 0.0 or x < 0.0 else 0. for x in (np.abs(np.array(params["Vector3"]) / norm_current_dir))]

        # Directions not aligned
        if not np.array_equal(current_dir, want_dir):
            raise ValueError(f"Direction doesn't work out! Wanted direction: {want_dir}, actual direction {current_dir}")

        path_points = [start]

        blocks = self.invoke_sync("get_all_blocks", {})["ListOfPoints"]
        block_dims = self.invoke_sync("GetBlockDimensions", {})["ListOfPoints"]
        formatPrint(self, f"BlockDims: {block_dims}")
        current_height = start[2]
        final_direction = [1. if x > 0.0 else -1. if x < 0.0 else 0. for x in (np.array(end) - np.array(start))]
        for b in blocks:
            block_dir = [1. if x > 0.0 else -1. if x < 0.0 else 0. for x in (np.array(b) - np.array(start))]
            # Block in the way
            if np.array_equal(block_dir, final_direction):
                formatPrint(self, f"Block in the Way: {b}, from Start {start} to {end}, Dir: {final_direction} vs {block_dir}")
                distances = final_direction * ((np.array(end) - np.array(start)) - (np.array(b) - np.array(start)))
                # Block will be reached.
                if (np.array(distances) < 0).any():
                    formatPrint(self, f"Distances: {distances} - {start}<{b}<{end}")
                    point = copy(b)
                    point[2] += block_dims[2]
                    path_points += [point]
        path_points += [end]
        return {"ListOfPoints": path_points}

    def callback(self, params: dict):
        print(f"GetPosition: {params}")

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
