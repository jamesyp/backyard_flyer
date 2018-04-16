import argparse
import time
from enum import Enum

import numpy as np
import math

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID

class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5

class BackyardFlyer(Drone):
    def __init__(self, connection, sq_len):
        super().__init__(connection, "log", "nav.log")

        self.sq_len = sq_len
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True

        self.flight_state = States.MANUAL

        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def start(self):
        print("Creating log file")
        self.start_log("log", "nav.log")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            target_altitude = 0.95 * self.target_position[2]

            if self.corrected_altitude() > target_altitude:
                self.all_waypoints = self.calculate_box()
                self.waypoint_idx = 0
                self.waypoint_transition()

        if self.flight_state == States.WAYPOINT:
            adjusted_local = np.copy(self.local_position)
            adjusted_local[2] *= -1
            distance = np.linalg.norm(self.target_position[:3] - adjusted_local)

            if distance < 0.5:
                if self.waypoint_idx < len(self.all_waypoints):
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if np.linalg.norm(self.local_velocity) < 0.1:
                alt_distance = abs(self.starting_altitude - self.corrected_altitude())
                if abs(alt_distance) < 0.02:
                    self.disarming_transition()

    def state_callback(self):
        if not self.in_mission:
            return
        if self.flight_state == States.MANUAL:
            self.arming_transition()
        if self.flight_state == States.ARMING:
            self.takeoff_transition()
        if self.flight_state == States.DISARMING:
            self.manual_transition()

    def calculate_box(self):
        forward = 0
        right = math.radians(90)
        left = math.radians(-90)
        back = math.radians(180)
        return [
            (        0.0, self.sq_len, self.low_alt,  right),
            (        0.0, self.sq_len, self.high_alt, forward),
            (self.sq_len, self.sq_len, self.high_alt, forward),
            (self.sq_len, self.sq_len, self.low_alt,  left),
            (self.sq_len,         0.0, self.low_alt,  left),
            (self.sq_len,         0.0, self.high_alt, back),
            (        0.0,         0.0, self.high_alt, back),
            (        0.0,         0.0, self.low_alt,  forward)
        ]

    def arming_transition(self):
        if self.global_position[0] == 0.0 and self.global_position[1] == 0.0:
            print("waiting for global position data")
            return

        print("arming transition")
        self.take_control()
        self.arm()

        self.set_home_position(*self.global_position)
        self.starting_altitude = self.global_position[2]
        self.low_alt  = self.starting_altitude + 5.0
        self.high_alt = self.low_alt + self.sq_len

        print('starting global position:', self.global_position)
        print('home position:', self.global_home)
        self.flight_state = States.ARMING

    def takeoff_transition(self):
        self.target_position[2] = self.low_alt
        print("takeoff transition - altitude", self.low_alt)

        self.takeoff(self.low_alt)
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        print("\nwaypoint transition")

        self.target_position = np.array(self.all_waypoints[self.waypoint_idx])
        print('target:', self.target_position)
        self.cmd_position(*self.target_position)
        self.waypoint_idx += 1

        self.flight_state = States.WAYPOINT

    def landing_transition(self):
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        print("disarm transition")
        self.disarm()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        print("manual transition")
        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def corrected_altitude(self):
        return -1 * self.local_position[2]

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port',  type=int,   default=5760, help='Port number')
    parser.add_argument('--host',  type=str,   default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    parser.add_argument('--sqlen', type=float, default=10.0, help="The length of the drone's flight square in meters")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    drone = BackyardFlyer(conn, args.sqlen)
    time.sleep(1)
    drone.start()
