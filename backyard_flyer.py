import argparse
import time
from enum import Enum

# https://numpy.org/doc/stable/reference/generated/numpy.linalg.norm.html
import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            # Once we have taken off and we are almost at the target position then move to the first way point
            if abs(self.local_position[2]) >= self.target_position[2]:
                self.all_waypoints = self.calculate_box()
                self.waypoint_transition()

        elif self.flight_state == States.WAYPOINT:
            norm_threshold = 1.0
            # when the diff between my current location and the target location is very small then
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < norm_threshold:
                # Process next way point
                if len(self.all_waypoints) > 0:
                    self.waypoint_transition()
                # Once we consumed all way points transition to landing
                else:
                    self.landing_transition()

    def velocity_callback(self):
        altitude_threshold = 1.5
        if self.flight_state == States.LANDING:
            # If the altitude difference between my current GPS altitude and the home GPS altitude then.
            if (self.global_position[2] - self.global_home[2] < altitude_threshold) \
                    and self.local_position[2] < altitude_threshold:
                # When is landing and altitude is less than 0.5 then disarm and release control
                self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    @staticmethod
    def calculate_box():
        print("Creating flight plan")
        altitude = 10.0
        local_waypoints = [
            [15.0, 0.0, altitude],
            [15.0, 15.0, altitude],
            [0.0, 15.0, altitude],
            [0.0, 0.0, altitude]
        ]
        return local_waypoints

    def arming_transition(self):
        self.take_control()
        self.arm()

        longitude = self.global_position[0]
        latitude = self.global_position[1]
        altitude = self.global_position[2]
        self.set_home_position(longitude, latitude, altitude)

        self.flight_state = States.ARMING
        print("arming transition")

    def takeoff_transition(self):
        target_altitude = 3.0
        # Set to target_position the target_altitude
        self.target_position[2] = target_altitude
        self.takeoff(target_altitude)
        self.flight_state = States.TAKEOFF
        print("takeoff transition")

    def waypoint_transition(self):
        self.target_position = self.all_waypoints.pop(0)

        longitude = self.target_position[0]
        latitude = self.target_position[1]
        altitude = self.target_position[2]
        self.cmd_position(longitude, latitude, altitude, 0)

        self.flight_state = States.WAYPOINT
        print("point transition to", self.target_position)

    def landing_transition(self):
        self.land()
        self.flight_state = States.LANDING
        print("landing transition")

    def disarming_transition(self):
        self.disarm()
        self.release_control()
        self.flight_state = States.DISARMING
        print("disarm transition")

    def manual_transition(self):
        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL
        print("manual transition")

    def start(self):
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


def create_connection():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    connection = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    # conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    return connection


if __name__ == "__main__":
    conn = create_connection()
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
