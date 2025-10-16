#!/usr/bin/python3
import rclpy
from rclpy.node import Node


# import srv
from seer_robot_interfaces.srv import (
    PathReservation, PathReleaseReservationByStation, PathReleaseReservationByRoute, CheckPathReservation)
from std_srvs.srv import Trigger


class PathReservationNode(Node):
    def __init__(self):
        super().__init__('PathReservation')
        self.get_logger().info("PathReservation node has been started.")

        # Path Reservations storage
        self.Path_reservation = []

        # Service server
        self.create_service(PathReservation, 'path_reservation/reserve_station', self.PathReservation_callback)
        self.create_service(PathReleaseReservationByStation, 'path_reservation/release_station_by_station', self.PathReleaseReservation_by_station_callback)
        self.create_service(PathReleaseReservationByRoute, 'path_reservation/release_station_by_route', self.PathReleaseReservation_by_route_callback)
        self.create_service(CheckPathReservation, 'path_reservation/check_reservation', self.PathReservationCheck_callback)
        self.create_service(Trigger, 'path_reservation/clear_all_reservations', self.PathReservationClassAll_callback)

    #####################################################
    ###             Path Reservation                  ###
    #####################################################

    def PathReservation_callback(self, request, response):
        robot_id = request.this_robot_id
        route = request.route_for_reservation  # List of station IDs to reserve

        # Check station in route is already reserved
        for station_id in route:
            for reservation in self.Path_reservation:
                if reservation['station_id'] == station_id:
                    if reservation['robot_id'] == robot_id:
                        # Same robot, pop the station
                        self.Path_reservation.remove(reservation)
                        self.get_logger().info(f"Station {station_id} popped from reservation for robot {robot_id}.")
                    else:
                        # Different robot, cannot reserve
                        response.has_reservation = False
                        response.message = f"Station {station_id} is already reserved by robot_0{reservation['robot_id']}."
                        self.get_logger().info(response.message)
                        return response

        # Reserve stations for the robot
        for station_id in route:
            self.Path_reservation.append(
                {'robot_id': robot_id, 'station_id': station_id})

        try:
            response.has_reservation = True
            response.message = f"Stations {route} successfully reserved for robot {robot_id}."
            self.get_logger().info(response.message)
            self.get_logger().info(
                f"Current Reservations: {self.Path_reservation}")
            return response
        except Exception as e:
            response.has_reservation = False
            response.message = f"Failed to reserve stations {route} for robot {robot_id}: {str(e)}"
            self.get_logger().error(response.message)
            return response
    
    #####################################################
    ###             Path Release Reservation          ###
    #####################################################

    def PathReleaseReservation_by_station_callback(self, request, response):
        robot_id = request.this_robot_id
        station_id_for_release = request.station_for_release

        # Check if the station is reserved by the requesting robot
        for reservation in self.Path_reservation:
            if reservation['station_id'] == station_id_for_release and reservation['robot_id'] == robot_id:
                self.Path_reservation.remove(reservation)
                response.has_released = True
                response.message = f"Station {station_id_for_release} released from reservation for robot {robot_id}."
                self.get_logger().info(response.message)
                self.get_logger().info(
                    f"Current Reservations: {self.Path_reservation}")
                return response
        try:
            response.has_released = False
            response.message = f"Station {station_id_for_release} is not reserved by robot {robot_id}."
            self.get_logger().info(response.message)
            return response
        except Exception as e:
            response.has_released = False
            response.message = f"Failed to release station {station_id_for_release} for robot {robot_id}: {str(e)}"
            self.get_logger().error(response.message)
            return response

    def PathReleaseReservation_by_route_callback(self, request, response):
        robot_id = request.this_robot_id
        # List of station IDs to release
        route_station_for_release = request.route_station_for_release

        # Release stations reserved by the robot
        released_stations = []
        for station_id in route_station_for_release:
            for reservation in self.Path_reservation:
                if reservation['station_id'] == station_id and reservation['robot_id'] == robot_id:
                    self.Path_reservation.remove(reservation)
                    released_stations.append(station_id)

        try:
            if released_stations:
                response.has_released = True
                response.message = f"Stations {released_stations} released from reservation for robot {robot_id}."
            else:
                response.has_released = False
                response.message = f"No stations from {route_station_for_release} were reserved by robot {robot_id}."
            self.get_logger().info(response.message)
            self.get_logger().info(
                f"Current Reservations: {self.Path_reservation}")
            return response
        except Exception as e:
            response.has_released = False
            response.message = f"Failed to release stations {route_station_for_release} for robot {robot_id}: {str(e)}"
            self.get_logger().error(response.message)
            return response

    #####################################################
    ###             Path Reservation Check            ###
    #####################################################

    def PathReservationCheck_callback(self, request, response):
        # Return only the list of reserved station IDs as strings
        # Return list of station IDs as strings
        response.station_has_reservation = [
            str(reservation['station_id']) for reservation in self.Path_reservation]
        self.get_logger().info(
            f"Current Reservations: {self.Path_reservation}")
        return response

    #####################################################
    ###             Class all reservation            ###
    #####################################################

    def PathReservationClassAll_callback(self, request, response):
        # Clear all reservations
        self.Path_reservation = []
        response.success = True
        response.message = "All reservations have been cleared."

        self.get_logger().info(response.message)
        self.get_logger().info(
            f"Current Reservations: {self.Path_reservation}")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = PathReservationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
