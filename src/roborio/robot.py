"""
    Custom CAN Bus Application Example Program
"""

import inspect
import struct
import datetime
from typing import cast, Tuple

import wpilib
import robotpy_rev_digit


# CAN Bus settings
DEVICE_NUMBER = 0

# API/Message Identifier for Manufacturer ID = 8, Device Type = 10
API_CLASS_COUNTER = 0  # Counting Device
API_INDEX_COUNTER_CONTROL = 1  # Counting Device Control
API_INDEX_COUNTER_COUNT = 2  # Counting Device Count Value


def can_api(api_class: int, api_index: int) -> int:
    """Constructs an CAN API value from a given API class and API index."""
    return (api_class << 4) + api_index


def encode_counter_command_packet(
    enabled: bool, button_a: bool, button_b: bool, speed: int
) -> bytes:
    """
    Construct a counter command message packet
    """
    flags = 0
    for flag in [enabled, button_b, button_a]:
        flags = flags << 1
        flags = flags | 1 if flag else flags
    return struct.pack("<BB", speed, flags)


def decode_counter_count_packet(packet: wpilib.CANData) -> int:
    """
    Extracts a count value from the counter command packet.
    """
    (count,) = cast(Tuple[int], struct.unpack("<H", packet.data[:2]))
    return count


# The timed robot scheduler will call the methods of this class both periodically
# and during transitions between the four robot modes. The four robot modes are
# disabled, autonomous, teleop and test. For each mode, the class provides an
# init method, a periodic method and an exit method. Also, the class provides robot
# methods that are called regardless of robot mode.
class MyRobot(wpilib.TimedRobot):
    """
    CAN Bus Example Program
    """

    def robotInit(self):
        """
        This method is called upon program startup and should be used for any
        initialization code.  The Driver Station Robot Code light will turn green
        when this method exits.
        """
        # Display information about this robot program on the driver station console
        print(inspect.getdoc(self))
        print(f"Robot started up at {datetime.datetime.now(datetime.UTC)} UTC")
        # print(f"robotpy_rev_digit version: {robotpy_rev_digit.__version__}")

        # Set up a CAN device interface with the specified device number
        # By default, the Manufacturer ID = 8 and Device Type = 10
        self.can = wpilib.CAN(DEVICE_NUMBER)

        # Set up the REV Digit MXP Board
        self.rev_digit = robotpy_rev_digit.RevDigitBoard()

        # Set up an interface to the roboRIO's internal functions
        self.robot = wpilib.RobotController

    def autonomousInit(self):
        """This method is called once when the robot enters autonomous mode."""
        self.rev_digit.clear_display()

    def autonomousPeriodic(self):
        """This method is called periodically during autonomous."""
        # Read the state of the REv Digit buttons and potentiometer
        button_a = self.rev_digit.button_a
        button_b = self.rev_digit.button_b
        speed = int(256 * self.rev_digit.potentiometer / 5 // 1)

        # Transmit control packet to the Counter Device and enable the counter
        msg = encode_counter_command_packet(True, button_a, button_b, speed)
        self.can.writePacket(
            data=msg, apiId=can_api(API_CLASS_COUNTER, API_INDEX_COUNTER_CONTROL)
        )

        # Receive the latest count from the Count Device
        buffer = wpilib.CANData()
        if self.can.readPacketLatest(
            apiId=can_api(API_CLASS_COUNTER, API_INDEX_COUNTER_COUNT), data=buffer
        ):
            self.rev_digit.display_message(hex(decode_counter_count_packet(buffer)))

    def autonomousExit(self):
        """This method is called once when autonomous mode exits."""
        # Disable the counter
        msg = encode_counter_command_packet(False, True, True, True)
        self.can.writePacket(
            data=msg, apiId=can_api(API_CLASS_COUNTER, API_INDEX_COUNTER_CONTROL)
        )

    def disabledPeriodic(self):
        """This method is called periodically while the robot is disabled."""
        # Display the battery voltage on the display when the robot is disabled
        self.rev_digit.display_message(self.robot.getBatteryVoltage())

    # This method is included here to suppress warnings on the Driver Station console
    def robotPeriodic(self):
        """This method is called periodically regardless of robot mode."""
        pass


if __name__ == "__main__":
    wpilib.run(MyRobot)
