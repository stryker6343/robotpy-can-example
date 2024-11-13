import wpilib


class CANSim:

    def __init__(self, can_object: wpilib.CAN):
        self.can = can_object
        self.buffer = {}
        setattr(self, "readPacketNew", self.read_packet_new_simulated)

    def read_packet_new_simulated(self, apiId: int, data: wpilib.CANData) -> bool:
        if apiId in self.buffer:
            data = self.buffer[apiId]
            return True
        else:
            return False

    def create_bus_packet(self, apiId: int, data: bytes, timestamp: int = 0):
        self.buffer[apiId] = wpilib.CANData()
        self.buffer[apiId].data = data
        self.buffer[apiId].length = len(data)
        self.buffer[apiId].timestamp = timestamp

