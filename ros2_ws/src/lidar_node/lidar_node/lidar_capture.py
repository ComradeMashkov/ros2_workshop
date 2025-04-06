import math
import serial
import struct
import threading

class Capture:
    def __init__(self, serial_port, data_size=460, is_invert=True):
        self.theta = [0.0] * data_size
        self.distance = [0.0] * data_size
        self.write_pos = 0
        self.serial = serial.Serial(port=serial_port, baudrate=115200)
        self.data_size = data_size
        self.thread = threading.Thread(target=self._get_data)
        self.lock = threading.Lock()
        self.is_invert = is_invert
        self.data_obtained = False
        self.rpm = 0
        self.running = True

    def _get_data_unit(self):
        header = b"\x55\xAA\x03\x08"
        header_pos = 0
        while True:
            tmp = self.serial.read(1)
            if tmp[0] == header[header_pos]:
                header_pos += 1
                if header_pos == len(header):
                    break
            else:
                header_pos = 0

        tmp = self.serial.read(4)
        rotation_speed_tmp, start_angle_tmp = struct.unpack_from("<2H", tmp)
        self.rpm = rotation_speed_tmp / 64
        start_angle = (start_angle_tmp - 0xA000) / 64

        distance_tmp = [0] * 8
        intensity_tmp = [0] * 8
        for i in range(8):
            tmp = self.serial.read(3)
            distance_tmp[i], intensity_tmp[i] = struct.unpack_from("<HB", tmp)

        tmp = self.serial.read(4)
        end_angle_tmp, _ = struct.unpack_from("<2H", tmp)
        end_angle = (end_angle_tmp - 0xA000) / 64

        return distance_tmp, intensity_tmp, start_angle, end_angle
    
    def _get_data(self):
        pre_start_angle = 0
        while self.running:
            distance_tmp, _, start_angle, end_angle = self._get_data_unit()

            if end_angle < start_angle:
                end_angle += 360
            if (start_angle - pre_start_angle < 0):
                self.data_obtained = True
            pre_start_angle = start_angle

            start_angle_rad = start_angle * math.pi / 180 * (-1 if self.is_invert else 1)
            end_angle_rad = end_angle * math.pi / 180 * (-1 if self.is_invert else 1)
            angle_increment = (end_angle_rad - start_angle_rad) / len(distance_tmp)

            with self.lock:
                for i in range(len(distance_tmp)):
                    self.theta[self.write_pos] = start_angle_rad + angle_increment * i
                    self.distance[self.write_pos] = distance_tmp[i] / 1000.0  # meters
                    self.write_pos = (self.write_pos + 1) % self.data_size

    def start(self):
        self.thread.start()

    def stop(self):
        self.running = False
        self.thread.join()

    def get_scan_data(self):
        with self.lock:
            return list(self.theta), list(self.distance)
        