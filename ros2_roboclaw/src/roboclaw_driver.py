import struct
import sys

class SerialPort(object):
    def __init__(self, serial_port):
        self.__port = serial_port
        self.__checksum = 0

    def close(self):
        self.__port.close()

    def reset_port(self):
        result = ''
        flushing = True

        self.__port.flush()
        self.__port.flushInput()
        self.__port.flushOutput()

        while flushing:
            char = self.__port.read(1)
            flushing = (char != '')
            result += char

        sys.stderr.write('\n===============\nFLUSH SERIAL PORT\n===============\n%s\n===============\n' % result)

    def get_checksum(self, mask=0x7F):
        return self.__checksum & mask

    def reset_checksum(self, value=0x0):
        self.__checksum = value

    def read(self, size):
        return self.__port.read(size)

    def write(self, char):
        self.__port.write(char)

    def flush(self):
        self.__port.flush()

    def send_command(self, address, command):
        self.write_byte(address)
        self.write_byte(command)

    def read_byte(self):
        res = self.__port.read(1)
        if len(res) == 1:
            val = struct.unpack('>B', res)
            self.__checksum = (val[0] + self.__checksum) & 0xFF
            return val[0]
        return None

    def read_sbyte(self):
        res = self.__port.read(1)
        if len(res) == 1:
            val = struct.unpack('>b', res)
            self.__checksum = (val[0] + self.__checksum) & 0xFF
            return val[0]
        return None

    def read_word(self):
        res = self.__port.read(2)
        if len(res) == 2:
            val = struct.unpack('>H', res)
            self.__checksum = (val[0] + self.__checksum) & 0xFF
            self.__checksum = ((val[0] >> 8) + self.__checksum) & 0xFF
            return val[0]
        return None

    def read_sword(self):
        res = self.__port.read(2)
        if len(res) == 2:
            val = struct.unpack('>h', res)
            self.__checksum = (val[0] + self.__checksum) & 0xFF
            self.__checksum = ((val[0] >> 8) + self.__checksum) & 0xFF
            return val[0]
        return None

    def read_long(self):
        res = self.__port.read(4)
        if len(res) == 4:
            val = struct.unpack('>L', res)
            self.__checksum = (val[0] + self.__checksum) & 0xFF
            self.__checksum = ((val[0] >> 8) + self.__checksum) & 0xFF
            self.__checksum = ((val[0] >> 16) + self.__checksum) & 0xFF
            self.__checksum = ((val[0] >> 24) + self.__checksum) & 0xFF
            return val[0]
        return None

    def read_slong(self):
        res = self.__port.read(4)
        if len(res) == 4:
            val = struct.unpack('>l', res)
            self.__checksum = (val[0] + self.__checksum) & 0xFF
            self.__checksum = ((val[0] >> 8) + self.__checksum) & 0xFF
            self.__checksum = ((val[0] >> 16) + self.__checksum) & 0xFF
            self.__checksum = ((val[0] >> 24) + self.__checksum) & 0xFF
            return val[0]
        return None

    def write_byte(self, val):
        self.__checksum = (val + self.__checksum) & 0xFF
        return self.__port.write(struct.pack('>B', val))

    def write_sbyte(self, val):
        self.__checksum = (val + self.__checksum) & 0xFF
        return self.__port.write(struct.pack('>b', val))

    def write_word(self, val):
        self.__checksum = (val + self.__checksum) & 0xFF
        self.__checksum = ((val >> 8) + self.__checksum) & 0xFF
        return self.__port.write(struct.pack('>H', val))

    def write_sword(self, val):
        self.__checksum = (val + self.__checksum) & 0xFF
        self.__checksum = ((val >> 8) + self.__checksum) & 0xFF
        return self.__port.write(struct.pack('>h', val))

    def write_long(self, val):
        self.__checksum = (val + self.__checksum) & 0xFF
        self.__checksum = ((val >> 8) + self.__checksum) & 0xFF
        self.__checksum = ((val >> 16) + self.__checksum) & 0xFF
        self.__checksum = ((val >> 24) + self.__checksum) & 0xFF
        return self.__port.write(struct.pack('>L', val))

    def write_slong(self, val):
        self.__checksum = (val + self.__checksum) & 0xFF
        self.__checksum = ((val >> 8) + self.__checksum) & 0xFF
        self.__checksum = ((val >> 16) + self.__checksum) & 0xFF
        self.__checksum = ((val >> 24) + self.__checksum) & 0xFF
        return self.__port.write(struct.pack('>l', val))
        
class Roboclaw(object):
    def __init__(self, port, rc_address=128):
        self.__port = port
        self.__rc_address = rc_address
        
    def reset_port(self):
        self.drive_mixed_with_signed_duty_cycle(0, 0)
        self.__port.reset_port()

    def flush(self):
        self.__port.flush()

    def terminate(self):
        self.reset_port()
        self.__port.close()

    def drive_forward_m1(self, val):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 0)
        self.__port.write_byte(val)
        self.__port.write_byte(self.__port.get_checksum())

    def drive_backwards_m1(self, val):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 1)
        self.__port.write_byte(val)
        self.__port.write_byte(self.__port.get_checksum())

    def set_min_main_voltage(self, val):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 2)
        self.__port.write_byte(val)
        self.__port.write_byte(self.__port.get_checksum())

    def set_max_main_voltage(self, val):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 3)
        self.__port.write_byte(val)
        self.__port.write_byte(self.__port.get_checksum())

    def drive_forward_m2(self, val):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 4)
        self.__port.write_byte(val)
        self.__port.write_byte(self.__port.get_checksum())

    def drive_backwards_m2(self, val):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 5)
        self.__port.write_byte(val)
        self.__port.write_byte(self.__port.get_checksum())

    def drive_m1(self, val):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 6)
        self.__port.write_byte(val)
        self.__port.write_byte(self.__port.get_checksum())

    def drive_m2(self, val):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 7)
        self.__port.write_byte(val)
        self.__port.write_byte(self.__port.get_checksum())

    def drive_forward(self, val):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 8)
        self.__port.write_byte(val)
        self.__port.write_byte(self.__port.get_checksum())

    def drive_backwards(self, val):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 9)
        self.__port.write_byte(val)
        self.__port.write_byte(self.__port.get_checksum())

    def turn_right(self, val):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 10)
        self.__port.write_byte(val)
        self.__port.write_byte(self.__port.get_checksum())

    def turn_left(self, val):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 11)
        self.__port.write_byte(val)
        self.__port.write_byte(self.__port.get_checksum())

    def drive_forward_or_backwards(self, val):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 12)
        self.__port.write_byte(val)
        self.__port.write_byte(self.__port.get_checksum())

    def turn_left_or_right(self, val):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 13)
        self.__port.write_byte(val)
        self.__port.write_byte(self.__port.get_checksum())

    def read_quad_encoder_register_m1(self):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 16)
        enc = self.__port.read_slong()
        status = self.__port.read_byte()
        crc = self.__port.get_checksum()
        if crc == self.__port.read_byte():
            return enc, status
        return -1, -1

    def read_quad_encoder_register_m2(self):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 17)
        enc = self.__port.read_slong()
        status = self.__port.read_byte()
        crc = self.__port.get_checksum()
        if crc == self.__port.read_byte():
            return enc, status
        return -1, -1

    def read_speed_m1(self):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 18)
        enc = self.__port.read_slong()
        status = self.__port.read_byte()
        crc = self.__port.get_checksum()
        if crc == self.__port.read_byte():
            return enc, status
        return -1, -1

    def read_speed_m2(self):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 19)
        enc = self.__port.read_slong()
        status = self.__port.read_byte()
        crc = self.__port.get_checksum()
        if crc == self.__port.read_byte():
            return enc, status
        return -1, -1

    def reset_quad_encoder_counters(self):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 20)
        self.__port.write_byte(self.__port.get_checksum())

    def read_firmware_version(self):
        self.__port.send_command(self.__rc_address, 21)
        # FIXME(paoolo): Check docs
        return self.__port.read(32)

    def read_main_battery_voltage_level(self):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 24)
        val = self.__port.read_word()
        crc = self.__port.get_checksum()
        if crc == self.__port.read_byte():
            return val
        return -1

    def read_logic_battery_voltage_level(self):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 25)
        val = self.__port.read_word()
        crc = self.__port.get_checksum()
        if crc == self.__port.read_byte():
            return val
        return -1

    def set_min_logic_voltage_level(self, val):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 26)
        self.__port.write_byte(val)
        self.__port.write_byte(self.__port.get_checksum())

    def set_max_logic_voltage_level(self, val):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 27)
        self.__port.write_byte(val)
        self.__port.write_byte(self.__port.get_checksum())

    def set_pid_constants_m1(self, p, i, d, qpps):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 28)
        self.__port.write_long(d)
        self.__port.write_long(p)
        self.__port.write_long(i)
        self.__port.write_long(qpps)
        self.__port.write_byte(self.__port.get_checksum())

    def set_pid_constants_m2(self, p, i, d, qpps):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 29)
        self.__port.write_long(d)
        self.__port.write_long(p)
        self.__port.write_long(i)
        self.__port.write_long(qpps)
        self.__port.write_byte(self.__port.get_checksum())

    def read_current_speed_m1(self):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 30)
        enc = self.__port.read_slong()
        status = self.__port.read_byte()
        crc = self.__port.get_checksum()
        if crc == self.__port.read_byte():
            return enc, status
        return -1, -1

    def read_current_speed_m2(self):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 31)
        enc = self.__port.read_slong()
        status = self.__port.read_byte()
        crc = self.__port.get_checksum()
        if crc == self.__port.read_byte():
            return enc, status
        return -1, -1

    def drive_m1_with_signed_duty_cycle(self, val):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 32)
        self.__port.write_sword(val)
        self.__port.write_byte(self.__port.get_checksum())

    def drive_m2_with_signed_duty_cycle(self, val):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 33)
        self.__port.write_sword(val)
        self.__port.write_byte(self.__port.get_checksum())

    def drive_mixed_with_signed_duty_cycle(self, m1, m2):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 34)
        self.__port.write_sword(m1)
        self.__port.write_sword(m2)
        self.__port.write_byte(self.__port.get_checksum())

    def drive_m1_with_signed_speed(self, val):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 35)
        self.__port.write_slong(val)
        self.__port.write_byte(self.__port.get_checksum())

    def drive_m2_with_signed_speed(self, val):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 36)
        self.__port.write_slong(val)
        self.__port.write_byte(self.__port.get_checksum())

    def drive_mixed_with_signed_speed(self, m1, m2):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 37)
        self.__port.write_slong(m1)
        self.__port.write_slong(m2)
        self.__port.write_byte(self.__port.get_checksum())

    def drive_m1_with_signed_speed_accel(self, accel, speed):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 38)
        self.__port.write_long(accel)
        self.__port.write_slong(speed)
        self.__port.write_byte(self.__port.get_checksum())

    def drive_m2_with_signed_speed_accel(self, accel, speed):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 39)
        self.__port.write_long(accel)
        self.__port.write_slong(speed)
        self.__port.write_byte(self.__port.get_checksum())

    def drive_mixed_with_signed_speed_accel(self, accel, speed1, speed2):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 40)
        self.__port.write_long(accel)
        self.__port.write_slong(speed1)
        self.__port.write_slong(speed2)
        self.__port.write_byte(self.__port.get_checksum())

    def buffered_m1_drive_with_signed_speed_distance(self, speed, distance, buf):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 41)
        self.__port.write_slong(speed)
        self.__port.write_long(distance)
        self.__port.write_byte(buf)
        self.__port.write_byte(self.__port.get_checksum())

    def buffered_m2_drive_with_signed_speed_distance(self, speed, distance, buf):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 42)
        self.__port.write_slong(speed)
        self.__port.write_long(distance)
        self.__port.write_byte(buf)
        self.__port.write_byte(self.__port.get_checksum())

    def buffered_drive_mixed_with_signed_speed_distance(self, speed1, distance1, speed2, distance2, buf):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 43)
        self.__port.write_slong(speed1)
        self.__port.write_long(distance1)
        self.__port.write_slong(speed2)
        self.__port.write_long(distance2)
        self.__port.write_byte(buf)
        self.__port.write_byte(self.__port.get_checksum())

    def buffered_m1_drive_with_signed_speed_accel_distance(self, accel, speed, distance, buf):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 44)
        self.__port.write_long(accel)
        self.__port.write_slong(speed)
        self.__port.write_long(distance)
        self.__port.write_byte(buf)
        self.__port.write_byte(self.__port.get_checksum())

    def buffered_m2_drive_with_signed_speed_accel_distance(self, accel, speed, distance, buf):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 45)
        self.__port.write_long(accel)
        self.__port.write_slong(speed)
        self.__port.write_long(distance)
        self.__port.write_byte(buf)
        self.__port.write_byte(self.__port.get_checksum())

    def drive_mixed_with_signed_speed_accel_distance(self, accel, speed1, distance1, speed2, distance2, buf):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 46)
        self.__port.write_long(accel)
        self.__port.write_slong(speed1)
        self.__port.write_long(distance1)
        self.__port.write_slong(speed2)
        self.__port.write_long(distance2)
        self.__port.write_byte(buf)
        self.__port.write_byte(self.__port.get_checksum())

    def read_buffer_length(self):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 47)
        buffer1 = self.__port.read_byte()
        buffer2 = self.__port.read_byte()
        crc = self.__port.get_checksum()
        if crc == self.__port.read_byte():
            return buffer1, buffer2
        return -1, -1

    def read_motor_currents(self):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 49)
        motor1 = self.__port.read_word()
        motor2 = self.__port.read_word()
        crc = self.__port.get_checksum()
        if crc == self.__port.read_byte():
            return motor1, motor2
        return -1, -1

    def drive_mixed_with_speed_individual_accel(self, accel1, speed1, accel2, speed2):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 50)
        self.__port.write_long(accel1)
        self.__port.write_slong(speed1)
        self.__port.write_long(accel2)
        self.__port.write_slong(speed2)
        self.__port.write_byte(self.__port.get_checksum())

    def drive_mixed_with_speed_individual_accel_distance(self,
                                                         accel1, speed1, distance1, accel2, speed2, distance2, buf):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 51)
        self.__port.write_long(accel1)
        self.__port.write_slong(speed1)
        self.__port.write_long(distance1)
        self.__port.write_long(accel2)
        self.__port.write_slong(speed2)
        self.__port.write_long(distance2)
        self.__port.write_byte(buf)
        self.__port.write_byte(self.__port.get_checksum())

    def drive_m1_with_signed_duty_accel(self, accel, duty):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 52)
        self.__port.write_sword(duty)
        self.__port.write_word(accel)
        self.__port.write_byte(self.__port.get_checksum())

    def drive_m2_with_signed_duty_accel(self, accel, duty):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 53)
        self.__port.write_sword(duty)
        self.__port.write_word(accel)
        self.__port.write_byte(self.__port.get_checksum())

    def drive_mixed_with_signed_duty_accel(self, accel1, duty1, accel2, duty2):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 54)
        self.__port.write_sword(duty1)
        self.__port.write_word(accel1)
        self.__port.write_sword(duty2)
        self.__port.write_word(accel2)
        self.__port.write_byte(self.__port.get_checksum())

    def read_m1_pidq_settings(self):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 55)
        p = self.__port.read_long()
        i = self.__port.read_long()
        d = self.__port.read_long()
        qpps = self.__port.read_long()
        crc = self.__port.get_checksum()
        if crc == self.__port.read_byte():
            return p, i, d, qpps
        return -1, -1, -1, -1

    def read_m2_pidq_settings(self):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 56)
        p = self.__port.read_long()
        i = self.__port.read_long()
        d = self.__port.read_long()
        qpps = self.__port.read_long()
        crc = self.__port.get_checksum()
        if crc == self.__port.read_byte():
            return p, i, d, qpps
        return -1, -1, -1, -1

    def set_main_battery_voltages(self, _min, _max):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 57)
        self.__port.write_word(_min)
        self.__port.write_word(_max)
        self.__port.write_byte(self.__port.get_checksum())

    def set_logic_battery_voltages(self, _min, _max):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 58)
        self.__port.write_word(_min)
        self.__port.write_word(_max)
        self.__port.write_byte(self.__port.get_checksum())

    def read_main_battery_voltage_settings(self):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 59)
        _min = self.__port.read_word()
        _max = self.__port.read_word()
        crc = self.__port.get_checksum()
        if crc == self.__port.read_byte():
            return _min, _max
        return -1, -1

    def read_logic_battery_voltage_settings(self):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 60)
        _min = self.__port.read_word()
        _max = self.__port.read_word()
        crc = self.__port.get_checksum()
        if crc == self.__port.read_byte():
            return _min, _max
        return -1, -1

    def set_m1_position_pid_constants(self, kp, ki, kd, ki_max, deadzone, _min, _max):
        self.__port.send_command(self.__rc_address, 61)
        self.__port.write_long(kp)
        self.__port.write_long(ki)
        self.__port.write_long(kd)
        self.__port.write_long(ki_max)
        self.__port.write_long(deadzone)
        self.__port.write_long(_min)
        self.__port.write_long(_max)

    def set_m2_position_pid_constants(self, kp, ki, kd, ki_max, deadzone, _min, _max):
        self.__port.send_command(self.__rc_address, 62)
        self.__port.write_long(kp)
        self.__port.write_long(ki)
        self.__port.write_long(kd)
        self.__port.write_long(ki_max)
        self.__port.write_long(deadzone)
        self.__port.write_long(_min)
        self.__port.write_long(_max)

    def read_m1_position_pid_constants(self):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 63)
        p = self.__port.read_long()
        i = self.__port.read_long()
        d = self.__port.read_long()
        i_max = self.__port.read_long()
        deadzone = self.__port.read_long()
        _min = self.__port.read_long()
        _max = self.__port.read_long()
        crc = self.__port.get_checksum()
        if crc == self.__port.read_byte():
            return p, i, d, i_max, deadzone, _min, _max
        return -1, -1, -1, -1, -1, -1, -1

    def read_m2_position_pid_constants(self):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 64)
        p = self.__port.read_long()
        i = self.__port.read_long()
        d = self.__port.read_long()
        i_max = self.__port.read_long()
        deadzone = self.__port.read_long()
        _min = self.__port.read_long()
        _max = self.__port.read_long()
        crc = self.__port.get_checksum()
        if crc == self.__port.read_byte():
            return p, i, d, i_max, deadzone, _min, _max
        return -1, -1, -1, -1, -1, -1, -1

    def drive_m1_with_signed_speed_accel_deccel_position(self, accel, speed, deccel, position, buf):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 65)
        self.__port.write_long(accel)
        self.__port.write_long(speed)
        self.__port.write_long(deccel)
        self.__port.write_long(position)
        self.__port.write_byte(buf)
        self.__port.write_byte(self.__port.get_checksum())

    def drive_m2_with_signed_speed_accel_deccel_position(self, accel, speed, deccel, position, buf):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 66)
        self.__port.write_long(accel)
        self.__port.write_long(speed)
        self.__port.write_long(deccel)
        self.__port.write_long(position)
        self.__port.write_byte(buf)
        self.__port.write_byte(self.__port.get_checksum())

    def drive_mixed_with_signed_speed_accel_deccel_position(self,
                                                            accel1, speed1, deccel1, position1,
                                                            accel2, speed2, deccel2, position2,
                                                            buf):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 67)
        self.__port.write_long(accel1)
        self.__port.write_long(speed1)
        self.__port.write_long(deccel1)
        self.__port.write_long(position1)
        self.__port.write_long(accel2)
        self.__port.write_long(speed2)
        self.__port.write_long(deccel2)
        self.__port.write_long(position2)
        self.__port.write_byte(buf)
        self.__port.write_byte(self.__port.get_checksum())

    def read_temperature(self):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 82)
        val = self.__port.read_word()
        crc = self.__port.get_checksum()
        if crc == self.__port.read_byte():
            return val
        return -1

    def read_error_state(self):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 90)
        val = self.__port.read_byte()
        crc = self.__port.get_checksum()
        if crc == self.__port.read_byte():
            return val
        return -1

    def read_encoder_mode(self):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 91)
        mode1 = self.__port.read_byte()
        mode2 = self.__port.read_byte()
        crc = self.__port.get_checksum()
        if crc == self.__port.read_byte():
            return mode1, mode2
        return -1

    def set_m1_encoder_mode(self, val):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 92)
        self.__port.write_byte(val)
        self.__port.write_byte(self.__port.get_checksum())

    def set_m2_encoder_mode(self, val):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 93)
        self.__port.write_byte(val)
        self.__port.write_byte(self.__port.get_checksum())

    def write_settings_to_eeprom(self):
        self.__port.reset_checksum()
        self.__port.send_command(self.__rc_address, 94)
        crc = self.__port.get_checksum()
        if crc == self.__port.read_byte():
            return crc
        return -1