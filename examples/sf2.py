from typing import List
import serial, struct, time, numpy as np
import math

from sim import Vector3D, Quaternion, Body, World, Renderer, GravitationalForce

class QuadCopter(Body):
    def __init__(self, position=None, velocity=None, acceleration=None, mass=1.0,
                 orientation=None, angular_velocity=None, ctrl=None, board=None):
        L = 0.3  # Length of each arm from center to tip
        num_arms = 4

        # Calculate the mass of each arm assuming equal distribution
        m_arm = mass / num_arms

        # Calculate the moments of inertia
        # Inertia about the Z-axis, assuming arms act like rods rotating around their center
        I_z = num_arms * (1/12) * m_arm * (L**2)

        # Inertia about the X and Y axes
        # Considering arms are at 45 degrees, projecting to L*cos(45 degrees) for each axis
        L_projected = L * np.cos(np.pi / 4)  # cos(45 degrees) = sqrt(2)/2
        I_x = num_arms * (1/3) * m_arm * (L_projected**2)
        I_y = I_x  # Symmetry in the configuration

        inertia = np.array([
            [I_x, 0, 0],
            [0, I_y, 0],
            [0, 0, I_z]
        ])
       
        self.ctrl = ctrl
        self.board = board

        super().__init__(position, velocity, acceleration, mass, orientation, angular_velocity, inertia)
        

    # PID stability loop

    # def update(self, dt):
    #     # Assuming self.position.v[2] is altitude, and self.orientation provides roll, pitch, and yaw directly
    #     current_altitude = self.position.v[2]
    #     current_roll, current_pitch, current_yaw = self.orientation.as_rotation_matrix()  # Adjust this method to your implementation
    #     print(current_altitude, current_roll, current_pitch, current_yaw)
        
    #     # Create a control vector from sensor data
    #     control_vector = np.array([current_altitude, current_roll, current_pitch, current_yaw])
        
    #     # Update the PID controller with this control vector
    #     T, R, Y, P = self.ctrl.update(control_vector)
    #     self.command([T, R, Y, P])
    #     super().update(dt)

    

    def update(self, dt):
        o = self.board.receive(108)
        print(o)

        qx = Quaternion.from_euler(np.radians(o['angx']), 0, 0)
        qy = Quaternion.from_euler(0, np.radians(o['angy']), 0)
        qz = Quaternion.from_euler(0, 0, np.radians(o['heading']))

        q_all = qx * qy * qz
        quad.orientation = q_all
        

        super().update(dt)

    def command(self, c):
        T, R, Y, P = c

        rotation_matrix = self.orientation.as_rotation_matrix()
        world_torque = rotation_matrix @ np.array([-P, -R, Y])
        world_thrust = rotation_matrix @ np.array([0, 0, T])

        self.apply_torque(Vector3D(*world_torque))
        self.apply_force(Vector3D(*world_thrust))

    def __repr__(self):
        return (f"QuadCopter(position={self.position}, velocity={self.velocity}, "
                f"acceleration={self.acceleration}, mass={self.mass}, "
                f"orientation={self.orientation}, motor_speeds={self.motor_speeds})")

class MSP:

    COMMANDS = {
        'IDENT': 100, 'STATUS': 101, 'RAW_IMU': 102, 'SERVO': 103, 'MOTOR': 104, 
        'RC': 105, 'RAW_GPS': 106, 'COMP_GPS': 107, 'ATTITUDE': 108, 'ALTITUDE': 109, 
        'ANALOG': 110, 'RC_TUNING': 111, 'PID': 112, 'BOX': 113, 'MISC': 114, 
        'MOTOR_PINS': 115, 'BOXNAMES': 116, 'PIDNAMES': 117, 'WP': 118, 'BOXIDS': 119, 
        'RC_RAW_IMU': 121, 'SET_RAW_RC': 200, 'SET_RAW_GPS': 201, 'SET_PID': 202, 
        'SET_BOX': 203, 'SET_RC_TUNING': 204, 'ACC_CALIBRATION': 205, 'MAG_CALIBRATION': 206, 
        'SET_MISC': 207, 'RESET_CONF': 208, 'SET_WP': 209, 'SWITCH_RC_SERIAL': 210, 
        'IS_SERIAL': 211, 'DEBUG': 254, 'VTX_CONFIG': 88, 'VTX_SET_CONFIG': 89, 
        'EEPROM_WRITE': 250, 'REBOOT': 68
    }

    def __init__(self, port, baudrate=115200):
        self.PIDcoef = {'rp': 0, 'ri': 0, 'rd': 0, 'pp': 0, 'pi': 0, 'pd': 0, 'yp': 0, 'yi': 0, 'yd': 0}
        self.rcChannels = {'roll': 0, 'pitch': 0, 'yaw': 0, 'throttle': 0, 'elapsed': 0, 'timestamp': 0}
        self.rawIMU = {'ax': 0, 'ay': 0, 'az': 0, 'gx': 0, 'gy': 0, 'gz': 0, 'mx': 0, 'my': 0, 'mz': 0, 'elapsed': 0, 'timestamp': 0}
        self.motor = {'m1': 0, 'm2': 0, 'm3': 0, 'm4': 0, 'elapsed': 0, 'timestamp': 0}
        self.attitude = {'angx': 0, 'angy': 0, 'heading': 0, 'elapsed': 0, 'timestamp': 0}
        self.altitude = {'estalt': 0, 'vario': 0, 'elapsed': 0, 'timestamp': 0}
        self.message = {'angx': 0, 'angy': 0, 'heading': 0, 'roll': 0, 'pitch': 0, 'yaw': 0, 'throttle': 0, 'elapsed': 0, 'timestamp': 0}
        
        self.elapsed = 0
        self.PRINT = 1

        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = baudrate

        self._wake_up_board()

    def _wake_up_board(self):
        wakeup = 2
        try:
            self.ser.open()
            if self.PRINT:
                print(f"Waking up board on {self.ser.port}...")
            for i in range(1, wakeup):
                if self.PRINT:
                    print('wakeup', wakeup - i)
                    time.sleep(1)
                else:
                    time.sleep(1)
        except Exception as error:
            print(f"\n\nError opening {self.ser.port} port.\n{str(error)}\n\n")

    def send(self, data_length, code, data, data_format):
        checksum = 0
        total_data = ['$'.encode('utf-8'), 'M'.encode('utf-8'), '<'.encode('utf-8'), data_length, code] + data
        for i in struct.pack('<2B' + data_format, *total_data[3:]):
            checksum ^= i
        total_data.append(checksum)
        try:
            self.ser.write(struct.pack('<3c2B' + data_format + 'B', *total_data))
        except Exception as error:
            print("\n\nError in sendCMD.")
            print(f"({str(error)})\n\n")

    def receive(self, cmd):
        try:
            start = time.time()
            self.send(0, cmd, [], '')
            header = self._wait_for_header()
            datalength, code, data = self._read_data(header)
            self._reset_buffers()

            elapsed = time.time() - start
            temp = struct.unpack('<' + 'h' * (datalength // 2), data)
            return self._process_received_data(cmd, temp, elapsed)
        except Exception as error:
            print(error)

    def _wait_for_header(self):
        while True:
            header = self.ser.read().decode('utf-8')
            if header == '$':
                return header + self.ser.read(2).decode('utf-8')

    def _read_data(self, header):
        datalength = struct.unpack('<b', self.ser.read())[0]
        code = struct.unpack('<b', self.ser.read())[0]
        data = self.ser.read(datalength)
        return datalength, code, data

    def _reset_buffers(self):
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def _process_received_data(self, cmd, temp, elapsed):
        if cmd == MSP.COMMANDS['ATTITUDE']:
            self.attitude.update({
                'angx': float(temp[0] / 10),
                'angy': float(temp[1] / 10),
                'heading': float(temp[2]),
                'elapsed': round(elapsed, 3),
                'timestamp': f"{time.time():0.2f}"
            })
            return self.attitude
        elif cmd == MSP.COMMANDS['ALTITUDE']:
            self.altitude.update({
                'estalt': float(temp[0]),
                'vario': float(temp[1]),
                'elapsed': round(elapsed, 3),
                'timestamp': f"{time.time():0.2f}"
            })
            return self.altitude
        elif cmd == MSP.COMMANDS['RC']:
            self.rcChannels.update({
                'roll': temp[0], 'pitch': temp[1], 'yaw': temp[2], 'throttle': temp[3],
                'elapsed': round(elapsed, 3),
                'timestamp': f"{time.time():0.2f}"
            })
            return self.rcChannels
        elif cmd == MSP.COMMANDS['RAW_IMU']:
            self.rawIMU.update({
                'ax': float(temp[0]), 'ay': float(temp[1]), 'az': float(temp[2]),
                'gx': float(temp[3]), 'gy': float(temp[4]), 'gz': float(temp[5]),
                'mx': float(temp[6]), 'my': float(temp[7]), 'mz': float(temp[8]),
                'elapsed': round(elapsed, 3),
                'timestamp': f"{time.time():0.2f}"
            })
            return self.rawIMU
        elif cmd == MSP.COMMANDS['MOTOR']:
            self.motor.update({
                'm1': float(temp[0]), 'm2': float(temp[1]), 'm3': float(temp[2]), 'm4': float(temp[3]),
                'elapsed': f"{elapsed:0.3f}",
                'timestamp': f"{time.time():0.2f}"
            })
            return self.motor
        elif cmd == MSP.COMMANDS['PID']:
            dataPID = [t % 256 for t in temp] + [t // 256 for t in temp]
            for p in range(0, 9, 3):
                dataPID[p] /= 10.0
                dataPID[p + 1] /= 1000.0
            self.PIDcoef.update({
                'rp': dataPID[0], 'ri': dataPID[1], 'rd': dataPID[2],
                'pp': dataPID[3], 'pi': dataPID[4], 'pd': dataPID[5],
                'yp': dataPID[6], 'yi': dataPID[7], 'yd': dataPID[8]
            })
            return self.PIDcoef
        else:
            return "No return error!"

port = "/dev/tty.usbmodem0x80000001"
baudrate = 115200
board = MSP(port, baudrate)

board.send(0, 205, [], '')

world = World()
quad = QuadCopter(position=Vector3D(0, 0, 10.0), mass=1.0, board=board)
world.add_object(quad)

r = Renderer(world)

r.run(1000)

