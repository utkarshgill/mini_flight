import os
import time
import termios
import struct

DEVICE = "/dev/tty.usbmodem0x80000001"
BAUD = 115200
CMD_RAW_IMU = 102

def msp_request(cmd:int) -> bytes:
    size = 0
    checksum = size ^ cmd
    return b"$M<" + bytes([size, cmd, checksum])

def xor_checksum(data: bytes) -> int:
    chk = 0
    for b in data: chk ^= b
    return chk

def open_serial():
    fd = os.open(DEVICE, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CREAD | termios.CLOCAL | termios.CS8
    attrs[3] = 0
    baud_const = getattr(termios, f"B{BAUD}", None)
    if hasattr(termios, "cfsetispeed"):
        termios.cfsetispeed(attrs, baud_const)
        termios.cfsetospeed(attrs, baud_const)
    else:
        attrs[4] = baud_const
        attrs[5] = baud_const

    termios.tcsetattr(fd, termios.TCSANOW, attrs)

    return fd

def parse_buffer(buf: bytearray):
    while True:
        start = buf.find(b"$M>")
        if start == -1:
            buf.clear()
            break

        if len(buf) < start + 5: break # need more data

        size = buf[start + 3]
        frame_len = 5 + size + 1
        if len(buf) < start + frame_len: break # wait for rest

        frame = bytes(buf[start:start + frame_len])
        del buf[:start + frame_len]
        yield frame

def read_frames(fd: int):
    buf = bytearray()
    while True:
        os.write(fd, msp_request(CMD_RAW_IMU))
        time.sleep(0.02)  # brief pause so the reply can arrive
        try: buf.extend(os.read(fd, 512))
        except BlockingIOError: continue
        yield from parse_buffer(buf)

def decode_raw_imu(frame: bytes):
    size = frame[3]
    cmd = frame[4]
    payload = frame[5:5+size]
    checksum = frame[5+size]

    calc = xor_checksum(frame[3:5+size]) # XOR over size, cmd, payload
    if calc != checksum: raise ValueError(f"Checksum mismatch: {calc} != {checksum}")
    return struct.unpack_from("<9h", payload) # ax, ay, az, gx, gy, gz, mx, my, mz

def repr(values):
    ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps = values
    return f"Accel {ax_g:8.4f} {ay_g:8.4f} {az_g:8.4f} | Gyro {gx_dps:8.4f} {gy_dps:8.4f} {gz_dps:8.4f}"

def measure_bias(fd, samples=200):
    acc_sum = [0, 0, 0]
    gyro_sum = [0, 0, 0]
    for _ in range(samples):
        frame = next(read_frames(fd))
        ax, ay, az, gx, gy, gz, *_ = decode_raw_imu(frame)
        acc_sum[0] += ax
        acc_sum[1] += ay
        acc_sum[2] += az
        gyro_sum[0] += gx
        gyro_sum[1] += gy
        gyro_sum[2] += gz
    acc_bias = [sum / samples for sum in acc_sum]
    gyro_bias = [sum / samples for sum in gyro_sum]
    return acc_bias, gyro_bias

ACC_LSB_PER_G = 2048.0       # ±16 g range → 2048 counts = 1 g
GYRO_LSB_PER_DPS = 16.384    # ±2000 °/s range → 16.384 counts = 1 °/s

def apply_bias_and_scale(values, acc_bias, gyro_bias):
    ax, ay, az, gx, gy, gz, *_ = values

    ax = (ax - acc_bias[0]) / ACC_LSB_PER_G
    ay = (ay - acc_bias[1]) / ACC_LSB_PER_G
    az = (az - acc_bias[2]) / ACC_LSB_PER_G

    gx = (gx - gyro_bias[0]) / GYRO_LSB_PER_DPS
    gy = (gy - gyro_bias[1]) / GYRO_LSB_PER_DPS
    gz = (gz - gyro_bias[2]) / GYRO_LSB_PER_DPS

    return ax, ay, az, gx, gy, gz

def main():
    fd = open_serial()
    try:
        print("Hold the craft still while we measure bias...")
        acc_bias, gyro_bias = measure_bias(fd, samples=200)
        print("Bias:", acc_bias, gyro_bias)

        for frame in read_frames(fd):
            raw = decode_raw_imu(frame)
            values = apply_bias_and_scale(raw, acc_bias, gyro_bias)
            print(repr(values))
    finally: os.close(fd)

if __name__ == "__main__":
    main()