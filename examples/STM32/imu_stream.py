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

        if len(buf) < start + 5:
            break # need more data

        size = buf[start + 3]
        frame_len = 5 + size + 1
        if len(buf) < start + frame_len:
            break # wait for rest

        frame = bytes(buf[start:start + frame_len])
        del buf[:start + frame_len]

        yield frame

def read_frames(fd: int):
    buf = bytearray()
    while True:
        os.write(fd, msp_request(CMD_RAW_IMU))
        time.sleep(0.02)  # brief pause so the reply can arrive
        try:
            buf.extend(os.read(fd, 512))
        except BlockingIOError:
            continue
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
    ax, ay, az, gx, gy, gz, mx, my, mz = values
    return f"Accel {ax:6d} {ay:6d} {az:6d} | Gyro {gx:6d} {gy:6d} {gz:6d} | Mag {mx:6d} {my:6d} {mz:6d}"

def main():
    fd = open_serial()
    try: 
        for frame in read_frames(fd):
            values = decode_raw_imu(frame)
            print(repr(values))
    finally:
        os.close(fd)

if __name__ == "__main__":
    main()