import asyncio
import json
from json import JSONDecodeError
import serial
import time
import asyncio
import json
import serial
import threading

class RoArmServer:
    CMD_XYZT_DIRECT_CTRL = 1041
    CMD_XYZT_GOAL_CTRL = 104

    def __init__(self, host, port, serial_port, baud_rate):
        self.host = host
        self.port = port
        self.serial = serial.Serial(serial_port, baudrate=baud_rate, dsrdtr=None, timeout=.1)
        self.serial.setRTS(False)
        self.serial.setDTR(False)

        # Start the serial reading thread
        self.serial_thread = threading.Thread(target=self.read_serial)
        self.serial_thread.daemon = True
        self.serial_lock = threading.RLock()
        #self.serial_thread.start()

    def read_serial(self):
        while True:
            with self.serial_lock:
                data = self.serial.readline().decode('utf-8')
                if data:
                    print(f"Received from arm: {data}", end='')
            time.sleep(0.1)

    async def handle_client(self, reader, writer):
        while True:
            data = await reader.read(100)
            if not data:
                break

            message = json.loads(data.decode())
            print(f"Received from client: {message}")

            with self.serial_lock:
                response = await self.process_command(message)

            writer.write(json.dumps(response).encode())
            await writer.drain()

        writer.close()

    async def process_command(self, command):
        if command['type'] == 'move_xyzt_interp':
            return await self.move_xyzt_interp(command['x'], command['y'], command['z'], command['grip_angle'], command['speed'])
        elif command['type'] == 'move_xyzt':
            return await self.move_xyzt(command['x'], command['y'], command['z'], command['grip_angle'])
        return {"status": "error", "message": "Unknown command"}

    async def move_xyzt_interp(self, x, y, z, t, speed):
        # Convert meters to millimeters
        x_mm = x * 1000
        y_mm = y * 1000
        z_mm = z * 1000

        arm_command = json.dumps({
            "T": self.CMD_XYZT_GOAL_CTRL,
            "x": x_mm,
            "y": y_mm,
            "z": z_mm,
            "t": t,
            "spd":speed
        })
        
        self.serial.write(arm_command.encode() + b'\n')
        
        # Wait for a short time to allow the arm to process the command
        await asyncio.sleep(0.1)

        data = ''
        for _ in range(4):
            data += self.serial.readline().decode('utf-8').strip()
        print(f"Received from arm: {data}", end='')

        print(f'sending response: {{"status": "success", "message": "Moved arm to x:{x}m, y:{y}m, z:{z}m, t:{t}rad", "response": {data}}}')
        return {"status": "success", "message": f"Moved arm to x:{x}m, y:{y}m, z:{z}m, t:{t}rad", "device_message": data}

    async def move_xyzt(self, x, y, z, t):
        # Convert meters to millimeters
        x_mm = x * 1000
        y_mm = y * 1000
        z_mm = z * 1000

        arm_command = json.dumps({
            "T": self.CMD_XYZT_DIRECT_CTRL,
            "x": x_mm,
            "y": y_mm,
            "z": z_mm,
            "t": t,
            "spd":0.25
        })
        
        self.serial.write(arm_command.encode() + b'\n')
        
        # Wait for a short time to allow the arm to process the command
        await asyncio.sleep(0.1)
        
        return {"status": "success", "message": f"Moved arm to x:{x}m, y:{y}m, z:{z}m, t:{t}rad"}

    async def run(self):
        server = await asyncio.start_server(
            self.handle_client, self.host, self.port)

        addr = server.sockets[0].getsockname()
        print(f'Serving on {addr}')

        async with server:
            await server.serve_forever()

if __name__ == "__main__":
    roarm_server = RoArmServer('localhost', 8659, '/dev/tty.usbserial-10', 115200)
    asyncio.run(roarm_server.run())

# {"T":1041,"x":235,"y":0,"z":234,"t":3.14} <-- this command works to move the arm to a specific position (in mm) and the gripper to a specific angle (in radians)

# ============================== below is the official serial demo from roarm ==============================

# import serial
# import argparse
# import threading

# def read_serial():
#     while True:
#         data = ser.readline().decode('utf-8')
#         if data:
#             print(f"Received: {data}", end='')

# def main():
#     global ser
#     parser = argparse.ArgumentParser(description='Serial JSON Communication')
#     parser.add_argument('port', type=str, help='Serial port name (e.g., COM1 or /dev/ttyUSB0)')

#     args = parser.parse_args()

#     ser = serial.Serial(args.port, baudrate=115200, dsrdtr=None)
#     ser.setRTS(False)
#     ser.setDTR(False)

#     serial_recv_thread = threading.Thread(target=read_serial)
#     serial_recv_thread.daemon = True
#     serial_recv_thread.start()

#     try:
#         while True:
#             command = input("")
#             ser.write(command.encode() + b'\n')
#     except KeyboardInterrupt:
#         pass
#     finally:
#         ser.close()


# if __name__ == "__main__":
#     main()