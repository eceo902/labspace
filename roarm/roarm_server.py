import asyncio
import json
import serial

class RoArmServer:
    def __init__(self, host, port, serial_port, baud_rate):
        self.host = host
        self.port = port
        self.serial = serial.Serial(serial_port, baud_rate)

    async def handle_client(self, reader, writer):
        while True:
            data = await reader.read(100)
            if not data:
                break

            message = json.loads(data.decode())
            print(f"Received: {message}")

            response = await self.process_command(message)

            writer.write(json.dumps(response).encode())
            await writer.drain()

        writer.close()

    async def process_command(self, command):
        if command['type'] == 'move_joint':
            return await self.move_joint(command['joint'], command['angle'])
        # Add more command types as needed
        return {"status": "error", "message": "Unknown command"}

    async def move_joint(self, joint_id, angle):
        # Convert the command to the format expected by the arm
        arm_command = json.dumps({
            'T': 3,
            f'P{joint_id}': self.angle_to_position(angle),
            f'S{joint_id}': 0,  # Speed, adjust as needed
            f'A{joint_id}': 60  # Acceleration, adjust as needed
        })
        
        self.serial.write(arm_command.encode())
        # You might need to add a small delay here depending on the arm's response time
        # await asyncio.sleep(0.1)
        
        # Read response from the arm if it provides one
        # response = self.serial.readline().decode().strip()
        
        return {"status": "success", "message": f"Moved joint {joint_id} to {angle} degrees"}

    def angle_to_position(self, angle):
        # Convert angle to position value expected by the arm
        # This is a placeholder - you'll need to implement the actual conversion
        return int(2047 + (angle / 180) * 2048)

    async def run(self):
        server = await asyncio.start_server(
            self.handle_client, self.host, self.port)

        addr = server.sockets[0].getsockname()
        print(f'Serving on {addr}')

        async with server:
            await server.serve_forever()

if __name__ == "__main__":
    roarm_server = RoArmServer('0.0.0.0', 8659, '/dev/ttyUSB0', 115200)
    asyncio.run(roarm_server.run())
