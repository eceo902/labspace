import asyncio
import json

class RoArmAPI:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.reader = None
        self.writer = None
    
    async def connect(self):
        self.reader, self.writer = await asyncio.open_connection(self.host, self.port)
        
    async def send_command(self, command):
        print(f"RoArmAPI: command = {command}")
        self.writer.write(json.dumps(command).encode())
        await self.writer.drain()
        
        data = await self.reader.read(4096)
        print(f"RoArmAPI: received data = {data}")
        return json.loads(data.decode())

    async def move(self, x, y, z, grip_angle=3.14):
        command = {
            "type": "move_xyzt",
            "x": x,
            "y": y,
            "z": z,
            "grip_angle": grip_angle
        }
        return await self.send_command(command)
    
    async def move_interp(self, x, y, z, grip_angle=3.14, speed=0.25):
        command = {
            "type": "move_xyzt_interp",
            "x": x,
            "y": y,
            "z": z,
            "grip_angle": grip_angle,
            "speed": speed
        }
        return await self.send_command(command)
    
    async def close(self):
        if self.writer:
            self.writer.close()
            await self.writer.wait_closed()
