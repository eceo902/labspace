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
        self.writer.write(json.dumps(command).encode())
        await self.writer.drain()
        
        data = await self.reader.read(100)
        return json.loads(data.decode())
    
    async def move_joint(self, joint_id, angle):
        command = {
            "type": "move_joint",
            "joint": joint_id,
            "angle": angle
        }
        return await self.send_command(command)
    
    async def close(self):
        if self.writer:
            self.writer.close()
            await self.writer.wait_closed()
