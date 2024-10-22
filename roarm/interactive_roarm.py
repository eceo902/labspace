from roarm import RoArmAPI
import asyncio
import sys
import re

class RoArmInteractive:
    HOME = [.32, 0, .23, 3.17] # x, y, z in meters, t in radians
    def __init__(self, host):
        self.arm = RoArmAPI(host, 8659)
        self.current_position = self.HOME[:] # Initialize with default values
        self.current_speed = 0.1

    async def connect(self):
        await self.arm.connect()

    async def close(self):
        await self.arm.close()

    async def move(self, coords):
        for i, coord in enumerate(coords):
            if coord is not None:
                self.current_position[i] = coord
        result = await self.arm.move_interp(*self.current_position, speed=self.current_speed)
        print(f"Moved to {self.current_position}")
        return result

    async def handle_command(self, command):
        match = re.match(r'm\s+([xyzt]+)\s+([-\d.,\s]+)', command)
        if match:
            axes = match.group(1)
            values = [float(v.strip()) for v in match.group(2).split(',')]
            coords = [None, None, None, None]
            for axis, value in zip(axes, values):
                coords['xyzt'.index(axis)] = value
            await self.move(coords)
        elif command.strip() == 'm':
            await self.move(self.current_position)
        elif command.strip() == 'home':
            await self.move(self.HOME)
        elif command.split()[0] == 's':
            self.current_speed = float(command.split()[1])
            print(f"Speed set to {self.current_speed}")
        elif command.strip() == 'q':
            return False
        else:
            print("Invalid command. Use 'm <axes> <values>' (e.g., 'm z -.08' or 'm xy .38, -.21'), 's <speed>', 'home', or 'q'")
        return True

async def main():
    host = '10.1.20.227' if '--host' not in sys.argv else sys.argv[sys.argv.index('--host')+1]
    arm_controller = RoArmInteractive(host)
    await arm_controller.connect()

    print("RoArm Interactive Controller")
    print("Enter commands like 'm z -.08' or 'm xy .38, -.21'")
    print("Enter 'q' to quit")

    while True:
        command = input("> ").strip().lower()
        should_continue = await arm_controller.handle_command(command)
        if not should_continue:
            break

    await arm_controller.close()

if __name__ == "__main__":
    asyncio.run(main())
