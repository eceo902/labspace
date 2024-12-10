import asyncio
import json
import re
import sys

from pynput.keyboard import Key, Listener

from roarm import RoArmAPI


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
    
    async def video_game_control(self):
        # Get the current event loop
        loop = asyncio.get_running_loop()
        
        # Create a Future to track the current move
        current_move = None

        step_size = 1
        
        def on_press(key):
            nonlocal current_move
            # If a move is still in progress, ignore this key press
            if current_move and not current_move.done():
                print("Move in progress, ignoring key press")
                return
    
            print(f"Key pressed: {key}")
            coords = [None, None, None, None]
            nonlocal step_size
            try:
                if key.char == 'w':
                    curr_x = self.current_position[0]
                    coords[0] = curr_x + (0.01 * step_size)
                if key.char == 's':
                    curr_x = self.current_position[0]
                    coords[0] = curr_x - (0.01 * step_size)
                if key.char == 'a':
                    curr_y = self.current_position[1]
                    coords[1] = curr_y + (0.01 * step_size)
                if key.char == 'd':
                    curr_y = self.current_position[1]
                    coords[1] = curr_y - (0.01 * step_size)
                if key.char == 'z':
                    step_size = max(step_size - 0.25, 0.25)
                    print(f"Step size is now: {step_size}")
                    return
                if key.char == 'x':
                    step_size = min(step_size + 0.25, 7.5)
                    print(f"Step size is now: {step_size}")
                    return
            except AttributeError:
                # Handle special keys here
                if key == Key.up:
                    curr_z = self.current_position[2]
                    coords[2] = curr_z + (0.01 * step_size)
                if key == Key.down:
                    curr_z = self.current_position[2]
                    coords[2] = curr_z - (0.01 * step_size)
                if key == Key.left:
                    curr_t = self.current_position[3]
                    coords[3] = max(curr_t - (0.05 * step_size), 1.7)
                if key == Key.right:
                    curr_t = self.current_position[3]
                    coords[3] = min(curr_t + (0.05 * step_size), 3.2)
                if key == Key.esc:
                    return False
            print(f"Moving to {coords}")
            # Create new Future for this move
            current_move = loop.create_task(self.move(coords))

        # Create and start the listener in non-blocking way
        listener = Listener(on_press=on_press)
        listener.start()

        # Keep the async function running until ESC is pressed
        try:
            while listener.running:
                if current_move:
                    # Wait for current move to complete
                    await current_move
                    current_move = None
                await asyncio.sleep(0.01)
        finally:
            listener.stop()
            print("Exiting video game control")

    def save_position(self, name):
        try:
            with open("saved_positions.json", 'r') as f:
                saved_positions = json.load(f)
        except FileNotFoundError:
            saved_positions = {}
        saved_positions[name] = self.current_position
        with open("saved_positions.json", 'w') as f:
            json.dump(saved_positions, f)
    
    def load_position(self, name):
        with open("saved_positions.json", 'r') as f:
            saved_positions = json.load(f)
            return saved_positions[name]

    async def handle_command(self, command):
        if match := re.match(r'm\s+([xyzt]+)\s+([-\d.,\s]+)', command):
            axes = match.group(1)
            values = [float(v.strip()) for v in match.group(2).split(',')]
            coords = [None, None, None, None]
            for axis, value in zip(axes, values):
                coords['xyzt'.index(axis)] = value
            await self.move(coords)
            return True
        elif match := re.match(r'save\s+([a-zA-Z0-9_]+)', command):
            name = match.group(1)
            self.save_position(name)
            print(f"Saved position to {name}")
        elif match := re.match(r'load\s+([a-zA-Z0-9_]+)', command):
            name = match.group(1)
            loaded_position = self.load_position(name)
            await self.move(loaded_position)
        elif command.strip() == 'm':
            await self.move(self.current_position)
        elif command.strip() == 'home':
            await self.move(self.HOME)
        elif command.split()[0] == 's':
            self.current_speed = float(command.split()[1])
            print(f"Speed set to {self.current_speed}")
        elif command.strip() == 'v':
            print("In video game control mode. Use WASD and arrow keys to move. Press ZX to increase/decrease step size. Press esc to exit.")
            await self.video_game_control()
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
    print("Enter 'save <name>' to save the current position")
    print("Enter 'load <name>' to load a saved position")
    print("Enter 'v' to enter video game control mode")
    print("Enter 'q' to quit")

    while True:
        command = input("> ").strip().lower()
        should_continue = await arm_controller.handle_command(command)
        if not should_continue:
            break

    await arm_controller.close()

if __name__ == "__main__":
    asyncio.run(main())
