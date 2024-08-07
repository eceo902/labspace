from roarm import RoArmAPI
import asyncio
import sys

async def main():
    # rpi is at 10.1.20.227
    arm = RoArmAPI("localhost", 8659)
    await arm.connect()
    
    result = await arm.move_interp((1 if 'back' in sys.argv else -1)*0.4, .1, 0)
    print(result)
    
    await arm.close()

if __name__ == "__main__":
    asyncio.run(main())
