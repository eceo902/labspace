from roarm import RoArmAPI
import asyncio

async def main():
    # rpi is at 10.1.20.227
    arm = RoArmAPI("10.1.20.227", 8659)
    await arm.connect()
    
    result = await arm.move_joint(1, 45)
    print(result)
    
    await arm.close()

if __name__ == "__main__":
    asyncio.run(main())
