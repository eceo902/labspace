from roarm import RoArmAPI
import nestbox
import asyncio
import numpy as np
import time

# whether a point is within a working area
# distance of .5 of the origin but no closer than .2 and has a z value greater than 0 and a y value greater than -.2
def in_bounds(point):
    assert len(point) == 3, "point must be a 3-tuple"
    return .2 < np.linalg.norm(point) < .5 and point[1] > -.2 and point[2] > 0

async def catch_up():
    await asyncio.sleep(7)

#generator for points waving back and forth inside the working area
def wave_points():
    start_time = time.time()
    center = np.array((.3, 0, .3))
    period = 5
    while True:
        dtime = time.time() - start_time
        yield center + np.array((0, .1*np.cos(dtime/period*3.14*2), .1*np.sin(dtime/period*3.14*2)))

async def main():
    # rpi is at 10.1.20.227
    arm = RoArmAPI("10.1.20.227", 8659)
    await arm.connect()
    
    point = np.array((.3, .1, .3))
    last_point = point
    result = await arm.move_interp(*point, speed=.1)
    print(result)

    await catch_up()

    waver = wave_points()

    try:
        while True:
            # point = nestbox.from_cs('cs2').to_cs('cs1').convert((0, 0, 0))
            point = next(waver)
            print(point)
            if in_bounds(point):
                # if it didn't go too far, move to the point slowly; otherwise, move to the point directly
                if np.linalg.norm(point - last_point) > .1:
                    result = await arm.move_interp(point[0], point[1], point[2], speed=.1)
                    await catch_up()
                else:
                    result = await arm.move(point[0], point[1], point[2])
                last_point = point
            else:
                print(f"Point {point} is out of bounds")
    finally:
        await arm.close()

if __name__ == "__main__":
    asyncio.run(main())
