"""helicopter_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from cmath import inf
from controller import Robot

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
upper_motor = robot.getDevice('upper_motor')
lower_motor = robot.getDevice('lower_motor')
tail_motor = robot.getDevice('tail_motor')

gps = robot.getDevice("gps")
inertial_unit = robot.getDevice('inertial unit')
gps.enable(timestep)
inertial_unit.enable(timestep)

upper_motor.setPosition(inf)
lower_motor.setPosition(inf)
tail_motor.setPosition(inf)

upper_motor.setVelocity(0)
lower_motor.setVelocity(0)
tail_motor.setVelocity(0)

#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)
started = False
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:

    ctl_msg = robot.getCustomData()
    if ctl_msg == 'start':
        if started:
            altitude = gps.getValues()[1]
            ratio = 1.0 - altitude / 2.3
            velocity = 100 + ratio
            upper_motor.setVelocity(-velocity)
            lower_motor.setVelocity(velocity)
        else:
            upper_motor.setVelocity(-100)
            lower_motor.setVelocity(100)
            tail_motor.setVelocity(15)
            started = True
    elif ctl_msg == 'hover':
        upper_motor.setVelocity(-100)
        lower_motor.setVelocity(100)
        tail_motor.setVelocity(0)
        # altitude = gps.getValues()[1]
    #     pitch =  inertial_unit.getRollPitchYaw()[1]
    #     tail_motor.setVelocity(20 - pitch*1e4)
    #     # print(pitch)
    #     ratio = 1.0 - altitude / 1.0
    #     velocity = 100 + ratio
    #     upper_motor.setVelocity(-velocity)
    #     lower_motor.setVelocity(velocity)
    else:
        tail_motor.setVelocity(0)
        velocity = 90
        upper_motor.setVelocity(-velocity)
        lower_motor.setVelocity(velocity)
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)

# Enter here exit cleanup code.
