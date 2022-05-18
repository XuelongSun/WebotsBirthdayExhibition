from Darwin import DarwinOP

from collections import deque

robot = DarwinOP('Explorer')

if __name__ == "__main__":
    # initialize the robot
    robot.initialize()

    # ready to walk
    robot.ready_to_walk()

    if not robot.is_walking:
        robot.gait_manager.start()
        robot.is_walking = True
    else:
        robot.gait_manager.stop()
        robot.is_walking = False
    while True:
        # check if fallen
        fallen = robot.check_if_fallen()
        if fallen == 'Fallen face up':
            robot.motion_manager.playPage(10)
            robot.wait(200)
            robot.ready_to_walk()

        elif fallen == 'Fallen face down':
            robot.motion_manager.playPage(11)
            robot.wait(200)
            robot.ready_to_walk()
        
        robot.keyboard_process()
        robot.move_to_destination()
        robot.walk_step()
        robot.my_step()
