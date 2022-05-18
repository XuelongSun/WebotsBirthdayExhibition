from mavic_2 import Mavic2Pro

robot = Mavic2Pro('Explorer')

if __name__ == "__main__":
    while True:
        robot.my_step()
        robot.keyboard_process()
        robot.run()