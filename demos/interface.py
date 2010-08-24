import sys
from control import Robot

def getchar():
	#Returns a single character from standard input
	import tty, termios
	fd = sys.stdin.fileno()
	old_settings = termios.tcgetattr(fd)
	try:
		tty.setraw(sys.stdin.fileno())
		ch = sys.stdin.read(1)
	finally:
		termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
	return ch

def main():
	robot = Robot()

	robot.sendPacket()

	motor = 550
	steering = 590
	while True:
		c = getchar()
		if c == 'w':
			motor -= 50
			robot.setMotor(motor)
		elif c == 's':
			motor += 50
			robot.setMotor(motor)
		elif c == 'e':
			robot.setMotor(550)
		elif c == 'a':
			steering -= 100
			robot.setSteering(steering)
		elif c == 'd':
			steering += 100
			robot.setSteering(steering)
		elif c == 'q':
			print 'quit'
			robot.setSteering(550)
			robot.setMotor(550)
			break



if __name__ == '__main__':
	main()
