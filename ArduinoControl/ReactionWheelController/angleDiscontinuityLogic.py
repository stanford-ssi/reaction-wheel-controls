import numpy as np
import math
import time

sign = lambda x: math.copysign(1, x)

if __name__ == "__main__":
	while True:
		currentAngle = float(input("\ncurrentAngle: "))
		delta = float(input("angle delta: "))

		originalSetpoint = currentAngle + delta
		print(originalSetpoint)

		oppSetpoint = originalSetpoint + sign(originalSetpoint) * -360
		print(oppSetpoint)

		error = delta

		kP = 0.1
		kD = 0.1
		speed = 0
		kS = 0.005

		while (abs(error) > 0.01):
			e1 = originalSetpoint - currentAngle
			e2 = oppSetpoint - currentAngle
			newError = e1 if (abs(e1) < abs(e2)) else e2
			print("Current Angle: " + str(currentAngle))
			print("Error: " + str(error))
			print()

			speed = error - newError
			error = newError

			currentAngle += speed * kS + kP * error - kD * speed

			if currentAngle > 180:
				currentAngle -= 360
			elif currentAngle <= -180:
				currentAngle += 360

			time.sleep(0.05)
