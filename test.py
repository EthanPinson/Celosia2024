import wpimath
import wpimath.controller
import math
pid = wpimath.controller.PIDController(0.5,0,0)

def getAngle(a):
    return  wpimath.angleModulus(a)                               

targetAngle = getAngle(math.pi/2)
pid.setSetpoint(targetAngle)
currentAngle = getAngle(-math.pi/2) 
print("Target Angle: " + str(targetAngle)) 
for i in range(179):
    currentAngle = getAngle(currentAngle - math.pi/180.0)
    output = pid.calculate(currentAngle)
    error = str(currentAngle - targetAngle)
    # print("Error: " + error + " Currentangle: " + str(currentAngle) + " output: " + str(output))

for i in range(719):
    #print(i * math.pi/180.0)
    print(str(getAngle(i * math.pi/180.0)))