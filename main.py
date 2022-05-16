import math
import time
# RAD2DEG = 
def trajectoryPlanning(startPoint, endPoint, n = 20):
    trajectory = []
    d = []
    for i in range(3):
    	d.append((endPoint[i]-startPoint[i]) / n)
    
    for t in range(n+1):
        x = startPoint[0] + d[0] * t
        y = startPoint[1] + d[1] * t
        z = startPoint[2] + d[2] * t
        trajectory.append([x,y,z])
    return trajectory

def inverseKinematic(trajectory):
	angles = []
	for position in trajectory:
		Xd, Yd, Zd = position[0], position[1], position[2]
		a1 = 6;
		a2 = 15;
		a3 = 14.5;

		d1 = 16.3;
		d2 = 0;
		d3 = 0;

		Theta1 = math.degrees(math.atan(Yd/Xd));

		r1 = math.sqrt((Xd - a1*math.cos(Theta1))**2 + (Yd - a1*math.sin(Theta1))**2 );
		r2 = Zd -d1;
		Phi2 = math.atan(r2/r1);

		r3 = math.sqrt(r1**2 + r2**2);
		Phi1 = math.acos((a2**2+r3**2-a3**2)/(2*a2*r3));

		Theta2 = math.degrees(math.pi/2 - (Phi1 + Phi2));


		Phi3 = math.acos((a2**2+a3**2-r3**2)/(2*a2*a3));

		Theta3 = math.degrees(math.pi - Phi3);

		angles.append([Theta1, Theta2, Theta3])

	return angles

def convert(anglesTrajectory, rate):
	angles = []
	for angle in anglesTrajectory:
		angle1 = angle[0] * (-rate)
		angle2 = angle[1] * (-rate)
		angle3 = angle[2] * (-rate)
		angles.append([angle1, angle2, angle3])
	return angles

def PID(Kp, Ki, Kd, MV_bar=0):
    # initialize stored data
    e_prev = 0
    t_prev = -100
    I = 0
    
    # initial control
    MV = MV_bar
    
    while True:
        # yield MV, wait for new t, PV, SP
        t, PV, SP = yield MV
        
        # PID calculations
        e = SP - PV
        
        P = Kp*e
        I = I + Ki*e*(t - t_prev)
        D = Kd*(e - e_prev)/(t - t_prev)
        
        MV = MV_bar + P + I + D
        
        # update stored data for next iteration
        e_prev = e
        t_prev = t 

def getAngleMotor(motor):
	return motor.position

def applyMotor(motor, MV):
	if (abs(MV) > 100):
		MV = math.copysign(1, MV) * 100
	motor.run_direct(duty_cycle_sp=int(MV))

def isStop(PV, SP, ERROR, motor):
	error = SP - PV
	if error < ERROR:
		motor.stop(stop_action='brake')
		return True
	return False
	

def writeData(file, data):
	# data include angle of motors
	string = ",".join(str(e) for e in data)
	file.write(data)

def main():
	# Planning
	start = [-5,5,5] # ????? xem tại góc 0 0 0 thì tọa độ là bao nhiêu 
	end = [-10,-10,10]
	trajectory = trajectoryPlanning(start, end)
	anglesTrajectory = inverseKinematic(trajectory)

	# Convert to angle of motors
	rate = 'đếm số răng bánh to chia số răng bánh nhỏ'
	angles = convert(anglesTrajectory, rate)

	# Init motors
	motor1 = LargeMotor('outA')
	motor2 = LargeMotor('outB')
	motor3 = LargeMotor('outC')

	# PID control constants
	KP = 5
	KI = 0.1
	KD = 2

	ERROR = 1

	# create pid control
	controller1 = PID(KP, KI, KD)         
	controller2 = PID(KP, KI, KD)        
	controller3 = PID(KP, KI, KD)       

	controller1.send(None)              # initialize
	controller2.send(None)              # initialize
	controller3.send(None)              # initialize

	# open file to record
	path = 'dataRecord.txt'
	file = open(path, 'w')
	writeData(file, [0, 0, 0]) # initialize first line (start from angle 0,0,0)

	for angle in angles:
		SP1, SP2, SP3 = angle[0], angle[1], angle[2]
		t0 =  time.time()
		while True:
			t = time.time() - t0
			# get measurement (degrees)
			PV1 = getAngleMotor(motor1) 
			PV2 = getAngleMotor(motor2)
			PV3 = getAngleMotor(motor3)

			# compute manipulated variable
			MV1 = controller1.send([t, PV1, SP1])
			MV2 = controller2.send([t, PV2, SP2])
			MV3 = controller3.send([t, PV3, SP3])

			# apply
			applyMotor(motor1, MV1)
			applyMotor(motor2, MV2)
			applyMotor(motor3, MV3)

			# Write data to file
			writeData(file, [PV1, PV2, PV3])

			# Stop motors
			status1 = isStop(PV1, SP1, ERROR, motor1)
			status2 = isStop(PV2, SP2, ERROR, motor2)
			status3 = isStop(PV3, SP3, ERROR, motor3)

			if (status1 and status2 and status3):
				break # break loop in this process
				
	# Complete trajectory
    file.close()  


if __name__ == "__main__":
	main()







