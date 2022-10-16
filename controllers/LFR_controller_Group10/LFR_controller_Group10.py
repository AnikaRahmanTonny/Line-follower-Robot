from controller import Robot

robot = Robot()

#get the time step of the current world. 
timestep = int(robot.getBasicTimeStep())

time_step = 64

speed = 4.5


wheels = []
wheelsNames = ['wheel1', 'wheel2', 'wheel3', 'wheel4']
for i in range(4):
    wheels.append(robot.getDevice(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)


right_ir = robot.getDevice('RIGHT')

right_ir.enable(time_step)

mid_ir = robot.getDevice('MID')

mid_ir.enable(time_step)

left_ir = robot.getDevice('LEFT')

left_ir.enable(time_step)


last_error = intg = diff = prop = weight_counter = 0
kp = 0.005
ki = 0
kd = 0.15

def pid(error):
    global last_error, intg, diff, prop, kp, ki, kd
    prop = error
    intg = error + intg
    diff = error - last_error
    balance = (kp * prop) + (ki * intg) + (kd * diff)
    last_error = error
    if balance > 5:
        balance = balance % 5
    # elif balance < 0:
        # balance = balance % 5
        # balance = balance * (-1) 
    return balance

def set_speed (speed, balance):
    wheels[0].setVelocity(speed + balance)
    wheels[1].setVelocity(speed - balance)
    
    wheels[2].setVelocity(speed + balance)
    wheels[3].setVelocity(speed - balance)



while robot.step(timestep) != -1:

    
    right_ir_val = right_ir.getValue()
    mid_ir_val = mid_ir.getValue()
    left_ir_val = left_ir.getValue()
    
    
    
    print("left: {} mid: {} right: {}".format(left_ir_val, mid_ir_val, right_ir_val))
    

    threshold = 400
    distance = 800

    if right_ir_val < threshold and left_ir_val < threshold and mid_ir_val >= threshold:
        error = 0
        balance = pid(error)
        set_speed(speed, balance)
    
    #For right turn 
    if  right_ir_val >= threshold and left_ir_val < threshold and mid_ir_val >= threshold:
        error = distance
        balance = pid(error)
        set_speed(speed, balance)
    
    #For left turn
    if right_ir_val < threshold and left_ir_val >= threshold and mid_ir_val >= threshold:
        error = -distance
        balance = pid(error)
        set_speed(speed, balance)
    
    #For left turn
    if right_ir_val < threshold and left_ir_val >= threshold and mid_ir_val < threshold:
        error = -distance
        balance = pid(error)
        set_speed(speed, balance)
    
    #For Right turn
    if right_ir_val >= threshold and left_ir_val < threshold and mid_ir_val < threshold:
        error = distance
        balance = pid(error)
        set_speed(speed, balance)
    
    #For rotation
    if right_ir_val < threshold and left_ir_val < threshold and mid_ir_val < threshold:
        error = 0
        balance = pid(error)
        set_speed(-speed, balance)
        
