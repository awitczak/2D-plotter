import RPi.GPIO as GPIO
import threading
import queue
from time import sleep
import sys

# limit switches
LIM_SW_X = 27
LIM_SW_Y = 17
LIM_SW_Z = 22

# stepper motors
DIR_X = 14
STEP_X = 15

DIR_Y = 5
STEP_Y = 6

DIR_Y2 = 13
STEP_Y2 = 19

DIR_Z = 23
STEP_Z = 24

CW = 1 # clockwise rotation
CCW = 0  # counterclockwise rotation
steps_per_rev = 200 # Steps/revolution for NEMA17 | 360deg / 1.8deg
XY_delay = .005/2 # .005s -> 1 rev/s  | Delay for X and Y axes
Z_delay = .005 * 4 # Delay for Z axis
d = 8 / 200 # 8mm is the step of the screw, 200 is the n of steps for a full rotation
alpha = 360 / steps_per_rev # minimal angle based on the steps

plotter_running = True

motorX_running = False
motorY_running = False
motorZ_running = False

debug_cnt1 = 0
debug_cnt2 = 0
debug_cnt3 = 0

lim_X_pressed = False
lim_Y_pressed = False
lim_Z_pressed = False

initial_X = 0
initial_Y = 0
initial_Z = 0

current_X = 0
current_Y = 0
current_Z = 0

target_X = 0
target_Y = 0
target_Z = 0

target_X_reached = False
target_Y_reached = False
target_Z_reached = False

start_homing_X = False
start_homing_Y = False
start_homing_Z = False
homing_started = False

  
def commandListener(commandQueueHandle : queue):
    global commandListener_running
    while commandListener_running:
        cmd = input()
        print("CL>> {}".format(cmd))
        commandQueueHandle.put(cmd)
        
        if cmd is "q":
            break
        
        sleep(.5)
        
    print("commandListener finished operation.")
        
def commandHandler(commandQueueHandle : queue):  
    global start_homing_X, start_homing_Y, start_homing_Z

    global commandHandler_running
    while commandHandler_running:
        user_input = commandQueueHandle.get()

        if user_input is not None:
            cmds = user_input.split(':')

            if len(cmds) == 2:
                cmd = cmds[0]
                val = float(cmds[1])
            elif len(cmds) == 3:
                cmd = cmds[0]
                val = float(cmds[1])
                val2 = float(cmds[2])
            else:
                cmd = user_input
                print(cmd)

            print("CH>> {}".format(user_input))
            
            if cmd == "buzz_x_cw":
                print(">> Buzzing stepper X")
                setStepperDirection(DIR_X, CW)
                update_stepper(STEP_X, XY_delay)
            elif cmd == "buzz_y_cw":
                print(">> Buzzing stepper Y")
                setStepperDirection(DIR_Y, CW)
                update_stepper(STEP_Y, XY_delay)
            elif cmd == "buzz_y2_cw":
                print(">> Buzzing stepper Y2")
                setStepperDirection(DIR_Y2, CW)
                update_stepper(STEP_Y2, XY_delay)
            elif cmd == "buzz_z_cw":
                print(">> Buzzing stepper Z")
                setStepperDirection(DIR_Z, CW)
                update_stepper(STEP_Z, Z_delay)
            elif cmd == "buzz_x_ccw":
                print(">> Buzzing stepper X")
                setStepperDirection(DIR_X, CCW)
                update_stepper(STEP_X, XY_delay)
            elif cmd == "buzz_y_ccw":
                print(">> Buzzing stepper Y")
                setStepperDirection(DIR_Y, CCW)
                update_stepper(STEP_Y, XY_delay)
            elif cmd == "buzz_y2_ccw":
                print(">> Buzzing stepper Y2")
                setStepperDirection(DIR_Y2, CCW)
                update_stepper(STEP_Y1, XY_delay)
            elif cmd == "buzz_z_ccw":
                print(">> Buzzing stepper Z")
                setStepperDirection(DIR_Z, CCW)
                update_stepper(STEP_Z, Z_delay)
            elif cmd == "set_target_X":
                setPositionStepperX(val)
                print(">> Setting target X position to: {}".format(val))
            elif cmd == "set_target_Y":
                setPositionStepperY(val)
                print(">> Setting target Y position to: {}".format(val))
            elif cmd == "set_target_Z":
                setPositionStepperZ(val)
                print(">> Setting target Z position to: {}".format(val))
            elif cmd == "set_pos":
                print(">> Setting target XY position to: X{} Y{}".format(val, val2))
                setPositionXY(val, val2)
            elif cmd == "reset_X":
                resetPositionX()
            elif cmd == "reset_Y":
                resetPositionY()
            elif cmd == "reset_Z":
                resetPositionZ()
            elif cmd == "home_X":
                start_homing_X = True
            elif cmd == "home_Y":
                start_homing_Y = True
            elif cmd == "home_Z":
                start_homing_Z = True
            elif cmd == "q":
                global plotter_running
                plotter_running = False
            else:
                print(">> Unknown command!")
        sleep(.5)
        
    print("commandHandler finished operation.")
        
def motorX_Handler(motorX_QueueHandle : queue):
    global initial_X
    global current_X
    global target_X
    global motorX_Handler_running
    global start_homing_X
    global target_X_reached

    cnt = 0
    delay = 0
    while motorX_Handler_running:

        if not motorX_QueueHandle.empty() and target_X_reached:
            
            if target_Y_reached:
                
                target_X_reached = False
                initial_X = current_X
                target_X = motorX_QueueHandle.get()
                

            # print(target_X)

        if start_homing_X:
            home_X()

        if round(abs(target_X - current_X), 4) >= d:
            if (abs(target_X - current_X) < abs(target_Y - current_Y)):
                delay = calculate_scaled_dt()
            else: 
                delay = XY_delay

            # print("X delay: {}[s]".format(delay))

            vel = getVelocity(delay)

            if target_X > current_X:            # if the target is greater than current position, spin CW
                setStepperDirection(DIR_X, CW)
                current_X += delay * vel
            elif target_X < current_X:          # if the target is lower than current position, spin CCW
                current_X -= delay * vel
                setStepperDirection(DIR_X, CCW)

            cnt += 1
            update_stepper(STEP_X, delay)
            print("Current X: {} | Target X: {}".format(current_X, target_X))

            # print("Cnt_X: {}".format(cnt))
        else:
            target_X_reached = True
            sleep(0.05)
      
    print("motorX_Handler finished operation.")

def motorY_Handler(motorY_QueueHandle : queue):
    global initial_Y
    global current_Y
    global target_Y
    global motorY_Handler_running
    global start_homing_Y
    global target_Y_reached

    cnt = 0
    delay = 0
    while motorY_Handler_running:

        if not motorY_QueueHandle.empty() and target_Y_reached:
            if target_X_reached:
                
                target_Y_reached = False
                initial_Y = current_Y
                target_Y = motorY_QueueHandle.get()

            # print(target_Y)

        if start_homing_Y:
            home_Y()

        if round(abs(target_Y - current_Y), 4) >= d:
            if (abs(target_X - current_X) > abs(target_Y - current_Y)):
                delay = calculate_scaled_dt()
            else: 
                delay = XY_delay

            # print("Y delay: {}[s]".format(delay))

            vel = getVelocity(delay)

            if target_Y > current_Y:            # if the target is greater than current position, spin CW
                setStepperDirection(DIR_Y, CW)
                setStepperDirection(DIR_Y2, CW)
                current_Y += delay * vel
            elif target_Y < current_Y:          # if the target is lower than current position, spin CCW
                current_Y -= delay * vel
                setStepperDirection(DIR_Y, CCW)
                setStepperDirection(DIR_Y2, CCW)
    
            cnt += 1
            update_steppers(STEP_Y, STEP_Y2, delay)
            print("Current Y: {} | Target Y: {}".format(current_Y, target_Y))

            # print("Cnt_Y: {}".format(cnt))

            sleep(0)
        else: 
            target_Y_reached = True
            sleep(0.05)

    print("motorY_Handler finished operation.")

def motorZ_Handler(motorZ_QueueHandle : queue):
    global initial_Z
    global current_Z
    global target_Z
    global motorZ_Handler_running
    global start_homing_Z
    global target_Z_reached

    delay = 0
    while motorZ_Handler_running:
    
        if not motorZ_QueueHandle.empty() and target_Z_reached:
                
                target_Z_reached = False
                initial_Z = current_Z
                target_Z = motorZ_QueueHandle.get()



        if start_homing_Z:
            home_Z()
        
        # if target_Z == "1.0":
        #     target_Z = 60
        # else:
        #     target_Z = 0
        
        print(target_Z)
        if round(abs(target_Z - current_Z), 4) >= alpha:
            ang_vel = getAngularVelocity(Z_delay)

            if target_Z > current_Z:            # if the target is greater than current position, spin CW
                setStepperDirection(DIR_Z, CW)
                current_Z += Z_delay * ang_vel
            elif target_Z < current_Z:          # if the target is lower than current position, spin CCW
                current_Z -= Z_delay * ang_vel
                setStepperDirection(DIR_Z, CCW)
    
            update_stepper(STEP_Z, Z_delay)
            # print("Current: {} | Target: {}".format(round(current_Z, 4), target_Z))

            sleep(0)
        else:
            target_Z_reached = True
            sleep(0.05)

    print("motorZ_Handler finished operation.")

def positionHandler(motorX_QueueHandle : queue, motorY_QueueHandle : queue, motorZ_QueueHandle : queue, coordinates : list):
    global positionHandler_running
    while positionHandler_running:

        for xyz in coordinates:
            if not motorX_QueueHandle.full():
                x = xyz[0]
                motorX_QueueHandle.put(x)
            if not motorY_QueueHandle.full():
                y = xyz[1]
                motorY_QueueHandle.put(y)
            if not motorZ_QueueHandle.full():
                z = xyz[2]
                motorZ_QueueHandle.put(z)
            else:
                sleep(0.5)

        positionHandler_running = False
        sleep(0.05)

    print("positionHandler finished operation.")

def setPositionStepperX(target:float):
    global target_X
    target_X = target

def setPositionStepperY(target:float):
    global target_Y
    target_Y = target

def setPositionStepperZ(target:float):
    global target_Z
    target_Z = target

def setPositionXY(pos_X:float, pos_Y:float):
    global target_X
    global target_Y
    target_X = pos_X
    target_Y = pos_Y

    global initial_X
    global initial_Y
    global current_X
    global current_Y
    initial_X = current_X
    initial_Y = current_Y

def resetPositionX():
    global target_X, current_X, initial_X
    target_X = 0
    current_X = 0
    initial_X = 0

def resetPositionY():
    global target_Y, current_Y, initial_Y
    target_Y = 0
    current_Y = 0
    initial_Y = 0

def resetPositionZ():
    global target_Z, current_Z, initial_Z
    target_Z = 0
    current_Z = 0
    initial_Z = 0

def calculate_scaled_dt():
    global target_X
    global target_Y

    global initial_X
    global initial_Y

    D = 0 # longer distance
    d = 0 # shorter distance

    if (abs(target_X - initial_X) > abs(target_Y - initial_Y)):
        D = abs(target_X - initial_X)
        d = abs(target_Y - initial_Y)
    else:
        d = abs(target_X - initial_X)
        D = abs(target_Y - initial_Y)

    ratio = D / d
    scaled_dt = ratio * XY_delay

    return round(scaled_dt, 4)

def setStepperDirection(dir_pin:int, direction:bool):
    GPIO.output(dir_pin, direction)

def getVelocity(dt:float) -> float:
    global d
    velocity = d / dt
    # print("Velocity: {} [mm/s]".format(velocity))
    return velocity

def getAngularVelocity(dt:float) -> float:
    angular_velocity = 360 / steps_per_rev / dt
    # print("Angular Velocity: {} [deg/s]".format(angular_velocity))
    return angular_velocity

def update_stepper(step_pin:int, dt:float):
    GPIO.output(step_pin, GPIO.HIGH)
    sleep(dt/2)
    GPIO.output(step_pin, GPIO.LOW)
    sleep(dt/2)

def update_steppers(step_pin:int, step2_pin, dt:float):
    GPIO.output(step_pin, GPIO.HIGH)
    GPIO.output(step2_pin, GPIO.HIGH)
    sleep(dt/2)
    GPIO.output(step_pin, GPIO.LOW)
    GPIO.output(step2_pin, GPIO.LOW)
    sleep(dt/2)

def home_X() -> bool:
    global start_homing_X, homing_started
    global lim_X_pressed

    if not homing_started:
        print("Start homing")
        setPositionStepperX(-500)
        homing_started = True

    if lim_X_pressed:
        print("Home - limit reached")
        resetPositionX()
        homing_started = False
        start_homing_X = False
        return True
    
    return False

def home_Y() -> bool:
    global start_homing_Y, homing_started
    global lim_Y_pressed

    if not homing_started:
        print("Start homing Y")
        setPositionStepperY(-600)
        homing_started = True

    if lim_Y_pressed:
        print("Home Y - limit reached")
        resetPositionY()
        homing_started = False
        start_homing_Y = False
        return True
    
    return False
    
def home_Z() -> bool:
    global start_homing_Z, homing_started
    global lim_Z_pressed

    if not homing_started:
        print("Start homing Z")
        setPositionStepperZ(90)
        homing_started = True

    if lim_Z_pressed:
        print("Home Z - limit reached")
        resetPositionZ()
        homing_started = False
        start_homing_Z = False
        return True
    
    return False

def hardware_setup() -> bool:
    try:
        GPIO.setmode(GPIO.BCM)

        # limit switch setup
        GPIO.setup(LIM_SW_X, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(LIM_SW_Y, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(LIM_SW_Z, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # adding interrupts to the limit switch pins
        GPIO.add_event_detect(LIM_SW_X, GPIO.RISING, callback=LIM_SW_X_pressed, bouncetime=25)
        GPIO.add_event_detect(LIM_SW_Y, GPIO.RISING, callback=LIM_SW_Y_pressed, bouncetime=25)
        GPIO.add_event_detect(LIM_SW_Z, GPIO.RISING, callback=LIM_SW_Z_pressed, bouncetime=25)

        # stepper motor setup
        GPIO.setup(DIR_X, GPIO.OUT)
        GPIO.setup(STEP_X, GPIO.OUT)

        GPIO.setup(DIR_Y, GPIO.OUT)
        GPIO.setup(STEP_Y, GPIO.OUT)
        
        GPIO.setup(DIR_Y2, GPIO.OUT)
        GPIO.setup(STEP_Y2, GPIO.OUT)
        
        GPIO.setup(DIR_Z, GPIO.OUT)
        GPIO.setup(STEP_Z, GPIO.OUT)

        return True

    except:
        return False

def LIM_SW_X_pressed(channel) -> bool:
    global debug_cnt1
    debug_cnt1 = 1

    global lim_X_pressed
    lim_X_pressed = True
    
def LIM_SW_Y_pressed(channel) -> bool:
    global debug_cnt2
    debug_cnt2 = 1

    global lim_Y_pressed
    lim_Y_pressed = True

def LIM_SW_Z_pressed(channel) -> bool:
    global debug_cnt3
    debug_cnt3 = 1

    global lim_Z_pressed
    lim_Z_pressed = True

def distance_to_steps(distance:float) -> int:
    SPR = 200 # steps per revolution
    trap_screw_const = 8 # 8 mm screw jump
    N_steps = round(SPR * distance / trap_screw_const)

    return N_steps

def angle_to_steps(angle:float) -> int:
    SPR = 200 # steps per revolution
    N_steps = round(angle / 360 * 200)

    return N_steps

def main(coordinates:list):
    if not hardware_setup():
        print("Hardware Setup failed!")
        # shut down the machine
        plotter_on = False
        sys.exit()
    else:
        print("Hardware setup successful!")

    # queues
    commandQueueHandle = queue.Queue(maxsize = 4)
    motorX_QueueHandle = queue.Queue(maxsize = 10000)
    motorY_QueueHandle = queue.Queue(maxsize = 10000)
    motorZ_QueueHandle = queue.Queue(maxsize = 10000)

    # motor state variables
    global commandListener_running, commandHandler_running, motorX_Handler_running, motorY_Handler_running, motorZ_Handler_running, positionHandler_running
    commandListener_running = True
    commandHandler_running = True
    motorX_Handler_running = True
    motorY_Handler_running = True
    motorZ_Handler_running = True
    positionHandler_running = True
    
    # create the threads
    commandListenerThread = threading.Thread(target=commandListener, args=(commandQueueHandle,))
    commandHandlerThread = threading.Thread(target=commandHandler, args=(commandQueueHandle,))
    motorX_HandlerThread = threading.Thread(target=motorX_Handler, args=(motorX_QueueHandle,))
    motorY_HandlerThread = threading.Thread(target=motorY_Handler, args=(motorY_QueueHandle,))
    motorZ_HandlerThread = threading.Thread(target=motorZ_Handler, args=(motorZ_QueueHandle,))
    positionHandlerThread = threading.Thread(target=positionHandler, args=(motorX_QueueHandle, motorY_QueueHandle, motorZ_QueueHandle, coordinates,))
    
    # start the threads
    commandListenerThread.start()
    commandHandlerThread.start()
    motorX_HandlerThread.start()
    motorY_HandlerThread.start()
    motorZ_HandlerThread.start()
    positionHandlerThread.start()
        
    try:
        while plotter_running:          
            global debug_cnt1, debug_cnt2, debug_cnt3
            if debug_cnt1 == 1:
                print("LIM1 pressed")
                lim_X_pressed = False
                debug_cnt1 = 0
            elif debug_cnt2 == 1:
                print("LIM2 pressed")
                lim_Y_pressed = False
                debug_cnt2 = 0
            elif debug_cnt3 == 1:
                print("LIM3 pressed")
                lim_Z_pressed = False
                debug_cnt3 = 0

            sleep(.25)
            
    except:
        print("An enormous hell broke lose.")
                
    finally:
        # disable threads
        commandListener_running = False
        commandHandler_running = False
        motorX_Handler_running = False
        motorY_Handler_running = False
        motorZ_Handler_running = False
        positionHandler_running = False
        
        commandListenerThread.join()
        commandHandlerThread.join()  
        motorX_HandlerThread.join()
        motorY_HandlerThread.join()
        motorZ_HandlerThread.join()
        positionHandlerThread.join()
      
        GPIO.cleanup()
        print("Cleanup done.")
        
        print("END")

if __name__ == "__main__":
    main([[0,0,0], [0,0,0]])
    # main()