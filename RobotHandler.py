import utime as time
from Maix import GPIO
from fpioa_manager import fm
from board import board_info
from machine import Timer, PWM
import KPU as kpu
import sensor, lcd, gc
from ucollections import namedtuple
# import TelegramHandler as tg
import MyConstants as conf
import math

class Robot():
    direction = conf.Direction()
    servo = conf.Servo()

    ToggleRotDir = {
        direction.CW : direction.CCW,
        direction.CCW : direction.CW
    }
    def __init__(self, onServo=True, onIR=True, onMotor=True, onCamera=True,
    onLCD=False):
        # Initialize global pins and struct naming
        self.upServoPin = board_info.PIN10
        self.downServoPin = board_info.PIN11
        self.bottomIRPin = board_info.PIN8
        self.topIRPin = board_info.PIN9
        self.motorEnPin = board_info.PIN6 # Gray Wire
        self.motorDirPin = board_info.PIN7 # Purple wire
        self.zone0Pin = board_info.PIN2
        self.zone1Pin = board_info.PIN3

        # Initializing robot components
        if onServo: self.initTopDownServos()
        if onIR: self.initIRSensors()
        if onMotor: self.initDrivingMotor()
        if onCamera: self.init_sensor() # Camera
        # self.initManualZoning()

        # Initializing AI
        # AI Is to be initialized separately becuase it needs its own anchors and model name

        if onLCD: self.init_lcd()

        # Initializing robot states

        self.cameraAngle = 0

        # Set Starting Servo Positions
        # TOP
        # self.resetCameraPosition()

    def initManualZoning(self):
        fm.register(self.zone0Pin, fm.fpioa.GPIO2, force=True)
        fm.register(self.zone1Pin, fm.fpioa.GPIO5, force=True)

        self.zone0 = GPIO(GPIO.GPIO2, GPIO.IN)
        self.zone1 = GPIO(GPIO.GPIO5, GPIO.IN)

    def getZone(self):
        print('Zone 0: ', end='')
        print(self.zone0.value(), end='')
        print('Zone 1: ', end='')
        print(self.zone1.value(), end='')

        if self.zone0.value() == 0 and self.zone1.value() == 0:
            return 'zone0'
        elif self.zone0.value() == 0 and self.zone1.value() == 1:
            return 'zone1'
        elif self.zone0.value() == 1 and self.zone1.value() == 0:
            return 'zone2'
        elif self.zone0.value() == 1 and self.zone1.value() == 1:
            return 'zone3'

    # Initializing Sensor (Camera) and LCD if needed
    def init_sensor(self, sensor_vflip=0, sensor_hmirror=0):
        sensor.reset()
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(sensor.QVGA)
        sensor.set_windowing((224, 224)) # set to 224x224 input
        sensor.set_vflip(sensor_vflip)
        sensor.set_hmirror(sensor_hmirror)
        sensor.run(1)
        sensor.skip_frames(30)

    def init_lcd(self):
        lcd.init(freq=15000000)
        lcd.rotation(1)
        lcd.clear()

    # Initializing and controlling Servo Motors
    def initTopDownServos(self):
        topTim = Timer(Timer.TIMER0, Timer.CHANNEL0, mode=Timer.MODE_PWM)
        downTim = Timer(Timer.TIMER1, Timer.CHANNEL1, mode=Timer.MODE_PWM)
        self.topServo = PWM(topTim, freq=50, duty=3,pin=self.upServoPin)
        self.downServo = PWM(downTim,freq=50,duty=3,pin=self.downServoPin)
        self.turnServo(self.servo.Top, self.direction.STOP)
        self.turnServo(self.servo.Down, self.direction.STOP)
        #return (topServo, downServo)

    def turnServo(self, servo, direction):
        isMoving = False
        servoToSet = None
        ccwSpeed, cwSpeed = None, None
        stopSpeed = 7.5

        if servo == self.servo.Top:
            # Controlled speed for better precision
            servoToSet = self.topServo
            cwSpeed = 6.6
            ccwSpeed =8.5

            if direction == self.direction.CCW:
                servoToSet.duty(ccwSpeed) # 12.9 is max power. 10.9 seems ok but lookingto go lower. 9.9
                isMoving = True
            elif direction == self.direction.CW:
                servoToSet.duty(cwSpeed) # 2.1 is max power 4.1 seems ok, also looking to go lower to try for srtability 5.1
                isMoving = True
            elif direction == self.direction.STOP:
                servoToSet.duty(stopSpeed)

        elif servo == self.servo.Down:
            servoToSet = self.downServo
            ccwSpeed = 2.1
            cwSpeed = 12.8

            if direction == self.direction.CCW:
                servoToSet.duty(ccwSpeed) # 12.9 is max power. 10.9 seems ok but lookingto go lower. 9.9
                isMoving = True
            elif direction == self.direction.CW:
                servoToSet.duty(cwSpeed) # 2.1 is max power 4.1 seems ok, also looking to go lower to try for srtability 5.1
                isMoving = True
            elif direction == self.direction.STOP:
                servoToSet.duty(stopSpeed)
        else:
            print('Invalid Servo selected')
            return False

        return isMoving

    def turnServoPrecise(self, servo, direction, timePerDegree, angle):
        # Make sure servo is stopped before initiating turn
        self.turnServo(servo, self.direction.STOP)
        sign = None, turnAngle = 1

        if direction == self.direction.CW:
            sign = -1
        else:
            sign = 1

        if math.fabs(self.cameraAngle + turnAngle * sign) > 90:
            turnAngle = math.fabs(math.fabs(self.cameraAngle) - turnAngle) # Limit the camera turning to 180 degrees

        turnTime = timePerDegree * turnAngle
        isWaiting = True

        self.turnServo(servo, direction)
        startTime = time.ticks_ms()

        while isWaiting:
            if time.ticks_diff(time.ticks_ms(), startTime) >= turnTime:
                isWaiting = False
                self.turnServo(servo, self.direction.STOP)
                break

        self.cameraAngle += (turnAngle * sign)

    def sweepScan(self):
        pass

    # Getters for Servo motors
    def ServoTop(self):
        return self.topServo

    def ServoDown(self):
        return self.downServo
    # Initializing and Controlling IR Sensors
    def initIRSensors(self):
        fm.register(self.bottomIRPin, fm.fpioa.GPIOHS0, force=True)
        fm.register(self.topIRPin, fm.fpioa.GPIOHS1, force=True)
        self.downIR = GPIO(GPIO.GPIOHS0, GPIO.IN)
        self.topIR = GPIO(GPIO.GPIOHS1, GPIO.IN)

        #return (topIR, downIR)
    def isIRDetected(self, irSensor):
        # IMPORTANT: Somehow if both are triggered at the same time, I get both False.
        # Probably best to do calibration for IR one at a time.
        return (irSensor.value() == 0)
    # IR Getters
    def IRTop(self):
        return self.topIR

    def IRDown(self):
        return self.downIR

    # Intializing and Controlling DC Motor
    def initDrivingMotor(self):
        # According to the wiring, this is how it should be.
        # However, the actual use case seems to be flipped, where en is Dir and Dir is en
        fm.register(self.motorEnPin, fm.fpioa.GPIO3, force=True)
        fm.register(self.motorDirPin, fm.fpioa.GPIO4, force=True)
        self.mEn = GPIO(GPIO.GPIO3, GPIO.OUT)
        self.mDir = GPIO(GPIO.GPIO4, GPIO.OUT)
        self.mEn.value(0)
        self.mDir.value(0)

        #return (mDir, mEn)

    def moveRobot(self, direction):
    # TODO: Need to finalize if i want to use F and B or CW and CCW
        if direction == 'F':
            # Move forward
            self.mEn.value(1)
            self.mDir.value(0) # Pending change. Unsure CW or CCW is forward
            # CW = 1, CCW = 0
        elif direction == 'B':
            self.mEn.value(1)
            self.mDir.value(1)
        else:
            # Stop Motor
            self.mEn.value(0)

    # Pre-written methods for calibration and stuff
    def calibrateCameraServo(self):
        # Assume that the camera is already at 0 degrees Starting position
        # Assume that CW and CCW share same timing

        calibratedOutput = {
            'CW':None,
            'CCW': None
        }

        self.resetCameraPosition()
        servoMotor = self.servo.Top
        irSensor = self.IRTop()

        startTime = None
        servoMoving = self.turnServo(servoMotor, self.direction.STOP)

        timePerDegree = None # detectedOnce = False

        while timePerDegree is None:
            if startTime is None:
                startTime = time.ticks_ms()
                self.turnServo(servoMotor, self.direction.CW)

            if self.isIRDetected(irSensor) and time.ticks_diff(time.ticks_ms(), startTime) > 500:
                self.turnServo(servoMotor, self.direction.STOP)
                timePerDegree = time.ticks_diff(time.ticks_ms(), startTime) / 360
                break

        # Move servo a little bit so it isnt on IR Anymore
        self.turnServo(servoMotor, self.direction.CCW)
        time.sleep_ms(300)
        self.turnServo(servoMotor, self.direction.STOP)
        self.resetCameraPosition()
        return timePerDegree

    def calibrateServo(self, servo):
        # Calibration Steps:
        # 1. Rotate until detect bar.
        # 2. Rotate opposite direction until bar is detected then undetected to reach strating positiong
        # 3. Calculate time taken for step 2 for 360 degree calibration

        calibratedOutput = {
            'CW':None,
            'CCW': None
        }

        self.resetCameraPosition()
        directions = ['CCW', 'CW']
        servoMotor = servo
        irSensor = self.IRTop() if servo == 'Top' else self.IRDown()

        for direction in directions:
            timePerDegree = None
            startTime = None
            endTime = None
            initialStopTime = None # Introduce slight delay to the system so it has time to move out of IR Range
            findingInitialPosition = True
            turnDirection = direction
            detectCount = 0
            servoMoving = self.turnServo(servoMotor, 'STOP')

        while timePerDegree is None:
            if findingInitialPosition:
                #print('finding initial position')
                #print('IR Detection: ', end='')
                #print(isIRDetected(irSensor))
                if not servoMoving:
                    print('Servo was not moving, turned on servo')
                    servoMoving = self.turnServo(servoMotor, turnDirection)

                if self.isIRDetected(irSensor):
                    #print('ir detected from finding initial position')
                    detectCount += 1 # First time detecting
                    findingInitialPosition = False

                    if servoMoving:
                        # Stop the motor and Toggle to other direction when reach the initial position
                        servoMoving = self.turnServo(servoMotor, 'STOP')
                        turnDirection = self.ToggleRotDir[turnDirection]
                        initialStopTime = time.ticks_ms()

            if not findingInitialPosition:
                if not servoMoving:
                    servoMoving = self.turnServo(servoMotor, turnDirection)
                    startTime = time.ticks_ms()

                if servoMoving and (time.ticks_diff(time.ticks_ms(), startTime) > 250):
                    if self.isIRDetected(irSensor):
                        if detectCount < 2:
                            # Second time detecting IR - detection happens on other side.
                            # Wait for undetect
                            detectCount += 1

                if not self.isIRDetected(irSensor) and detectCount >= 2:
                    servoMoving = self.turnServo(servoMotor, self.direction.STOP)
                    endTime = time.ticks_ms()
                    turnDuration = time.ticks_diff(endTime,
                    startTime)
                    timePerDegree = turnDuration / 360 # In milliseconds
                    calibratedOutput[turnDirection] = timePerDegree
                    time.sleep_ms(1500)

        return (calibratedOutput['CW'], calibratedOutput['CCW'])

    # Scanning april tags
    def tryScanAprilTag(self, snapshot):
        # Will return None if no tags detected. Otherwise returns tag data in namedtuple form
        #snapshot = sensor.snapshot()

        scannedTags = []
        #_target_rect = [(320-120)//2, (240-120)//2, 120, 120]
        _target_rect = [(224-120)//2, (224-120)//2, 120, 120]
        TagInfo = namedtuple('TagInfo', ('id', 'xtrans', 'ytrans', 'ztrans','xrot', 'yrot', 'zrot', 'xcentre', 'ycentre', 'xbox','ybox','wbox','hbox'))
        myTag = None

        try:
            target = snapshot.copy(_target_rect)
            target = target.to_grayscale()
            target = target.resize(28,28)
            scannedTags = target.find_apriltags()

            if len(scannedTags) > 0:
                # Found tags. Assign to named tuple to be returned as an object
                tag = scannedTags[0]
                myTag = TagInfo(tag.id(), tag.x_translation(), tag.y_translation(), tag.z_translation(), tag.x_rotation(), tag.y_rotation(), tag.z_rotation(), tag.cx(), tag.cy(), tag.x(), tag.y(), tag.w(), tag.h())
                print(scannedTags[0])

        except Exception as e:
            print("[ERROR] Exception: %s" % (e))

        return myTag

    def offloadAI(self):
        self.isAIInitialized = False
        kpu.deinit(self.task)
        self.task = None
        gc.collect()

    def initObjDetectionAI(self, modelName, anchors, labels):
        anch = anchors
        modelAddress = '/sd/' + modelName
        self.labels = labels
        self.modelName = modelName
        self.anchors = anchors

        self.task = None
        self.isAIInitialized = False

        try:
            print('Loading AI')
            self.task = kpu.load(modelAddress)
            kpu.init_yolo2(self.task, 0.5, 0.3, 5, anch)
            self.isAIInitialized = True
        except Exception as e:
            raise e

    def aiVisualDebug(self, img):
        objLabels = self.labels
        objects = None
        minimumConfidence = 0.7

        if self.isAIInitialized:
            objects = kpu.run_yolo2(self.task,img)

            # To take the array returned from tryToGetObjects() method
            if objects:
            for obj in objects:
                detectedObj = self.labels[obj.classid()]
                objx = obj.x()
                objy = obj.y()
                objh = obj.h()
                objw = obj.w()
                confidence = obj.value()

                # TODO: Either return commands for the main bot to process or cary out the commands here
                if confidence >= minimumConfidence:
                    if detectedObj == 'blocked':
                        pass
                    elif detectedObj == 'maybeTag':
                        print('Do this thing based on the object detected')
                    elif detectedObj == 'clear':
                        print('Detected Clear')
                    elif detectedObj == 'Path':
                        print('is path')
                    elif detectedObj == 'tag':
                        print('Found Tag')
                pos = obj.rect()
                img.draw_rectangle(pos)
                img.draw_string(pos[0], pos[1], "%s : %.2f" %(detectedObj, confidence), scale=2, color=(255, 0, 0))

        return img # to be used in display lcd

    def tryToGetObjects(self, img):
        objects = None

        if self.isAIInitialized:
            objects = kpu.run_yolo2(self.task, img)
        else:
            self.initObjDetectionAI(self.modelName, self.anchors,
            self.labels)

        return objects # return the array of objects then process in separate function??

    def processObjects(self, detectedObjects, minimumConfidence = 0.5):
        # To take the array returned from tryToGetObjects() method
        if detectedObjects:
            for obj in detectedObjects:
                detectedObj = self.labels[obj.classid()]
                objx = obj.x()
                objy = obj.y()
                objh = obj.h()
                objw = obj.w()
                confidence = obj.value()

                # TODO: Either return commands for the main bot to process or cary out the commands here
                if confidence >= minimumConfidence:
                    if detectedObj == 'blocked':
                        pass
                    elif detectedObj == 'maybeTag':
                        print('Do this thing based on the object detected')
                    elif detectedObj == 'clear':
                        print('Detected Clear')
                    elif detectedObj == 'Path':
                        print('is path')
                    elif detectedObj == 'tag':
                        print('Found Tag')

    def resetCameraPosition(self):
        self.turnServo(self.servo.Top, self.direction.STOP)
        self.topStoppedLeft = self.isIRDetected(self.IRTop())

        if not self.topStoppedLeft: self.turnServo(self.servo.Top, self.direction.CCW)

        while not self.topStoppedLeft:
            # print('in resetCamera position. waiting for IR check')
            self.topStoppedLeft = self.isIRDetected(self.IRTop())
            if self.topStoppedLeft:
                self.turnServo(self.servo.Top, self.direction.STOP)

        self.cameraAngle = 0

    def roughSweepScanForTags(self):
        turnDirection = self.direction.STOP

        if self.isAIInitialized:
            if self.topStoppedLeft:
                turnDirection = self.direction.CW
            else:
                turnDirection = self.direction.CCW
# --------------------------------------------------------------------------
# --------------------------------------------------------------------------
# END OF CLASS DECLARATION
# --------------------------------------------------------------------------
# --------------------------------------------------------------------------

