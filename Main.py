message = '/deliver E218 to E212'

sendCommands = message.split(' ')
pickUp = sendCommands[1]
dropOff = sendCommands[3]

print(sendCommands[1])

import sensor, image, lcd
import utime as time
import KPU as kpu
import gc, sys
from fpioa_manager import fm
from board import board_info
from machine import Timer
from Maix import GPIO
import RecordingAIDataToFile as myLog
import MyConstants as const
from RobotHandler import *
import TelegramHandler as tg

input_size = (224, 224)
# Target rect 120x120 center of QVGA.
_target_rect = [(224-120)//2, (224-120)//2, 120, 120]
camCX = 224/2
camCY = 224/2

labels = ['Tag', 'Path']
anchors = [0.72, 0.69, 0.41, 0.38, 1.41, 1.28, 0.53, 0.5, 6.94, 4.22]
modelName = 'model-95803.kmodel'
model_addr = '/sd/' + modelName
code = []

fm.register(board_info.BOOT_KEY, fm.fpioa.GPIO3, force=True)
key_input = GPIO(GPIO.GPIO3, GPIO.IN)

def lcd_show_except(e):
    import uio
    err_str = uio.StringIO()
    sys.print_exception(e, err_str)
    err_str = err_str.getvalue()
    img = image.Image(size=input_size)
    img.draw_string(0, 10, err_str, scale=1, color=(0xff,0x00,0x00))
    lcd.display(img)

def init_sensor():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    # set to 224x224 input
    sensor.set_windowing((224, 224))
    sensor.set_vflip(0)
    sensor.set_hmirror(0)
    sensor.run(1)
    sensor.skip_frames(30)

def init_lcd():
    lcd.init(type=1)
    lcd.rotation(1)
    lcd.clear(lcd.WHITE)

def AIgetItemNamed(detectedObjects, itemLabel, labels=labels):
    cx, cy, drawProps, confidence = None, None, None, None

    if detectedObjects:
        for obj in detectedObjects:
            if labels[obj.classid()] == itemLabel:
                drawProps = obj.rect()
                cx = obj.x() + obj.w()/2
                cy = obj.y() + obj.h()/2
                confidence = obj.value()
                break

    return cx, cy, drawProps, confidence

myRobot = Robot(onServo=False, onIR=False, onMotor=False, onLCD=True)
telegram = tg.TelegramHandler()

# init_sensor()
# init_lcd()
botState = const.States()
MapRoomTag = const.RoomTagMap
GenDirections = const.NSEW()
currentState = botState.WaitingCommand

running = True
task = None

myRobot.initObjDetectionAI(modelName, anchors, labels)

telegram.sendNotificationTo(telegram.getDataChat(), 'Completed Init')
currentState = botState.WaitingCommand
pickup, dropoff = None, None
currentZone, currentOrientation = None, None
isZoneChanged = None
scanStateCommand = None
currentTargetLocation = None
targetTagroup, targetZone = None, None
isPickingUp = None

waitTime = 10000
startTime = None
movementStartTime = None
movementDuration = 10000 # 3 seconds move forwards
message, group = None, None
# Use in Search for tag state only - Boolean states for SearchForAprilTag State
isMovingForwardSearchTag = False
isAprilFound, isScanningAprilTags = False, False
isOnPathOnInit = False
hasExitedCustom = False

customModeOn = False
customIsFoundPath = False

while running:
    img = sensor.snapshot()
    print('Curent State: ', end='')
    print(currentState)

    if currentState == botState.WaitingCommand:
        # Wait for telegram command.
        img.draw_string(0,200,'Idle. Wait.', scale=2, color=(0, 0, 0))
        if startTime is None: startTime = time.ticks_ms()

        if time.ticks_diff(time.ticks_ms(), startTime) >= waitTime:
            message, group = telegram.checkForUpdates(telegram.getMainChat())
            startTime = time.ticks_ms()

        if message is not None:
            messageOutput = telegram.processMessage(message, group)
            if messageOutput is not None:
            pickUpLocation = messageOutput[0]
            dropOffLocation = messageOutput[1]

            if pickUpLocation in MapRoomTag and dropOffLocation in MapRoomTag:
                # Valid pickup and dropoff
                pickUp = pickUpLocation
                dropoff = dropOffLocation
                isPickingUp = True

                currentState = botState.InitNavigationParams

    elif currentState == botState.InitNavigationParams:
    # Detect current zone, find target zone, detect orientation
        message = None

        if time.ticks_diff(time.ticks_ms(), startTime) >= waitTime:
            message, group = telegram.checkForUpdates(telegram.getDataChat())
            startTime = time.ticks_ms()

        img.draw_string(0,200,'Init Nav Params', scale=2, color=(0, 0, 0))
    # Assuming i type zoning zorreclty and theres no error
    if message is not None:
        if currentZone is None: currentZone = message
        currentOrientation = GenDirections.West

        if isPickingUp:
            currentTargetLocation = pickUp
        else:
            currentTargetLocation = dropOff

        if currentTargetLocation == 'E216':
            targetTagroup =33
            targetZone = 0
        elif currentTargetLocation == 'E212':
            targetTagroup = 36
            targetZone = 2
        elif currentTargetLocation == 'E213':
            targetTagroup = 32
            targetZone = 2

        if int(currentZone[-1]) == targetZone:
            currentState = botState.ScanningForApril
        elif int(currentZone[-1]) == 0:
            currentState = botState.InitGetPath

    elif currentState == botState.InitGetPath:
        if not isOnPathOnInit:
            getPathCommand = 'Reverse'

        if key_input.value() == 0 and not isOnPathOnInit:
            getPathCommand = 'Rotate R'
            isOnPathOnInit = True

        if isOnPathOnInit:
            aiObjects = myRobot.tryToGetObjects(img)
            cx, cy, drawProps, confidence = AIgetItemNamed(aiObjects, 'Path')

            if drawProps is not None:
                img.draw_rectangle(drawProps)
                if cy >= 80 and cy <= 130:
                    currentState = botState.MovingToTargetZone
                    getPathCommand = 'Switch States'
                    # img.draw_string(0,0,'Foward', scale=2, color=(0, 0, 255))
                elif cy < 80:
                    getPathCommand = 'Adjust Right'
                    # img.draw_string(0,0,'Adjust Right', scale=2, color=(0, 0, 255))
                else:
                    getPathCommand = 'Adjust Left'
                    # img.draw_string(0,0,'Adjust Left', scale=2, color=(0, 0, 255))
            else:
                getPathCommand = 'Rotate R'

        img.draw_string(0,0,getPathCommand, scale=2, color=(0, 0, 255))
        img.draw_string(0,200,'InitialFindPath', scale=2, color=(0, 0, 0))

    elif currentState == botState.MovingToTargetZone:
        # Find Scan path along the way - USE AI
        movingStateCommand = None
        message = None
        print('in Moving TOwards Tagert State')
        print(currentZone)
        print('CURENT TRGT ZONE')
        print(targetZone)
        print(currentZone == targetZone)
        if time.ticks_diff(time.ticks_ms(), startTime) >= waitTime:
            message, group = telegram.checkForUpdates(telegram.getDataChat())
            startTime = time.ticks_ms()
            movingStateCommand = 'Wait'

        aiObjects = myRobot.tryToGetObjects(img)
        cx, cy, drawProps, confidence = AIgetItemNamed(aiObjects, 'Path')

        if drawProps is not None:
            img.draw_rectangle(drawProps)
            # img.draw_string(drawProps[0], drawProps[1], "%s : %.2f" %(labels[obj.classid()], obj.value()), scale=2, color=(255, 0, 0))

            if cy >= 80 and cy <= 130:
                movingStateCommand = 'Forward'
                # img.draw_string(0,0,'Foward', scale=2, color=(0, 0, 255))
            elif cy < 80:
                movingStateCommand = 'Adjust Right'
                # img.draw_string(0,0,'Adjust Right', scale=2, color=(0, 0, 255))
            else:
                movingStateCommand = 'Adjust Left'
                # img.draw_string(0,0,'Adjust Left', scale=2, color=(0, 0, 255))
        else:
            movingStateCommand = 'Rotate. Find Path'

        img.draw_string(0,0,movingStateCommand, scale=2, color=(0, 0, 255))
        stateString = 'T: ' + str(currentTargetLocation) + ', Z: ' + str(targetZone)
        img.draw_string(0,200,stateString, scale=2, color=(0, 0, 0))

        if message is not None:
            isZoneChanged = (currentZone != message)
            targetTagroup = None
            # targetZone = None

            if isZoneChanged:
                zOld = int(currentZone[-1])
                zNew = int(message[-1])
                changeNum = math.fabs(zOld - zNew)

                if changeNum == 1:
                    currentZone = message

        if int(currentZone[-1]) == targetZone:
            currentState = botState.ScanningForApril

    elif currentState == botState.ScanningForApril:
        # scanStateCommand = None
        message = None
        aiObjects = None
        tagcx, tagcy, tagh, tagw = None, None, None, None
        img.draw_string(0,200,'Scanning for Tags', scale=2, color=(0, 0, 0))
        aiObjects = myRobot.tryToGetObjects(img)

        if time.ticks_diff(time.ticks_ms(), startTime) >= waitTime:
            message, group = telegram.checkForUpdates(telegram.getDataChat())
            startTime = time.ticks_ms()
            scanStateCommand = 'WAIT'

        if not customModeOn and key_input.value() == 0 and not hasExitedCustom:
            customModeOn = True

        if customModeOn:
            scanStateCommand = 'Rotate to South'
            pathcx, pathcy, pathdrawProps, pathconfidence = AIgetItemNamed(aiObjects, 'Path')

            if pathdrawProps is not None:
                scanStateCommand = 'Move Left'
                img.draw_rectangle(pathdrawProps)
                # img.draw_string(drawProps[0], drawProps[1], "%s : %.2f" %(labels[obj.classid()], obj.value()), scale=2, color=(255, 0, 0))

                if pathcy >= 80 and pathcy <= 130:
                    movingStateCommand = 'Forward'
                    # img.draw_string(0,0,'Foward', scale=2, color=(0, 0, 255))
                    customModeOn = False
                    hasExitedCustom = True
        else:
            pathcx, pathcy, pathdrawProps, pathconfidence = AIgetItemNamed(aiObjects, 'Path')
            tagcx, tagcy, tagdrawProps, tagconfidence = AIgetItemNamed(aiObjects, 'Tag')
            if tagdrawProps is not None:
                # Found Tags - Orient to make Tags Centerd (Y coordinate because camera is rotated sideways)
                # After oritent, move forward
                isAprilFound = True
                img.draw_rectangle(tagdrawProps)
                tagw = tagdrawProps[2]
                tagh = tagdrawProps[3]

            if not isAprilFound:
                # Move forward for movementDuratrion of time then stop to scan
                # Sweep to scan for April tags in cone of 30 - 60 degrees
                # Simulate compelte scanning by presing boot button
                if movementStartTime is None:
                    scanStateCommand = 'Forward'
                    movementStartTime = time.ticks_ms()
                    isMovingForwardSearchTag = True

                if time.ticks_diff(time.ticks_ms(), movementStartTime) >= movementDuration:
                    isMovingForwardSearchTag = False
                    scanStateCommand = 'Sweep'

                    if key_input.value() == 0 and not isMovingForwardSearchTag:
                        # To simulate complete sweeping of view angle
                        isMovingForwardSearchTag = True
                        scanStateCommand = 'Forward'
                        movementStartTime = time.ticks_ms()
            else:
            # The previously detected tag still not scanned - move towards it
                if tagw is not None:
                    # The tag was detected again in this loop
                    if tagcy >= 90 and tagcy <= 120:
                        # Is at center, move forward
                        scanStateCommand = 'Forward'
                    elif tagcy < 90:
                        scanStateCommand = 'Rot Right'
                    else:
                        scanStateCommand = 'Rot Left'

                    if scanStateCommand == 'Forward' and (tagh >= 75 or tagw >= 75):
                        # Tag should be close enough for April tag scanning
                        # Forward command means it should be at the center
                        # currentState = botState.TryAprilTagScan
                        scanStateCommand = 'Stop'

                else:
                    scanStateCommand = 'Forward'

        img.draw_string(0,0,scanStateCommand, scale=2, color=(0, 0, 255))

    elif currentState == botState.TryAprilTagScan:
        img.draw_string(0,200,'Scanning April', scale=2, color=(0, 0, 0))

        if myRobot.isAIInitialized:
            myRobot.offloadAI()
            time.sleep_ms(10000)

        print('AI Init State: ', end='')
        print(myRobot.isAIInitialized)

        img.draw_rectangle(_target_rect,color=(0,255,0),thickness=5)
        april = myRobot.tryScanAprilTag(img)

        if april is not None:
            print('Found APRIL')
        print(april)

    lcd.display(img)
