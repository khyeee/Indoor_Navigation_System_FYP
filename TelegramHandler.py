import uos
import myRequestsMod as requests
from network_esp32 import wifi

class TelegramHandler:
    def __init__(self):
        # For interacting with telegram bot API
        # Method of API Call is: https://api.telegram.org/bot<token>/METHOD_NAME
        self.tgBotAPIKey = "TELEGRAM_BOT_API_KEY"
        self.tgBaseURL = "https://api.telegram.org/bot"

        # Chat IDs
        # NOTE: IDs and Hotspot credentials are just boilerplate numbers for uploading into Git Repo
        # Refer back to submission for actual values if reusing code

        self.mainChat = '-1234566789'
        self.dataChat = '-1234567891'
        self.myChatID = '1029385748'

        # Wifi data
        self.SSID = "Phone Hotspot Name"
        self.PASW = "Phone Hotspot Password"
        self.tryConnectWifi()
        if wifi.isconnected():
            print('Connected')
            #self.sendNotificationTo(self.dataChat, 'Connection Successful')

    def getMainChat(self):
        return self.mainChat

    def getDataChat(self):
        return self.dataChat

    def tryConnectWifi(self):
        if not wifi.isconnected():
            for i in range(5):
                try:
                    # Running within 3 seconds of power-up can cause an SD load error
                    # wifi.reset(is_hard=False)
                    wifi.reset(is_hard=True)
                    print('try AT connect wifi...')
                    wifi.connect(self.SSID, self.PASW)

                    if wifi.isconnected():
                        break
                except Exception as e:
                print(e)

        print('network state:', wifi.isconnected(), wifi.ifconfig())

    def disconnectWifi(self):
        wifi.nic.disconnect()

    def sendNotificationTo(self, chatID, message):
        params = {'chat_id': chatID, 'text': message, 'parse_mode': 'HTML'}
        sendMessageURL = self.tgBaseURL + self.tgBotAPIKey + '/sendMessage'

        try:
            resp = requests.post(sendMessageURL, json=params)
            print(resp.status_code)
        except:
            pass

    # Getting Last Update ID from SD Card - Make sure SD is attached to MAixDuino
    def getLastUpdateID(self, forChatGroup):
        isSDCardAttached = False
        isFileExists = False
        targetTxtFile = ''

        if forChatGroup == self.mainChat:
            targetTxtFile = 'deliveryGroupLastUpdateID.txt'
        elif forChatGroup == self.dataChat:
            targetTxtFile = 'dataSharingLastUpdateID.txt'
        else:
            print("Received data from a non-accepted chat ID. Disregard and dont process message")
            return -1

        mountPoints = uos.listdir('/')
        for mount in mountPoints:
            if mount == 'sd':
                isSDCardAttached = True
                for files in uos.listdir('/sd'):
                    if files == targetTxtFile:
                        isFileExists = True
                        break

                if isFileExists:
                    break

        if not isSDCardAttached:
            print('No SD Card attached/SD Card is unreadable')
            return -1

        if not isFileExists:
            # TXT file does not exist. Create a new file with initialized value of 0
            print('File does not exist. Creating file in SD Card')
            with open(targetTxtFile, 'w') as f:
            f.write('0')
            isFileExists = True

        try:
            with open(targetTxtFile, 'r') as f:
            fileContents = f.read()
            #print('Reading from ', targetTxtFile)
            #print('Contents: ', fileContents)
            return int(fileContents)
        except:
            print('Failed to open file to read')
            return -1

    def updateLastUpdateID(self, forTargetGroup, newUpdateID):
        if forTargetGroup == self.mainChat:
            targetTxtFile = 'deliveryGroupLastUpdateID.txt'
        elif forTargetGroup == self.dataChat:
            targetTxtFile = 'dataSharingLastUpdateID.txt'
        else:
            print('Received data from a non-accepted chat ID. Disregard and dont process message')
            return -1

        with open(targetTxtFile, 'w') as file:
            file.write(str(newUpdateID))

    def checkForUpdates(self, fromGroupID):
        # Returns an array containing: [messsage, recevingGroupID]
        # Both of these information will be used in processMessage
        print('Begin checking for updates')
        data = [None, None]
        params = {
            'offset':self.getLastUpdateID(fromGroupID) + 1
        }

        getUpdatesURL = self.tgBaseURL + self.tgBotAPIKey + '/getUpdates'
        resp_updates = requests.get(getUpdatesURL, json=params)
        print('Printing get results stat code')
        print(resp_updates.status_code)

        if resp_updates.status_code == 200:
            jsonData = resp_updates.json()
            dataResults = jsonData['result']

            if dataResults is not None:
        for result in dataResults:
            updateID = result['update_id']
            receiverID = result['message']['chat']['id']

            if str(receiverID) == fromGroupID and updateID > int(self.getLastUpdateID(fromGroupID)):
                messageText = result['message']['text']
                senderID = result['message']['from']['id']

                data = [messageText, fromGroupID]
                self.updateLastUpdateID(fromGroupID, updateID)
        # Uncomment break for bot to process "EVERY" New message
        # Otherwise, leave "break" commented for robot to only process the most recent update
        # break
        return data

    def processMessage(self, messageText, fromGroupID):
        if fromGroupID == self.mainChat:
            # Process commands from users in the chat group - can potentially add further verification from a list of accepted users
            print('Received Text from: ', fromGroupID)
            print(messageText)

            if messageText == '/status':
                print('Developing: Share robot location and status')
            elif messageText == '/help':
                print('Show list of commands that can be used')
            elif '/send' in messageText:
                # Received send command process the rest
                # /send PICKUP to DROPOFF
                print('Before Split')
                sendCommand = messageText.split(' ')
                print('AfterSplit')
                print(sendCommand)
                pickupLocation = sendCommand[1]
                dropoffLocation = sendCommand[3]

                return [pickupLocation, dropoffLocation]

            elif messageText == '/pickup':
                print('Deciding Pickup Point')
            elif messageText == '/debugstop':
                print('For emergency Stopping the robot operation')
            elif messageText == '/debugforward':
                print('Debugging move forward')
            elif messageText == '/debugRotCamCW':
                print('Debugging rotate camera servo CW')
            elif messageText == '/debugRotCamCCW':
                print('Debugging rotate camera servo CCW')
            elif messageText == '/debugRotWheelCW':
                print('Debugging rotate wheel Clockwise')
            elif messageText == '/debugRotWheelCCW':
                print('Debugging rotate wheel counter CW')
            elif messageText == '/record':
                self.sendNotificationTo(self.mainChat, 'Received record command')

                return -1000 # For debugging only
            elif messageText == '/stopRecord':
                self.sendNotificationTo(self.mainChat, 'Received STOP command')

                return -2000 # For debugging only

        elif fromGroupID == self.dataChat:
            # Process the data passed in from ESP8266 of which zone the robot is currently in
            print('Received Zoning Info from: ', fromGroupID)
            print(messageText)
            zone = messageText[-1]

            return int(zone)

        return None

