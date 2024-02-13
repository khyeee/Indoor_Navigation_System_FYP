class Direction():
    def __init__(self):
        self.CCW = 'CCW'
        self.CW = 'CW'
        self.STOP = 'STOP'

class Servo():
    def __init__(self):
        self.Top = 'Top'
        self.Down = 'Down'

class States():
    def __init__(self):
        self.WaitingCommand = 'Waiting Command'
        self.MovingToTargetZone = 'MovingToTargetZone'
        self.ScanningForApril = 'ScanApril'
        self.FindingAprilAI = 'SweepingForAprilTag'
        self.InitNavigationParams = 'InitNavigation' # Get Command, go to pick up, wait for pck up, go to dropoff, wait for pickup, go home
        self.TryAprilTagScan = 'OffAIScanApril'
        self.InitGetPath = 'InitialGetPath'

class NSEW():
    def __init__(self):
        self.North = 'North'
        self.South = 'South'
        self.East = 'East'
        self.West = 'West'

RoomTagMap = {
    # TagID, Zone Location
    'E212': (36, 2),
    'E213': (32, 2),
    'E216': (33, 0) # HOME
}
