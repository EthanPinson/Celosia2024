from cscore import CameraServer

def main():
    CameraServer.enableLogging()

    CameraServer.startAutomaticCapture(0)
    #CameraServer.startAutomaticCapture(1)

    CameraServer.waitForever()