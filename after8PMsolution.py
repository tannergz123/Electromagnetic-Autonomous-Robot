import time
import csv
from adafruit_servokit import ServoKit
import board
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
import digitalio
import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera

#setting up board
i2c = board.I2C()
pca = PCA9685(i2c)
pca.frequency = 50

mag17 = digitalio.DigitalInOut(board.D17)
mag17.direction = digitalio.Direction.OUTPUT
mag17.value = True

button_pin = board.D24
button = digitalio.DigitalInOut(button_pin)
button.direction = digitalio.Direction.INPUT
button.pull = digitalio.Pull.UP

kit = ServoKit(channels=16)

#Turn magnet on/off
def magnet(status):
    mag17.value = not status
    if (not status):
        print("GPIO pin is ON")
    else:
        print("GPIO pin is OFF")
        
#Moves arm behind its normal range to drop off picked up objects
def placeItem():
    move(servo4,4,20)
    time.sleep(0.5)
    move(servo2,2,50)
    time.sleep(0.5)
    move(servo0,0,60)
    time.sleep(0.5)

#Writes an updated servo position to file.
def writePosition(servoNum, angle):
    copyDict = {}
    with open('servoPosition.csv', mode='r') as file:   
        reader = csv.reader(file)
        for line in reader:
            copyDict.update({str(line[0]):line[1]})
        with open('servoPosition.csv', 'w') as csv_file:  
            writer = csv.writer(csv_file)
            copyDict.update({str(servoNum):str(angle)})
            for key, value in copyDict.items():
                writer.writerow([key, value]) 
                     
#Gets the current position of a servo.
def getPosition(servoNum):
    copyDict = {}
    with open("servoPosition.csv", 'r') as file:
        reader = csv.reader(file)
        for line in reader:
            copyDict.update({str(line[0]):line[1]})
        return copyDict[str(servoNum)]

#Moves a servo to an angle.
def move(servo, num, ang):
    initialAngle = float(getPosition(num))
    angleDiff = (abs(ang - initialAngle))
    if (angleDiff < 0.3):
        return
    numStep = max(round(1.2 * angleDiff), 1)
    increment = (ang - initialAngle)/(numStep)
    currentAngle = initialAngle
    for x in range(0, numStep):
        currentAngle += increment
        if (currentAngle < 0):
            currentAngle = 1
        elif (currentAngle > 180):
            currentAngle = 180
        servo.angle = round(currentAngle,1)
        writePosition(int(num), str(round(currentAngle,1)))
        time.sleep(abs(increment/40))
        
#returns the arm to a known default position.
def returnDefault(angle0 = 0):
    move(servo0, 0, angle0)
    time.sleep(.5)
    move(servo2, 2, 60)
    time.sleep(.5)
    move(servo4, 4, 60)
    time.sleep(.5)

#rotates, searching for objects to pick up and picks them up.
def scan_and_rotate_servo():
    start_angle = 10
    end_angle = 150
    step = 1  # Define step size for angle increment

    # Allow the camera and servo to warm up
    time.sleep(0.1)
    
    # Main loop to capture frames and rotate servo
    for angle in range(start_angle, end_angle, step):
        servo0.angle = angle
        writePosition('0', str(angle))
        time.sleep(0.05)
        # Capture a single frame
        camera.capture(rawCapture, format="bgr")
        image = rawCapture.array

        # Process the image
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        _, binary = cv2.threshold(edges, 50, 255, cv2.THRESH_BINARY)
        kernel = np.ones((5,5), np.uint8)
        dilated = cv2.dilate(binary, kernel, iterations=1)
        contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        height, width = image.shape[:2]
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)

            if x > 0 and y > 0 and (x+w) < width and (y+h) < height:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    if (cy >= 200):
                        continue
                    cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    cv2.circle(image, (cx, cy), 5, (0, 0, 255), -1)
                    if 140<= cx <= 150:
                        #print("Object detected at 100px along the x-axis")
                        #print('cy', cy)
                        angle1, angle2 = get_servo_angles(cy)
                        if (angle1 != -1):
                            move(servo4, 4, float(angle2))
                            time.sleep(.5)
                            magnet(True)
                            time.sleep(.1)
                            move(servo2, 2, float(angle1))
                            time.sleep(.5)
                            placeItem()
                            time.sleep(.25)
                            magnet(False)
                            returnDefault(angle)  
                            time.sleep(.5)

        # Clear the stream in preparation for the next capture
        rawCapture.truncate(0)
        rawCapture.seek(0)

        cv2.destroyAllWindows()


#Retrieve servo angles from y-pixel location
calibration_data = {
    163: (88.3, 113),
    152: (86.9, 112),
    134: (85.7, 109.3),
    119: (85.9, 106),
    103: (85.7, 102.2),
    90: (85.9, 99),
    80: (86.3, 96.6),
    69: (86.7, 94.3),
    56: (87.8, 90.1),
    45: (89.5, 86),
    38: (90.5, 82.4),
    24: (92, 78)
}

#helper function to convert pixel values into servo angles.
def get_servo_angles(y_pixel):
    # If exact match found in calibration data
    if y_pixel in calibration_data:
        return calibration_data[y_pixel]
    
    # If no exact match, interpolate between the two closest points
    # Requires calibration_data to be sorted by key
    sorted_keys = sorted(calibration_data.keys())   
    for i in range(len(sorted_keys) - 1):
        if sorted_keys[i] < y_pixel < sorted_keys[i + 1]:
            # Perform linear interpolation
            lower_key = sorted_keys[i]
            upper_key = sorted_keys[i + 1]
            lower_value = calibration_data[lower_key]
            upper_value = calibration_data[upper_key]
            fraction = (y_pixel - lower_key) / (upper_key - lower_key)
            interpolated_angle_servo2 = lower_value[0] + fraction * (upper_value[0] - lower_value[0])
            interpolated_angle_servo4 = lower_value[1] + fraction * (upper_value[1] - lower_value[1])
            return (interpolated_angle_servo2, interpolated_angle_servo4)
    
    # If y_pixel is outside the calibrated range, return -1
    return (-1, -1)
       
#testing function for calibration
def outputY():

    camera.capture(rawCapture, format="bgr")
    image = rawCapture.array

    # Process the image
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 150)
    _, binary = cv2.threshold(edges, 50, 255, cv2.THRESH_BINARY)
    kernel = np.ones((5,5), np.uint8)
    dilated = cv2.dilate(binary, kernel, iterations=1)
    contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    height, width = image.shape[:2]
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        if x > 0 and y > 0 and (x+w) < width and (y+h) < height:
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                if (cy >= 200):
                    continue
                cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.circle(image, (cx, cy), 5, (0, 0, 255), -1)
                if 130<= cx <= 140:
                    #print("Object detected at 100px along the x-axis.")
                    #print('cy', cy)
                    return cy
                else:
                    return None
            rawCapture.truncate(0)
            rawCapture.seek(0)
        rawCapture.truncate(0)
        rawCapture.seek(0)
                
camera = PiCamera()
camera.resolution = (320, 240)
rawCapture = PiRGBArray(camera, size=(320, 240))                
                
#Setting default conditions for servos
servo0 = servo.Servo(pca.channels[0], min_pulse=150, max_pulse=4000)
servo2 = servo.Servo(pca.channels[2], min_pulse=150, max_pulse=4000)
servo4 = servo.Servo(pca.channels[4], min_pulse=150, max_pulse=4000)

#initial position
move(servo0, 0, 10)
move(servo2, 2, 60)
move(servo4, 4, 80)

writePosition('0', '10')
writePosition('2', '60')
writePosition('4', '80')

time.sleep(2)

while True:
    if (button.value == False):
        print('Starting Scan')
        break
scan_and_rotate_servo()
returnDefault()

