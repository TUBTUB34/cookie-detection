import cv2
import cv2.aruco as aruco
import numpy as np
import serial
import time

class CPi:
    def __init__(self):
        self._serial_port = serial.Serial('/dev/ttyAMA0', 9600, timeout=2)  # A

        self._cameraMatrix = np.array([[645.48909789308948, 0.0, 290.33679782424349],
                                       [0.0, 644.5968920384571, 241.67036944659537],
                                       [0.0, 0.0, 1.0]], dtype=np.float32)
        self._distCoeffs = np.array([0.012652587272703592,
                                     0.10027345940008622,
                                     -0.0011629470619494956,
                                     -0.013918174901761067,
                                     -0.28570038292864758], dtype=np.float32)

        self._squaresX = 5
        self._squaresY = 7
        self._squareLength = 0.035
        self._markerLength = 0.021
        self._image_size = (1000, 600)
        self._canvas = np.zeros((self._image_size[1], self._image_size[0], 3), dtype=np.uint8)

        CANVAS_NAME = "CookieProject"  
        cv2.namedWindow(CANVAS_NAME)

        self._morph_size = 8
        self._lowHue = 0  
        self._highHue = 180
        self._lowSaturation = 0
        self._highSaturation = 8
        self._lowValue = 220
        self._highValue = 255 

        self.command_sent = False 
        self.previous_time = time.time()

    def get_position(self):
        valid_pose = False

        self._vid = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
        self._vid.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self._vid.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        #board = aruco.CharucoBoard_create(self._squaresX, self._squaresY, self._squareLength, self._markerLength, dictionary)
        arucoParams = cv2.aruco.DetectorParameters_create()
        
        obj_points = np.array([[-self._markerLength / 2, self._markerLength / 2, 0],
                               [self._markerLength / 2, self._markerLength / 2, 0],
                               [self._markerLength / 2, -self._markerLength / 2, 0],
                               [-self._markerLength / 2, -self._markerLength / 2, 0]], dtype=np.float32)

        if self._vid.isOpened():
            while True:
                ret, self._canvas = self._vid.read()
                if not ret:
                    break

                ids, corners, rejected = None, None, None
                (corners, ids, rejected) = cv2.aruco.detectMarkers(self._canvas, dictionary, parameters=arucoParams)

                if ids is not None:
                    aruco.drawDetectedMarkers(self._canvas, corners, ids)
                    
                    found_marker2 = False
                    found_marker4 = False
                    self.first_corner_marker2 = None
                    self.first_corner_marker4 = None

                    for i in range(len(ids)):
                        if ids[i] == 2 and not found_marker2:
                            self.first_corner_marker2 = corners[i][0]  # Get the first corner of marker 2
                            found_marker2 = True
                            #print(f"corner {ids[i]}: {self.first_corner_marker2[0]}")
                        elif ids[i] == 4 and not found_marker4:
                            self.first_corner_marker4 = corners[i][0]  # Get the first corner of marker 4
                            found_marker4 = True
                            #print(f"corner {ids[i]}: {self.first_corner_marker4[0]}")
                        #elif ids[i] == 12:
                            #c12 = corners[i][0]
                            #print(f"corner {ids[i]}: {c12[0]}")
                        if found_marker2 and found_marker4:
                            break

                    if found_marker2 and found_marker4:
                        self.draw_offset_dots(70)

                    #rvecs, tvecs = [], []
                    #for corner in corners:
                        #ret, rvec, tvec = cv2.solvePnP(obj_points, corner, self._cameraMatrix, self._distCoeffs)
                        #rvecs.append(rvec)
                        #tvecs.append(tvec)
                    
                    #for rvec, tvec in zip(rvecs, tvecs):
                        #cv2.drawFrameAxes(self._canvas, self._cameraMatrix, self._distCoeffs, rvec, tvec, self._markerLength * 1.5, 2)
                
                self.get_contour()
                cv2.imshow("Canvas", self._canvas)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break


            self._vid.release()
            cv2.destroyAllWindows()

    def draw_offset_dots(self, offset):
        # Calculate the offset points
        self.offset_point1 = (int(self.first_corner_marker2[0][0] - 40), int(self.first_corner_marker2[0][1] + offset))
        self.offset_point2 = (int(self.first_corner_marker4[0][0] - 70), int(self.first_corner_marker4[0][1] - offset))
        self.offset_point3 = (self.offset_point1[0] - 50, self.offset_point1[1]+3)
        self.offset_point4 = (self.offset_point2[0] - 50, self.offset_point2[1]+3)

        # Draw small circles (dots) at the calculated positions
        cv2.circle(self._canvas, self.offset_point1, 5, (0, 255, 0), -1)  # Green dot
        cv2.circle(self._canvas, self.offset_point2, 5, (0, 0, 255), -1)  # Red dot
        cv2.circle(self._canvas, self.offset_point3, 5, (0, 255, 0), -1)  # Green dot
        cv2.circle(self._canvas, self.offset_point4, 5, (0, 0, 255), -1)  # Red dot

    def get_contour(self):

        current_time = time.time()
        x1, y1 = self.offset_point1
        x2, y2 = self.offset_point2
        x3, y3 = self.offset_point3
        x4, y4 = self.offset_point4

        x = min(x1, x2, x3, x4)
        y = min(y1, y2, y3, y4)
        width = max(x1, x2, x3, x4) - x
        height = max(y1, y2, y3, y4) - y
        roi = (x, y, width, height)

        hsv_image = cv2.cvtColor(self._canvas, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_image, (self._lowHue, self._lowSaturation, self._lowValue), (self._highHue, self._highSaturation, self._highValue))

        element = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mask = cv2.erode(mask, element)
        mask = cv2.dilate(mask, element)

        roi_mask = mask[y:y+height, x:x+width]

        cv2.imshow("Bin", roi_mask)

        contours, hierarchy = cv2.findContours(roi_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            self.command_sent = False
            
        if contours and not self.command_sent: # checking to see if in the ROI
            contour = contours[0]
            contour = contour + [x, y]  # Shift x and y coordinates by ROI's offset

            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                center = (cx, cy)
                if current_time - self.previous_time >= 1:
                    print(f"corner 2: {self.first_corner_marker2[0]}")
                    center = (self.first_corner_marker2[0][0] -center[0], center[1] -self.first_corner_marker2[0][1])
                    self.send_command(center) #sending the center of the cor
                    
                    self.previous_time = time.time()
                     
        
            #cv2.circle(self._canvas, center, 5, (255, 0, 0), -1)  # Blue dot at cent

            # Show the canvas with contours
            #cv2.imshow("Canvas with Contours", self._canvas)

    def send_command(self, command):

        if self._serial_port.is_open:
            command = f"{command[0]},{command[1]}\n"
            self._serial_port.write(command.encode('utf-8'))
            print(f"Sent command: {command}")
        self.command_sent = True
                 
def main():
    Cpi = CPi()
    try:
        while True:
            Cpi.get_position()
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("Process interrupted")

    finally:
        # Ensure the serial port and windows are properly closed
        if Cpi._serial_port.is_open:
            Cpi._serial_port.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
