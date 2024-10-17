from cv2 import aruco
import cv2
import math
import numpy as np
import matplotlib.pyplot as plt
import random as rd
import scipy.misc as scm
from time import perf_counter
from numpy import linalg as la
import serial
import time


aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = aruco.DetectorParameters()
# matrice camera
cameraMatrix = np.array([[4.78870006e+03, 0.00000000e+00, 5.52691234e+02],
                        [0.00000000e+00, 4.76397206e+03, 4.46541770e+02],
                        [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

distCoeffs = np.array([[-1.70046099e+00, -3.95398448e+00, -4.86193935e-02,  6.09110906e-02, -1.35621767e+02]])

Liste_avant = []
Liste_coord = []

initial = 0

def newpixtoref(P,origine):  # origine au choix
    dx = P[0] - origine[0]
    dy = P[1] - origine[1]
    x = dx * correct
    y = -dy * correct
    return np.array([x, y])


def newreftopix(C, origine):
    px = C[0]/correct + origine[0]
    py = -C[1]/correct + origine[1]
    return np.array([px, py])


def calculate_angle(pointrobot1, pointrobot2, cible):
    vector1 = (-pointrobot1[0] + pointrobot2[0], -pointrobot1[1] + pointrobot2[1])
    vector2 = (cible[0] - pointrobot1[0], cible[1] - pointrobot1[1])

    # angle rad
    angle_radians = math.atan2(vector2[1], vector2[0]) - math.atan2(vector1[1], vector1[0])

    # angle degrees
    angle_degrees = math.degrees(angle_radians)
    while angle_degrees > 180 or angle_degrees < -180:
        if angle_degrees < 180:
            angle_degrees += 360
        else :
            angle_degrees -= 360
    return angle_degrees

def connect_to_arduino(port, baud_rate=9600):
    try:
        ser = serial.Serial(port, baud_rate, timeout=0)
        print(f"Connected to Arduino on port {port} at {baud_rate} baud.")
        return ser
    except serial.SerialException as e:
        print(f"Error: {e}")
        return None


def send_command(ser, command):
    ser.write(command.encode())
    ser.flushInput()
    #time.sleep(0.5)  # Add a delay to allow the Arduino to process the command


# Replace 'COMx' with the actual port your Arduino is connected to (e.g., 'COM3' on Windows, '/dev/ttyUSB0' on Linux)
arduino_port = 'COM5'
arduino_baud_rate = 9600

arduino = connect_to_arduino(arduino_port, arduino_baud_rate)

# Function to detect tennis ball
count_0 = 0
def detect_tennis_ball(frame):
    # couleur, min max (in HSV)
    # lower_bound = np.array([20, 100, 100])
    # upper_bound = np.array([30, 255, 255])
    # valeurs experimentales(testés par hugo)
    lower_bound = np.array([0.10 * 256, 0.20 * 256, 0.5 * 256])
    upper_bound = np.array([0.15 * 256, 0.60 * 256, 1 * 256])

    # Convertion image hsv
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # recherche de contour
    mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # trouver le max
        max_contour = max(contours, key=cv2.contourArea)
        moments = cv2.moments(max_contour)

        if moments['m00'] != 0:
            # Coordonnés du centre
            centroid_x = int(moments['m10'] / moments['m00'])
            centroid_y = int(moments['m01'] / moments['m00'])

            return centroid_x, centroid_y

    return None, None


prev_marker_center = None
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

# réglage pour la convertion
marker_dimensions_cm = 6 ##float(input("Enter the marker dimensions in centimeters: "))

a = 4 #int(input("identifiant premier coin (down left) :"))
b = 5 #int(input("identifiant coin 2 (down right) :"))
c = 6 #int(input("identifiant coin 3 (up right) :"))
d = 7 #int(input("identifiant coin 4 (up left) :"))
e = 15 #int(input("identifiant arrière robot :"))
f = 8 #int(input("identifiant avant robot :"))
g = True #indentifiant arrivée à définir

t = perf_counter()

centers = np.zeros((20, 3))  # coord centres (cm)
centerspix = np.zeros((20, 2))  # coord centres en pixels

for i in range(0, 20):
    centers[i][0] = i


needtocalculatetrajectory = True

if arduino:
    while True:
        #print(initial)
        ret, frame = cap.read()
        #initial += 1
        #if initial > 5:
        #    time.sleep(0.1)
        if not ret:
            break
        t0 = perf_counter()
        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

        if ids is not None:
            # print(ids)
            # dessiner les contours d'aruco
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            for i in range(len(ids)):
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], marker_dimensions_cm, cameraMatrix,
                                                                    distCoeffs)
                cv2.drawFrameAxes(frame, cameraMatrix, distCoeffs, rvec, tvec, marker_dimensions_cm * 0.5)

                if ids[i][0] < 20:
                    h = ids[i][0]
                    # print("##########  ",a," ##########")
                    translation = tvec[0][0]
                    # print(translation)
                    # centre du marqueur
                    marker_center = np.mean(corners[i][0], axis=0)
                    centers[h][1] = translation[0]
                    centers[h][2] = translation[1]
                    centerspix[h][0] = marker_center[0]
                    centerspix[h][1] = marker_center[1]
                    # centers.append({
                    #    'x': translation[0],
                    #   'y': translation[1]})

                # ligne qui connecte les centres des marqueurs
                if prev_marker_center is not None:
                    cv2.line(frame, (int(prev_marker_center[0]), int(prev_marker_center[1])),
                             (int(marker_center[0]), int(marker_center[1])), (0, 255, 0), 2)

                    # mesure se fait en pixel sur l'image
                    length_px = np.linalg.norm(marker_center - prev_marker_center)
                    # facteur correctif variable
                    correct = marker_dimensions_cm / np.linalg.norm(corners[i][0][0] - corners[i][0][1])
                    # dimension réelle
                    length_cm = length_px * correct

                    # affichage des distances
                    cv2.putText(frame, f'{length_cm:.2f} cm', (int((marker_center[0] + prev_marker_center[0]) / 2),
                                                               int((marker_center[1] + prev_marker_center[1]) / 2)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                prev_marker_center = marker_center

        #t1 = perf_counter()
        #print('t1-t0 ', t1 - t0)
        # time.sleep(0.5)
        # Detect tennis ball
        ball_x, ball_y = detect_tennis_ball(frame)


        if ball_x is not None and ball_y is not None:
            # afficher un cercle pour la position de la balle
            cv2.circle(frame, (ball_x, ball_y), 10, (0, 0, 255), -1)

            # affichage coordonné
            cv2.putText(frame, f'Balle: ({ball_x}, {ball_y})', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        cv2.imshow('Frame', frame)

        extrait = centers.tolist()
        identifiants = dict()

        for x in extrait:
            if x[1] != 0.0 and x[2] != 0.0:
                x[0] = int(x[0])
                identifiants[x[0]] = [x[1], x[2]]
        angle = 0

        #if ball_x is not None and ball_y is not None:
        if 1==1:
            if needtocalculatetrajectory:
                needtocalculatetrajectory = False
                startpoint = (centerspix[e] + centerspix[f]) / 2
                #endpoint = centerspix[g]
                endpoint = np.array([173,280])

                ref0SP = newpixtoref(startpoint, startpoint)
                ref0EP = newpixtoref(endpoint, startpoint)
                ref0BALL = newpixtoref([ball_x, ball_y], startpoint)

                # plt.plot([ref0SP[0], ref0EP[0], ref0BALL[0]], [ref0SP[1], ref0EP[1], ref0BALL[1]], 'o')
                # plt.show()

                theta1 = math.radians(calculate_angle(ref0SP, [ref0EP[0], ref0SP[1]], ref0EP))
                Passage = np.array([[np.cos(theta1), -np.sin(theta1)],
                                    [np.sin(theta1), np.cos(theta1)]])


                def B0toB1(X):
                    X.shape = (np.size(X), 1)
                    return np.dot(la.inv(Passage), X)


                def B1toB0(X):
                    X.shape = (np.size(X), 1)
                    return np.dot(Passage, X)


                ref1SP = B0toB1(ref0SP)
                ref1EP = B0toB1(ref0EP)
                ref1BALL = B0toB1(ref0BALL)

                # plt.plot([ref1SP[0], ref1EP[0], ref1BALL[0]], [ref1SP[1], ref1EP[1], ref1BALL[1]], 'o')
                # plt.show()

                L = ref1EP[0][0]

                m = 2  # marge
                n = 50  # nb de points
                d = 40  # para traj3

                # balle
                xb = ref1BALL[0][0]
                yb = ref1BALL[1][0]

                # plt.plot(xb, yb, 'o', color='g')

                vmaxrob = 20  # vitesse max du robot
                amax = 30  # accéleration normale max que le robot peut subir


                # calcul du rayon de courbure
                def RC(f, x):
                    if scm.derivative(f, x, n=2) == 0:
                        return 100
                    return (1 + scm.derivative(f, x) ** 2) ** (3 / 2) / scm.derivative(f, x, n=2)


                # tableau des vitesses
                def vmax(f, X):
                    V = np.zeros(len(X) - 1)
                    for i in range(len(X) - 1):
                        vc = (amax * RC(f, X[i])) ** (1 / 2)
                        if vc < vmaxrob:
                            V[i] = vc
                        else:
                            V[i] = vmaxrob
                    return V


                # temps de parcours
                def temps(f, X):
                    v = vmax(f, X)
                    t = 0
                    for i in range(len(X) - 1):
                        t += ((f(X[i + 1]) - f(X[i])) ** 2 + (X[i + 1] - X[i]) ** 2) ** (1 / 2) / v[i]
                    return t


                # lil tool
                def FtoT(f, X):
                    T = np.zeros(len(X))
                    for i in range(len(X)):
                        T[i] = f(X[i])
                    return T


                X = np.linspace(0, L, n)

                # trajectoire 1      ordre 2 + droite
                dy1 = -yb / (L - xb)

                M1 = np.array([[xb ** 2, xb, 1],
                               [0, 0, 1],
                               [2 * xb, 1, 0]], dtype=float)

                N1 = np.array([yb, 0, dy1], dtype=float)

                Coeff = la.solve(M1, N1)

                A1 = Coeff[0]
                B1 = Coeff[1]
                C1 = Coeff[2]


                def f1(x):
                    if x < xb:
                        return A1 * x ** 2 + B1 * x + C1
                    else:
                        return dy1 * (x - xb) + yb


                t1 = temps(f1, X)
                Tmin = t1
                fmin = f1

                # trajectoire 2     droite + ordre 2
                dy2 = yb / xb

                M2 = np.array([[xb ** 2, xb, 1],
                               [L ** 2, L, 1],
                               [2 * xb, 1, 0]], dtype=float)

                N2 = np.array([yb, 0, dy2], dtype=float)

                Coeff2 = la.solve(M2, N2)

                A2 = Coeff2[0]
                B2 = Coeff2[1]
                C2 = Coeff2[2]


                def f2(x):
                    if x < xb:
                        return dy2 * (x - xb) + yb
                    else:
                        return A2 * x ** 2 + B2 * x + C2


                t2 = temps(f2, X)
                if t2 < Tmin:
                    Tmin = t2
                    fmin = f2

                # trajectoire 3     droite + ordre 3 + droite
                if xb > d:
                    dy3_1 = -yb / (L - xb)
                    dy3_2 = yb / (xb - d)

                    M3 = np.array([[xb ** 3, xb ** 2, xb, 1],
                                   [(xb - d) ** 3, (xb - d) ** 2, xb - d, 1],
                                   [3 * xb ** 2, 2 * xb, 1, 0],
                                   [3 * (xb - d) ** 2, 2 * (xb - d), 1, 0]], dtype=float)

                    N3 = np.array([yb, yb, dy3_1, dy3_2], dtype=float)

                    Coeff3 = la.solve(M3, N3)

                    A3 = Coeff3[0]
                    B3 = Coeff3[1]
                    C3 = Coeff3[2]
                    D3 = Coeff3[3]


                    def f3(x):
                        if x < xb - d:
                            return dy3_2 * x
                        elif x > xb:
                            return dy3_1 * (x - xb) + yb
                        else:
                            return A3 * x ** 3 + B3 * x ** 2 + C3 * x + D3


                    t3 = temps(f3, X)
                    if t3 < Tmin:
                        Tmin = t3
                        fmin = f3

                # trajectoire 4     ordre 2

                M4 = np.array([[xb ** 2, xb, 1],
                               [L ** 2, L, 1],
                               [0, 0, 1]], dtype=float)

                N4 = np.array([yb, 0, 0], dtype=float)

                Coeff4 = la.solve(M4, N4)

                A4 = Coeff4[0]
                B4 = Coeff4[1]
                C4 = Coeff4[2]


                def f4(x):
                    return A4 * x ** 2 + B4 * x + C4


                t4 = temps(f4, X)
                if t4 < Tmin:
                    Tmin = t4
                    fmin = f4

                Tab1 = FtoT(fmin, X)
                X0 = np.zeros(n)
                F0 = np.zeros(n)
                for i in range(n):
                    B = B1toB0(np.array([X[i], Tab1[i]]))
                    X0[i], F0[i] = B[0][0], B[1][0]

                plt.plot(X0, F0, 'y')
                #plt.plot(X, FtoT(fmin, X), 'y')

                distSPtoBALL = np.linalg.norm(ref0SP - ref0BALL)
                distEPtoBALL = np.linalg.norm(ref0EP - ref0BALL)

                precision = 20
                n1 = int(distSPtoBALL // precision)
                n2 = int(distEPtoBALL // precision)

                nbpoints = int(n1 + n2 + 3)

                Discret = np.zeros((nbpoints, 2))
                Discret[0] = np.array([ref0SP[0][0], ref0SP[1][0]])
                for i in range(n1):
                    xpt = (i + 1) / (n1 + 1) * ref1BALL[0][0]
                    Pt = B1toB0(np.array([xpt, fmin(xpt)]))
                    Discret[i + 1] = np.array([Pt[0][0], Pt[1][0]])
                Discret[n1 + 1] = np.array([ref0BALL[0][0], ref0BALL[1][0]])
                for i in range(n2):
                    xpt = ref1BALL[0][0] + (i + 1) / (n2 + 1) * (ref1EP[0][0] - ref1BALL[0][0])
                    Pt = B1toB0(np.array([xpt, fmin(xpt)]))
                    Discret[i + n1 + 2] = np.array([Pt[0][0], Pt[1][0]])
                Discret[nbpoints - 1] = np.array([ref0EP[0][0], ref0EP[1][0]])
                # print(Discret)

                DiscretPix = np.zeros((nbpoints, 2))  # this is the one
                for i in range(nbpoints):
                    DiscretPix[i] = newreftopix(Discret[i], startpoint)
                print(DiscretPix)

                nextpoint = 1
                """
                DiscretX = np.zeros(nbpoints)
                DiscretF = np.zeros(nbpoints)

                for i in range(nbpoints):
                    DiscretX[i] = Discret[i][0]
                    DiscretF[i] = Discret[i][1]

                plt.plot(DiscretX, DiscretF, 'ob')
                plt.plot(ref0BALL[0], ref0BALL[1], '-r')
                plt.show()
                """
                #plt.plot(ref0BALL[0], ref0BALL[1], '-r')
            distance = correct * np.linalg.norm((centerspix[e] + centerspix[f]) / 2 - DiscretPix[nextpoint])

            if perf_counter() - t > 0.3:
                Liste_coord.append((centerspix[e] + centerspix[f]) / 2)
                t = perf_counter()

            if perf_counter() - t > 1.5:
                #Liste_coord.append((centerspix[e] + centerspix[f]) / 2)

                Liste_avant.append(centerspix[f])
                print(Liste_coord)
                #angle1 = calculate_angle(centerspix[e], centerspix[f], [ball_x, ball_y])
                angle1 = calculate_angle((centerspix[e]+centerspix[f])/2, centerspix[f], DiscretPix[nextpoint])
                if abs(angle1 - angle) < 10:
                    angle = None
                    #print("no change in angle")
                else:
                    angle = angle1
                t = perf_counter()
                if angle == None:
                    string = "+000:"
                    distint = str(int(distance))
                    for i in range(3 - len(distint)):
                        string += "0"
                    string += distint
                else :
                    string = ""
                    if angle < 0:
                       string += "-"
                    else :
                        string += "+"
                    angleint = str(int(abs(angle)))
                    for i in range(3-len(angleint)):
                        string += "0"
                    string += angleint + ":000"
                if distance < 10:
                    if nextpoint + 1 == len(DiscretPix):
                        Liste_coord.append((centerspix[e] + centerspix[f]) / 2)
                        Liste_avant.append(centerspix[f])
                        break
                    else:
                        nextpoint += 1
                print(string)
                count_0 += 1
                print("nb d'instructions envoyes: ", count_0)
                send_command(arduino, string)
                maxint = max(int(string[1:4]), int(string[5:]))
                if maxint < 20:
                    delayo = 0.2
                else:
                    delayo = 0.007 * (maxint + 10)
                time.sleep(delayo)
            #print(pixtoref([ball_x, ball_y]))

        # interrompre le programme avec 'q'

        #t3 = perf_counter()
        #print("t3-t2 ",t3-t2)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


def dessiner(Liste_coord, c):
    Tabreel = np.zeros((len(Liste_coord),2))
    TabreelX = np.zeros(len(Liste_coord))
    TabreelY = np.zeros(len(Liste_coord))
    for i in range(len(Liste_coord)):
        reel = newpixtoref([Liste_coord[i][0],Liste_coord[i][1]], startpoint)
        Tabreel[i][0] = reel[0]
        TabreelX[i] = reel[0]
        Tabreel[i][1] = reel[1]
        TabreelY[i] = reel[1]

    DiscretX = np.zeros(nbpoints)
    DiscretF = np.zeros(nbpoints)

    for i in range(nbpoints):
        DiscretX[i] = Discret[i][0]
        DiscretF[i] = Discret[i][1]

    plt.plot(TabreelX, TabreelY, c)
    plt.plot(DiscretX, DiscretF, 'ob')
#plt.plot(ref0BALL[0], ref0BALL[1], 'og')


dessiner(Liste_coord, '-g')
#dessiner(Liste_avant, '-r')
plt.axis("equal")
plt.show()

# Release the video capture object and close the OpenCV windows
cap.release()
cv2.destroyAllWindows()

