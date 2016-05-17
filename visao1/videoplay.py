#!/usr/bin/python
#-*- coding:utf-8 -*-

import cv2
import numpy as np
import sys
import math

path = "hall_box_battery (1).mp4"
cap = cv2.VideoCapture(path)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    cdst = cv2.Canny(frame, 50, 200) # aplica o detector de bordas de Canny à imagem src

    if True: # HoughLinesP
        lines = cv2.HoughLinesP(cdst, 1, math.pi/180.0, 40, np.array([]), 50, 10)
        a,b,c = lines.shape
        for i in range(a):
            # Faz uma linha ligando o ponto inicial ao ponto final, com a cor vermelha (BGR)
            cv2.line(cdst, (lines[i][0][0], lines[i][0][1]), (lines[i][0][2], lines[i][0][3]), (0, 0, 255), 3)

    else:    # HoughLines
        # Esperemos nao cair neste caso
        lines = cv2.HoughLines(cdst, 1, math.pi/180.0, 50, np.array([]), 0, 0)
        a,b,c = lines.shape
        for i in range(a):
            rho = lines[i][0][0]
            theta = lines[i][0][1]
            a = math.cos(theta)
            b = math.sin(theta)
            x0, y0 = a*rho, b*rhos
            pt1 = ( int(x0+1000*(-b)), int(y0+1000*(a)) )
            pt2 = ( int(x0-1000*(-b)), int(y0-1000*(a)) )
            cv2.line(cdst, pt1, pt2, (0, 0, 255), 3, cv2.CV_AA)
        print("Used old vanilla Hough transform")

    cv2.imshow("source", frame)
    cv2.imshow("detected lines", cdst)


# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()