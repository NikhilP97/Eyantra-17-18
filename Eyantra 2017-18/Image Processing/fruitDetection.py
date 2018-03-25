import cv2
import numpy as np


'''
*Function Name: image_processing()
*Input: None
*Output: if fruit detected, return Fruit Name, Size; else return Q, Q
*Logic: This function is used to detect the fruit and size of the fruit by using OpenCv. If fruit detected it returns 
        the Fruit initial letter and Size initial letter
*Example Call: image_processing(); If Large Apple detected, output --> 'A', 'L'
'''

def image_processing():
    # HSV Range Values
    lower_apple = np.array([128, 23, 108])
    upper_apple = np.array([169, 255, 255])
    lower_berry = np.array([107, 94, 151])
    upper_berry = np.array([131, 206, 255])
    lower_orange = np.array([126, 14, 239])
    upper_orange = np.array([188, 96, 255])

    # Area of Each Fruit
    orange_small_area = 9904.0
    orange_large_area = 25000.0
    max_orange_area = 60000.0

    max_apple = 28000.0  # og --> 75419.0 # large
    large_apple = 8857.0 # medium
    small_apple = 5000.0 # small

    max_berry = 28000.0 # large
    medium_berry = 13000
    small_berry = 3000.0
    square_area = 144564.0

    camera_port = 1 # Using Webcam, 0 --> default
    # noinspection PyArgumentList
    cap = cv2.VideoCapture(camera_port)
    timer = 0
    # Repeat for 70 frames
    while timer < 70:
        ret, img = cap.read()
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        #------------------------------------------ apple--------------------------------------------------------------#
        mask = cv2.inRange(hsv, lower_apple, upper_apple)
        cv2.imshow('apple mask ', mask)
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            contour_areas = []
            for i in range(0, len(contours)):
                area = cv2.contourArea(contours[i])
                contour_areas.append((area, i))
            contour_areas.sort()
            max_cnt_index = contour_areas[-1][1]
            cnt = contours[max_cnt_index]
            area = cv2.contourArea(contours[max_cnt_index])
            # print 'area of apple : ', area
            if 10 < area < max_apple:
                M = cv2.moments(contours[max_cnt_index])
                centre_x = int(M["m10"] / M["m00"])
                centre_y = int(M["m01"] / M["m00"])
                if len(contours[max_cnt_index]) > 5:
                    (a, b), (h, w), angle = cv2.fitEllipse(
                        contours[max_cnt_index])  # function return (x,y) position and (h,w) dimensions
                    # of rectangle in which ellipse is inscribed
                    major_axis = h / 2
                    minor_axis = w / 2
                    ellipse_area = 3.14159 * (major_axis) * (minor_axis)  # Area of ellipse = pi*a*
                    if ellipse_area > 0:
                        ellipse_extent = float(area) / ellipse_area
                        (a, b), radius = cv2.minEnclosingCircle(contours[max_cnt_index])
                        center = (int(a), int(b))
                        radius = int(radius)
                        cir_area = 3.14159 * radius * radius
                        if cir_area > 0:
                            cir_extent = float(area) / cir_area
                            if ellipse_extent > 0.90 and 0.79 < cir_extent < 0.90:
                                print 'APPLE'
                                font = cv2.FONT_HERSHEY_SIMPLEX
                                cv2.putText(img, "apple", (centre_x, centre_y + 15), font, 0.6, (0, 0, 0), 2,
                                            cv2.LINE_AA)
                                cv2.drawContours(img, [cnt], 0, (0, 255, 0), 3)
                                if area > large_apple:
                                    cv2.putText(img, "large", (centre_x - 50, centre_y + 15), font, 0.6, (0, 0, 0), 2,
                                                cv2.LINE_AA)
                                    print ' LARGE'
                                    return 'A', 'L'
                                elif area < small_apple:
                                    cv2.putText(img, "small", (centre_x - 50, centre_y + 15), font, 0.6, (0, 0, 0), 2,
                                                cv2.LINE_AA)
                                    print ' SMALL'
                                    return 'A', 'S'
                                else:
                                    cv2.putText(img, "medium", (centre_x - 60, centre_y + 15), font, 0.6, (0, 0, 0), 2,
                                                cv2.LINE_AA)
                                    print ' MEDIUM'
                                    return 'A', 'M'

        # ------------------------------------------------blueberry---------------------------------------------------#
        mask = cv2.inRange(hsv, lower_berry, upper_berry)
        cv2.imshow('berry mask ', mask)
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            contour_areas = []
            for i in range(0, len(contours)):
                area = cv2.contourArea(contours[i])
                contour_areas.append((area, i))
            contour_areas.sort()
            # cnt = None
            max_cnt_index = contour_areas[-1][1]
            cnt = contours[max_cnt_index]
            area = cv2.contourArea(contours[max_cnt_index])
            # print 'area of blueberry : ', area
            if 10 < area < max_berry:
                M = cv2.moments(contours[max_cnt_index])
                centre_x = int(M["m10"] / M["m00"])
                centre_y = int(M["m01"] / M["m00"])
                if len(contours[max_cnt_index]) > 5:
                    (a, b), (h, w), angle = cv2.fitEllipse(
                        contours[
                            max_cnt_index])  # function return (x,y) position and (h,w) dimensions
                    # of rectangle in which ellipse is inscribed
                    major_axis = h / 2
                    minor_axis = w / 2
                    ellipse_area = 3.14159 * (major_axis) * (minor_axis)  # Area of ellipse = pi*a*
                    if ellipse_area > 0:
                        ellipse_extent = float(area) / ellipse_area
                        (a, b), radius = cv2.minEnclosingCircle(contours[max_cnt_index])
                        center = (int(a), int(b))
                        radius = int(radius)
                        cir_area = 3.14159 * radius * radius
                        if cir_area > 0:
                            cir_extent = float(area) / cir_area
                            if ellipse_extent > 0.90 and cir_extent > 0.85:
                                print 'BLUEBERRY'
                                font = cv2.FONT_HERSHEY_SIMPLEX
                                cv2.putText(img, "blueberry", (centre_x, centre_y + 15), font, 0.6, (0, 0, 0), 2,
                                            cv2.LINE_AA)
                                cv2.drawContours(img, [cnt], 0, (0, 255, 0), 3)
                                if area > max_berry - 50000.0:
                                    cv2.putText(img, "large", (centre_x - 60, centre_y + 15), font, 0.6, (0, 0, 0), 2,
                                                cv2.LINE_AA)
                                    print ' LARGE'
                                    return 'B', 'L'
                                elif area < small_berry:
                                    cv2.putText(img, "small", (centre_x - 60, centre_y + 15), font, 0.6, (0, 0, 0), 2,
                                                cv2.LINE_AA)
                                    print ' SMALL'
                                    return 'B', 'S'
                                else:
                                    cv2.putText(img, "medium", (centre_x - 60, centre_y + 15), font, 0.6, (0, 0, 0), 2,
                                                cv2.LINE_AA)
                                    print ' MEDIUM'
                                    return 'B', 'M '

        # ------------------------------------------------------orange-------------------------------------------------#
        mask = cv2.inRange(hsv, lower_orange, upper_orange)
        cv2.imshow('orange mask', mask)
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            contour_areas = []
            for i in range(0, len(contours)):
                area = cv2.contourArea(contours[i])
                contour_areas.append((area, i))
            contour_areas.sort()
            # cnt = None
            max_cnt_index = contour_areas[-1][1]
            cnt = contours[max_cnt_index]
            area = cv2.contourArea(contours[max_cnt_index])
            # print 'area of orange : ', area
            if area > 10:
                M = cv2.moments(contours[max_cnt_index])
                centre_x = int(M["m10"] / M["m00"])
                centre_y = int(M["m01"] / M["m00"])
                if len(contours[max_cnt_index]) > 5:
                    (a, b), (h, w), angle = cv2.fitEllipse(
                        contours[
                            max_cnt_index])  # function return (x,y) position and (h,w) dimensions
                    # of rectangle in which ellipse is inscribed
                    major_axis = h / 2
                    minor_axis = w / 2
                    ellipse_area = 3.14159 * (major_axis) * (minor_axis)  # Area of ellipse = pi*a*
                    if ellipse_area > 0:
                        ellipse_extent = float(area) / ellipse_area
                        (a, b), radius = cv2.minEnclosingCircle(contours[max_cnt_index])
                        center = (int(a), int(b))
                        radius = int(radius)
                        cir_area = 3.14159 * radius * radius
                        if cir_area > 0:
                            cir_extent = float(area) / cir_area
                            if ellipse_extent > 0.90 and cir_extent > 0.85:
                                print 'ORANGE'
                                font = cv2.FONT_HERSHEY_SIMPLEX
                                cv2.putText(img, "orange", (centre_x, centre_y + 15), font, 0.6, (0, 0, 0), 2,
                                            cv2.LINE_AA)
                                cv2.drawContours(img, [cnt], 0, (0, 255, 0), 3)
                                if area < orange_small_area:
                                    cv2.putText(img, "small", (centre_x - 50, centre_y + 15), font, 0.6, (0, 0, 0), 2,
                                                cv2.LINE_AA)
                                    print ' SMALL'
                                    return 'O', 'S'
                                elif area > orange_large_area:
                                    cv2.putText(img, "large", (centre_x - 50, centre_y + 15), font, 0.6, (0, 0, 0),
                                                2,
                                                cv2.LINE_AA)
                                    print ' LARGE'
                                    return 'O', 'L'
                                else:
                                    cv2.putText(img, "medium", (centre_x - 50, centre_y + 15), font, 0.6, (0, 0, 0),
                                                2,
                                                cv2.LINE_AA)
                                    print ' MEDIUM'
                                    return 'O', 'M'
        #----------------------------------NO FRUIT--------------------------------------------------------------------#
        else:
            # print 'NO FRUIT'
            font = cv2.FONT_HERSHEY_SIMPLEX
            # cv2.putText(img, "no fruit", (centre_x, centre_y + 15), font, 0.6, (0, 0, 0), 2, cv2.LINE_AA)
            # cv2.drawContours(img, [cnt], 0, (0, 255, 0), 3)
        timer += 1
        cv2.imshow('main loop', img)
        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break

    print 'NO FRUIT DETECTED '
    cap.release()
    cv2.destroyAllWindows()
    return 'Q', 'Q'








