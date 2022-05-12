"""This File contains some utility functions"""
import copy
import math

import cv2
import numpy as np
from PyQt5 import QtGui
from PyQt5.QtCore import QSize

import config


def get_draw_area_size(elements):
    points = []
    max_x = 0
    max_y = 0
    for element in elements:
        points.extend(element.points)
    for point in points:
        point_x = abs(point.x)
        point_y = abs(point.y)
        if max_x < point_x:
            max_x = point_x
        if max_y < point_y:
            max_y = point_y
    return max_x, max_y


def get_final_result(elements):
    max_x, max_y = get_draw_area_size(elements)
    image = np.zeros((int((max_x + 5) * 2), int((max_y + 5) * 2), 3), np.uint8)
    for i in range(len(elements)):
        points = get_element_points(elements[i], max_x, max_y)
        image = draw_polygon(image, points, color=config.COLORS[i])
    return image


def get_element_points(element, x_offset, y_offset):
    points = []
    for point in element.points:
        points.append(point.get_coordinate())
    points += np.array([x_offset, y_offset])
    points = np.round(np.array([points])).astype(np.int32)
    points[:, :, :] = points[:, :, [1, 0]]
    return points


def draw_polygon(image, points, color=128):
    """Draw polygon on an image.
    Can be used to transfer state when you fill an element on the mask.
    """
    img = copy.deepcopy(image)
    cv2.fillPoly(img, points, color)

    return img


def get_distance_point_to_point(p1, p2):
    """Get the distance between 2 points."""

    return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)


def get_distance_point_to_segment(p, s):
    """Get the distance between 1 point and 1 segment."""
    area = triangle_area([p, s.p1, s.p2])
    d = get_distance_point_to_point(s.p1, s.p2)
    h = 2. * area / d

    return h


def get_slope(p1, p2):
    """Get the slope of the line going through 2 points."""
    x1, y1 = p1.get_coordinate()
    x2, y2 = p2.get_coordinate()
    slope = (y2 - y1) / (x2 - x1) if x2 - x1 != 0 else None

    return slope


def get_intercept(p1, p2):
    """Get the intercept of the line going through 2 points."""
    x1, y1 = p1.get_coordinate()
    x2, y2 = p2.get_coordinate()
    intercept = (x2 * y1 - x1 * y2) / (x2 - x1) if x2 - x1 != 0 else None

    return intercept


def get_line(p1, p2):
    """Get the slope and intercept of the line going through 2 points."""
    slope = get_slope(p1, p2)
    intercept = get_intercept(p1, p2)

    return slope, intercept


def right_sin(p1, p2):
    delta_x = p1.x - p2.x
    if delta_x == 0:
        return 1
    elif delta_x < 0:
        delta_x = -delta_x
        delta_y = p2.y - p1.y
    else:
        delta_y = p1.y - p2.y
    distance_sqrt = math.sqrt(delta_x * delta_x + delta_y * delta_y)
    return delta_y / distance_sqrt


def on_one_line(points):
    sin_1 = right_sin(points[0], points[1])

    for j in range(2, len(points)):
        sin_j = right_sin(points[0], points[j])
        if math.fabs(sin_1 - sin_j) > 10 ** -4:
            return False
    return True


def cross_product(v1, v2):
    """Calculate the cross product of 2 vectors as (x1 * y2 - x2 * y1)."""

    return v1.x * v2.y - v2.x * v1.y


def inner_product(v1, v2):
    """Calculate the inner product of 2 vectors as (x1 * x2 + y1 * y2)."""

    return v1.x * v2.x + v1.y * v2.y


def triangle_area(points):
    """Calculate the area of a triangle."""
    a = get_distance_point_to_point(points[0], points[1])
    b = get_distance_point_to_point(points[1], points[2])
    c = get_distance_point_to_point(points[2], points[0])
    p = (a + b + c) / 2.
    s = (p * (p - a) * (p - b) * (p - c)) ** 0.5

    return s
