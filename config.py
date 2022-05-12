"""Config file for 7 elements Tangram puzzle."""

import math

# in 7 Tangram we always use all the elements
from enum import Enum, IntEnum

ELEMENT_NUM = 7
TANGRAM_S = 8.
PI = math.pi
SQRT_2 = 2 ** 0.5

# number of angles tried to place elements
# for example, 8 means every rotation is 2 * pi / 8 = pi / 4
ANGLE_NUM = 8

# total state number for each element
TRIANGLE_STATE = 3  # p0, p1, p2
SQUARE_STATE = 1  # p0
PARALLELOGRAM_STATE = 4  # p0, p1, inverse_p0, inverse_p1

RESULT_PATH = "result/"

class ConcaveConvexEnum(Enum):
    concave = 1
    convex = 2


class TangramElementEnum(IntEnum):
    large_triangle = 1
    square = 2
    parallelogram = 3
    medium_triangle = 4
    small_triangle = 5


TRIANGLE_ELEMENT_STATE_DIC = {
    TangramElementEnum.large_triangle: TRIANGLE_STATE,
    TangramElementEnum.medium_triangle: TRIANGLE_STATE,
    TangramElementEnum.small_triangle: TRIANGLE_STATE,
    TangramElementEnum.square: SQUARE_STATE,
    TangramElementEnum.parallelogram: PARALLELOGRAM_STATE
}

# standard size of elements in a set of Tangram, the first element is its area
STANDARD_SIZE = {
    TangramElementEnum.large_triangle: [2., 2 * SQRT_2, 2., 2.],
    TangramElementEnum.medium_triangle: [1., 2., SQRT_2, SQRT_2],
    TangramElementEnum.small_triangle: [0.5, SQRT_2, 1., 1.],
    TangramElementEnum.square: [1., 1., 1., 1., 1.],
    TangramElementEnum.parallelogram: [1., SQRT_2, 1., SQRT_2, 1.]
}

# colors for drawing elements of Tangram
COLORS = [
    (int(255. * i / ELEMENT_NUM),
     int(128. + 255. * i / ELEMENT_NUM) % 255,
     int(255. - 255. * i / ELEMENT_NUM))
    for i in range(ELEMENT_NUM)
]
