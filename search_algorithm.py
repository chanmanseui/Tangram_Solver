import copy
import multiprocessing
import os
import profile
import time
from math import cos, sin
from multiprocessing import Queue, Process

import cv2
import numpy

import config
import utils
from config import TangramElementEnum, ANGLE_NUM, TRIANGLE_ELEMENT_STATE_DIC, PI, ConcaveConvexEnum
from tangram_element import Point, Triangle, Square, Parallelogram, Segment
from utils import get_final_result


class Solve:

    def __init__(self):
        self.corners = []
        self.edges = []
        self.error_threshold = 0.01
        self.unit_length = 620 / 8
        self.concave_convex_enum = ConcaveConvexEnum.convex
        # mark some elements of Tangram
        self.unused_elements_enum = [
            TangramElementEnum.medium_triangle,
            TangramElementEnum.square,
            TangramElementEnum.small_triangle,
            TangramElementEnum.large_triangle,
            TangramElementEnum.small_triangle,
            TangramElementEnum.large_triangle,
            TangramElementEnum.parallelogram
        ]
        self.used_elements = []

    def used_element_check(self):
        result = 0
        for element in self.used_elements:
            if element.element_enum == TangramElementEnum.large_triangle:
                result += 1
            elif element.element_enum == TangramElementEnum.medium_triangle:
                result += 10
            elif element.element_enum == TangramElementEnum.small_triangle:
                result += 100
            elif element.element_enum == TangramElementEnum.square:
                result += 1000
            else:
                result += 10000
        return result

    def __hash__(self):
        result = 0
        for x in self.used_elements:
            result = result ^ hash(x)
        return result

    def __eq__(self, other):
        if len(self.used_elements) != len(other.used_elements):
            return False
        if self.used_element_check() != other.used_element_check():
            return False
        for x in self.used_elements:
            equal_flag = False
            for y in other.used_elements:
                if x == y:
                    equal_flag = True
                    break
            if not equal_flag:
                return False
        return True

    def init(self, element_enum=TangramElementEnum.medium_triangle):
        self.corners.append(Point.get_instance(x=0., y=0.))
        new_element = self.__try_element(element_enum, 0, 0, 0)
        self.__update(new_element)

    def __update(self, element):
        new_element_enum = element.element_enum
        assert new_element_enum in self.unused_elements_enum
        self.unused_elements_enum.remove(new_element_enum)
        self.used_elements.append(element)
        self.__update_corners()

    def __update_corners(self):
        self.corners = []
        self.edges = []
        # 多个element顶点拼合的顶点
        all_element_edges = []
        angle_at_point = {}
        self.concave_convex_enum = ConcaveConvexEnum.convex
        for element in self.used_elements:
            all_element_edges.extend(element.edges)
            points = element.points
            for point in points:
                angle = element.get_angle_at_point(point)
                if point not in angle_at_point:
                    angle_at_point[point] = 0.
                angle_at_point[point] += angle

        for k, v in angle_at_point.items():
            if abs(v - 2 * PI) < self.error_threshold:
                continue
            for e in all_element_edges:
                if e.point_is_on_segment(k, self.error_threshold, end_point=False):
                    angle_at_point[k] += PI
                    break

        for k, v in angle_at_point.items():
            if v > PI and abs(v - PI) > self.error_threshold and abs(v - 2 * PI) > self.error_threshold:
                self.concave_convex_enum = ConcaveConvexEnum.concave
            if abs(v - PI) < self.error_threshold or abs(v - 2 * PI) < self.error_threshold:
                continue
            self.corners.append(k)

        point_edge_dict = {}
        for e in all_element_edges:
            if e.p1 not in point_edge_dict:
                point_edge_dict[e.p1] = []
            point_edge_dict[e.p1].append(e)
            if e.p2 not in point_edge_dict:
                point_edge_dict[e.p2] = []
            point_edge_dict[e.p2].append(e)

        element_edges1 = []
        # 去掉完全重合的边
        for e in all_element_edges:
            if e not in element_edges1:
                element_edges1.append(e)
            else:
                element_edges1.remove(e)

        element_edges = []
        for e in element_edges1:

            if e.p1 in self.corners and e.p2 in self.corners:
                overlap = False
                for e1 in all_element_edges:
                    e.segments_has_overlap(e1,self.error_threshold)

            else:
                element_edges.append(e)

        if len(element_edges) == 0:
            return
        points_group = [element_edges.pop(0).get_points_set()]
        for e in element_edges:
            new_group = True
            for group_set in points_group:
                e_points_set = e.get_points_set()
                test_set = group_set.union(e_points_set)
                if utils.on_one_line(list(test_set)):
                    group_set.update(e_points_set)
                    new_group = False
                    break
            if new_group:
                points_group.append(e.get_points_set())

        for group in points_group:
            edge_points = []
            for p in group:
                if p in self.corners:
                    edge_points.append(p)
            if len(edge_points) < 2:
                continue
            assert len(edge_points) == 2
            new_edge = Segment(edge_points[0], edge_points[1])
            if new_edge in self.edges:
                self.edges.remove(new_edge)
            else:
                self.edges.append(new_edge)

        assert len(self.corners) == len(self.edges)

    def __element_is_valid(self, element):
        all_used_elements_edges = []
        for used_element in self.used_elements:
            all_used_elements_edges.extend(used_element.edges)
            if element.element_is_intersect_element(used_element, self.error_threshold):
                return False

        for x in element.edges:
            for y in all_used_elements_edges:
                if x.segments_has_overlap(y, self.error_threshold):
                    return True
        return False

    def __element_is_valid_extra(self, element):
        for used_element in self.used_elements:
            if element.element_is_intersect_element(used_element, self.error_threshold):
                return False
        return True

    def __try_element(self, element_enum, corner, state, angle):
        assert isinstance(element_enum, TangramElementEnum)
        if element_enum == TangramElementEnum.parallelogram:
            new_element = Parallelogram(element_enum, self.unit_length)
        elif element_enum == TangramElementEnum.square:
            new_element = Square(element_enum, self.unit_length)
        else:
            new_element = Triangle(element_enum, self.unit_length)
        new_element.set_points(p=self.corners[corner],
                               position=state * ANGLE_NUM + angle)
        return new_element

    def get_proper_element(self):
        result = []
        ele_list = []
        for x in self.unused_elements_enum:
            for i in range(1, len(config.STANDARD_SIZE[x])):
                e_len = config.STANDARD_SIZE[x][i] * self.unit_length
                for e in self.edges:
                    if abs(e.length - e_len) < self.error_threshold and e not in ele_list:
                        result.append([x, e])
                        ele_list.append(e)
                        break
                else:
                    continue
                break

        return result

    def try_element_extra(self, element_enum, edge):
        result = []
        new_elements = self.__try_element_extra(element_enum, edge)
        for e in new_elements:
            if not self.__element_is_valid_extra(e):
                continue
            new_solve = copy.deepcopy(self)
            new_solve.__update(e)
            result.append(new_solve)
        return result

    def __try_element_extra(self, element_enum, edge):
        new_elements = []
        a = utils.right_sin(edge.p1, edge.p2)
        if element_enum == config.TangramElementEnum.square:
            square_edge_length = config.STANDARD_SIZE[element_enum][-1] * self.unit_length
            if a == 0:
                new_element1 = Square(element_enum, self.unit_length)
                new_element1.init([edge.p1, edge.p2,
                                   Point.get_instance(edge.p2.x, edge.p2.y + square_edge_length),
                                   Point.get_instance(edge.p1.x, edge.p1.y + square_edge_length)])
                new_elements.append(new_element1)
                new_element2 = Square(element_enum, self.unit_length)
                new_element2.init([edge.p1, edge.p2,
                                   Point.get_instance(edge.p2.x, edge.p2.y - square_edge_length),
                                   Point.get_instance(edge.p1.x, edge.p1.y - square_edge_length)])
                new_elements.append(new_element2)
            elif a == 1:
                new_element1 = Square(element_enum, self.unit_length)
                new_element1.init([edge.p1, edge.p2,
                                   Point.get_instance(edge.p2.x + square_edge_length, edge.p2.y),
                                   Point.get_instance(edge.p1.x + square_edge_length, edge.p1.y)])
                new_elements.append(new_element1)
                new_element2 = Square(element_enum, self.unit_length)
                new_element2.init([edge.p1, edge.p2,
                                   Point.get_instance(edge.p2.x - square_edge_length, edge.p2.y),
                                   Point.get_instance(edge.p1.x - square_edge_length, edge.p1.y)])
                new_elements.append(new_element2)
            else:
                lp = edge.p1 if edge.p1.x < edge.p2.x else edge.p2
                rp = edge.p2 if edge.p1.x < edge.p2.x else edge.p1
                offset = square_edge_length * sin(0.25 * PI)
                if lp.y > rp.y:
                    new_element1 = Square(element_enum, self.unit_length)
                    new_element1.init([lp, rp, Point.get_instance(rp.x + offset, rp.y + offset),
                                       Point.get_instance(lp.x + offset, lp.y + offset),
                                       ])
                    new_elements.append(new_element1)
                    new_element2 = Square(element_enum, self.unit_length)
                    new_element2.init([lp, rp, Point.get_instance(rp.x - offset, rp.y - offset),
                                       Point.get_instance(lp.x - offset, lp.y - offset)])
                    new_elements.append(new_element2)
                else:
                    new_element1 = Square(element_enum, self.unit_length)
                    new_element1.init([lp, rp, Point.get_instance(rp.x - offset, rp.y + offset),
                                       Point.get_instance(lp.x - offset, lp.y + offset)
                                       ])
                    new_elements.append(new_element1)
                    new_element2 = Square(element_enum, self.unit_length)
                    new_element2.init([lp, rp, Point.get_instance(rp.x + offset, rp.y - offset),
                                       Point.get_instance(lp.x + offset, lp.y - offset)])
                    new_elements.append(new_element2)
        elif element_enum == config.TangramElementEnum.parallelogram:
            other_edge = config.STANDARD_SIZE[element_enum][-1] * self.unit_length
            if abs(edge.length - config.STANDARD_SIZE[element_enum][-1] * self.unit_length) < self.error_threshold:
                other_edge = config.STANDARD_SIZE[element_enum][1] * self.unit_length

            if a == 0 or a == 1:
                offset = other_edge * sin(0.25 * PI)
                new_element1 = Parallelogram(element_enum, self.unit_length)
                new_element1.init([edge.p1, edge.p2, Point.get_instance(edge.p2.x + offset, edge.p2.y + offset),
                                   Point.get_instance(edge.p1.x + offset, edge.p1.y + offset)])
                new_element2 = Parallelogram(element_enum, self.unit_length)
                new_element2.init([edge.p1, edge.p2, Point.get_instance(edge.p2.x - offset, edge.p2.y + offset),
                                   Point.get_instance(edge.p1.x - offset, edge.p1.y + offset)])
                new_element3 = Parallelogram(element_enum, self.unit_length)
                new_element3.init([edge.p1, edge.p2, Point.get_instance(edge.p2.x - offset, edge.p2.y - offset),
                                   Point.get_instance(edge.p1.x - offset, edge.p1.y - offset)])
                new_element4 = Parallelogram(element_enum, self.unit_length)
                new_element4.init([edge.p1, edge.p2, Point.get_instance(edge.p2.x + offset, edge.p2.y - offset),
                                   Point.get_instance(edge.p1.x + offset, edge.p1.y - offset)])
                new_elements.append(new_element1)
                new_elements.append(new_element2)
                new_elements.append(new_element3)
                new_elements.append(new_element4)
            else:
                new_element1 = Parallelogram(element_enum, self.unit_length)
                new_element1.init([edge.p1, edge.p2, Point.get_instance(edge.p2.x - other_edge, edge.p2.y),
                                   Point.get_instance(edge.p1.x - other_edge, edge.p1.y)])
                new_element2 = Parallelogram(element_enum, self.unit_length)
                new_element2.init([edge.p1, edge.p2, Point.get_instance(edge.p2.x + other_edge, edge.p2.y),
                                   Point.get_instance(edge.p1.x + other_edge, edge.p1.y)])
                new_elements.append(new_element1)
                new_elements.append(new_element2)
        else:
            new_points = []
            # edge与最长边重合
            if abs(edge.length - config.STANDARD_SIZE[element_enum][1] * self.unit_length) < self.error_threshold:
                if a == 0:
                    new_points.append(Point.get_instance((edge.p1.x + edge.p2.x) / 2.,
                                                         edge.p1.y + self.unit_length *
                                                         config.STANDARD_SIZE[element_enum][-1] * sin(
                                                             0.25 * PI)))
                    new_points.append(Point.get_instance((edge.p1.x + edge.p2.x) / 2.,
                                                         edge.p1.y - self.unit_length *
                                                         config.STANDARD_SIZE[element_enum][-1] * sin(
                                                             0.25 * PI)))
                elif a == 1:
                    new_points.append(
                        Point.get_instance(edge.p1.x + self.unit_length * config.STANDARD_SIZE[element_enum][-1] * sin(
                            0.25 * PI), (edge.p1.y + edge.p2.y) / 2.))
                    new_points.append(
                        Point.get_instance(edge.p1.x - self.unit_length * config.STANDARD_SIZE[element_enum][-1] * sin(
                            0.25 * PI), (edge.p1.y + edge.p2.y) / 2.))
                else:
                    new_points.append(Point.get_instance(edge.p2.x, edge.p1.y))
                    new_points.append(Point.get_instance(edge.p1.x, edge.p2.y))
            # 其他情况
            else:
                if a == 0:
                    new_points.append(Point.get_instance(edge.p1.x, edge.p1.y + self.unit_length *
                                                         config.STANDARD_SIZE[element_enum][-1]))
                    new_points.append(Point.get_instance(edge.p1.x, edge.p1.y - self.unit_length *
                                                         config.STANDARD_SIZE[element_enum][-1]))
                    new_points.append(Point.get_instance(edge.p2.x, edge.p2.y + self.unit_length *
                                                         config.STANDARD_SIZE[element_enum][-1]))
                    new_points.append(Point.get_instance(edge.p2.x, edge.p2.y - self.unit_length *
                                                         config.STANDARD_SIZE[element_enum][-1]))
                elif a == 1:
                    new_points.append(
                        Point.get_instance(edge.p1.x - self.unit_length * config.STANDARD_SIZE[element_enum][-1],
                                           edge.p1.y))
                    new_points.append(
                        Point.get_instance(edge.p1.x + self.unit_length * config.STANDARD_SIZE[element_enum][-1],
                                           edge.p1.y))
                    new_points.append(
                        Point.get_instance(edge.p1.x - self.unit_length * config.STANDARD_SIZE[element_enum][-1],
                                           edge.p2.y))
                    new_points.append(
                        Point.get_instance(edge.p1.x + self.unit_length * config.STANDARD_SIZE[element_enum][-1],
                                           edge.p2.y))
                else:
                    lp = edge.p1 if edge.p1.x < edge.p2.x else edge.p2
                    rp = edge.p2 if edge.p1.x < edge.p2.x else edge.p1
                    if lp.y > rp.y:
                        new_points.append(
                            Point.get_instance(rp.x - self.unit_length * config.STANDARD_SIZE[element_enum][1], rp.y))
                        new_points.append(
                            Point.get_instance(lp.x, lp.y - self.unit_length * config.STANDARD_SIZE[element_enum][1]))
                        new_points.append(
                            Point.get_instance(lp.x + self.unit_length * config.STANDARD_SIZE[element_enum][1], lp.y))
                        new_points.append(
                            Point.get_instance(rp.x, rp.y + self.unit_length * config.STANDARD_SIZE[element_enum][1]))
                    else:
                        new_points.append(
                            Point.get_instance(rp.x - self.unit_length * config.STANDARD_SIZE[element_enum][1], rp.y))
                        new_points.append(
                            Point.get_instance(lp.x, lp.y + self.unit_length * config.STANDARD_SIZE[element_enum][1]))
                        new_points.append(
                            Point.get_instance(lp.x + self.unit_length * config.STANDARD_SIZE[element_enum][1], lp.y))
                        new_points.append(
                            Point.get_instance(rp.x, rp.y - self.unit_length * config.STANDARD_SIZE[element_enum][1]))
            for p in new_points:
                new_element = Triangle(element_enum, self.unit_length)
                new_element.init([edge.p1, edge.p2, p])
                new_elements.append(new_element)
        return new_elements

    def __try_element_triangle(self, element_enum, edge):
        pass

    def try_element(self, element_enum, corner, state, angle):
        new_element = self.__try_element(element_enum, corner, state, angle)
        if not self.__element_is_valid(new_element):
            return None
        new_solve = copy.deepcopy(self)
        new_solve.__update(new_element)
        return new_solve


class DFS:
    def __init__(self, pentagon_type):
        super(DFS, self).__init__()
        self.__pentagon_type = pentagon_type
        self.__stack = []
        self.__result = []
        self.__visited_dict = {}

    def check_visited(self, solve):
        used_element_check = solve.used_element_check()
        if used_element_check not in self.__visited_dict:
            self.__visited_dict[used_element_check] = []
        solve_list = self.__visited_dict[used_element_check]

        new_solve_flag = True
        for x in solve_list:
            if solve == x:
                new_solve_flag = False
                break
        if new_solve_flag:
            solve_list.append(solve)
            self.__stack.append(solve)

    def run(self):
        begin = Solve()
        begin.init()
        self.__stack.append(begin)
        while len(self.__stack) > 0:
            solve = self.__stack.pop(0)
            if len(solve.unused_elements_enum) == 0:
                self.__result.append(solve)
            for element_enum in solve.unused_elements_enum:
                for corner in range(len(solve.corners)):
                    for state in range(TRIANGLE_ELEMENT_STATE_DIC[element_enum]):
                        for angle in range(ANGLE_NUM):
                            new_solve = solve.try_element(element_enum=element_enum, corner=corner, state=state,
                                                          angle=angle)
                            if new_solve is None or \
                                    len(new_solve.corners) > 8 or \
                                    (len(new_solve.used_elements) > 3 and
                                     len(new_solve.corners) > len(new_solve.used_elements) + 1):
                                continue

                            self.check_visited(new_solve)


class BFS(Process):
    def __init__(self, pentagon_type, result_queue):
        super(BFS, self).__init__()
        self.__result = result_queue
        self.__pentagon_type = pentagon_type
        self.__result_queue = Queue(-1)
        self.__queue = Queue(-1)
        self.__remove_dup_queue = Queue(-1)
        self.__multiprocess_solver = []
        self.__message_queue = Queue(-1)
        self.__remove_dup_process = None

    def run(self):
        begin = Solve()
        begin.init()
        self.__queue.put(begin)

        self.__remove_dup_process = BFS.RemoveDupProcess(self.__remove_dup_queue, self.__queue, self.__message_queue,
                                                         self.__result_queue)
        self.__remove_dup_process.start()

        cpu_count = multiprocessing.cpu_count() - 1 if multiprocessing.cpu_count() < 9 else multiprocessing.cpu_count() - 3
        for i in range(cpu_count):
            solver_process = BFS.BFSSolverProcess(self.__queue, self.__remove_dup_queue,
                                                  self.__message_queue)
            solver_process.start()
            self.__multiprocess_solver.append(solver_process)

        time.sleep(5)

        while True:
            if not self.__queue.empty() or not self.__remove_dup_queue.empty():
                time.sleep(2)
                continue
            break

        self.__message_queue.put(2)
        for _ in self.__multiprocess_solver:
            self.__message_queue.put(1)

        time.sleep(20)

        count = 1
        test = []
        while not self.__result_queue.empty():
            solve = self.__result_queue.get()
            if self.__pentagon_type == 0 and solve.concave_convex_enum == ConcaveConvexEnum.concave:
                continue
            if self.__pentagon_type == 1 and solve.concave_convex_enum == ConcaveConvexEnum.convex:
                continue
            image_path = os.path.join(config.RESULT_PATH, str(count) + ".jpg")
            image_array = get_final_result(solve.used_elements)
            test.append(solve)
            cv2.imwrite(image_path, image_array, [cv2.IMWRITE_JPEG_QUALITY, 100])
            count += 1
        self.__result.put("BFS finish")
        for x in test:
            x.update_corners()
        print(test)

    class RemoveDupProcess(Process):
        def __init__(self, dup_queue, queue, message_queue, result_queue):
            super(BFS.RemoveDupProcess, self).__init__()
            self.__dup_queue = dup_queue
            self.__queue = queue
            self.__result_queue = result_queue
            self.__solve_dict = {}
            self.__message_queue = message_queue
            self.__stop = False

        def run(self):
            while not self.__stop or not self.__dup_queue.empty():
                try:
                    message = self.__message_queue.get_nowait()
                    if message == 2:
                        self.__stop = True
                    else:
                        self.__message_queue.put(message)
                except Exception as e:
                    pass

                try:
                    solve = self.__dup_queue.get_nowait()
                except Exception as e:
                    time.sleep(1)
                    continue
                used_element_check = solve.used_element_check()
                if used_element_check not in self.__solve_dict:
                    self.__solve_dict[used_element_check] = []
                solve_list = self.__solve_dict[used_element_check]

                new_solve_flag = True
                for x in solve_list:
                    if solve == x:
                        new_solve_flag = False
                        break
                if new_solve_flag:
                    solve_list.append(solve)
                    if len(solve.unused_elements_enum) == 0:
                        self.__result_queue.put(solve)
                    else:
                        self.__queue.put(solve)

    class BFSSolverProcess(Process):
        def __init__(self, queue, dup_queue, message_queue):
            super(BFS.BFSSolverProcess, self).__init__()
            self.__queue = queue
            self.__dup_queue = dup_queue
            self.__message_queue = message_queue
            self.__stop = False

        def run(self):
            while not self.__stop or not self.__queue.empty():
                try:
                    message = self.__message_queue.get_nowait()
                    if message == 1:
                        self.__stop = True
                    else:
                        self.__message_queue.put(message)
                except Exception as e:
                    pass
                try:
                    solve = self.__queue.get_nowait()
                except Exception as e:
                    time.sleep(1)
                    continue
                for element_enum in solve.unused_elements_enum:
                    for corner in range(len(solve.corners)):
                        for state in range(TRIANGLE_ELEMENT_STATE_DIC[element_enum]):
                            for angle in range(ANGLE_NUM):
                                new_solve = solve.try_element(element_enum=element_enum, corner=corner, state=state,
                                                              angle=angle)
                                if new_solve is None or \
                                        len(new_solve.corners) > 8 or \
                                        (len(new_solve.used_elements) > 3 and
                                         len(new_solve.corners) > len(new_solve.used_elements) + 1):
                                    continue

                                self.__dup_queue.put(new_solve)


class Greedy(Process):
    def __init__(self, pentagon_type, result_queue):
        super(Greedy, self).__init__()
        self.__begin_element = [TangramElementEnum.medium_triangle,
                                TangramElementEnum.square,
                                TangramElementEnum.small_triangle,
                                TangramElementEnum.parallelogram]
        self.__queue = []
        self.__result = result_queue
        self.__pentagon_type = pentagon_type
        self.__solved = []

    def run(self):
        for element in self.__begin_element:
            begin = Solve()
            begin.init(element)
            self.__queue.append(begin)

        while len(self.__queue) > 0:
            solve = self.__queue.pop(0)
            proper_element = solve.get_proper_element()
            if len(solve.unused_elements_enum) == 0:
                self.__solved.append(solve)
                continue
            if len(proper_element)>0:
                for ele, e in proper_element:
                    temp_solve_list = solve.try_element_extra(ele, e)
                    for temp_solve in temp_solve_list:
                        if len(temp_solve.edges) > 6:
                            continue
                        self.__queue.append(temp_solve)
            else:
                if len(solve.used_elements)<5:
                    continue
                for element_enum in solve.unused_elements_enum:
                    for corner in range(len(solve.corners)):
                        for state in range(TRIANGLE_ELEMENT_STATE_DIC[element_enum]):
                            for angle in range(ANGLE_NUM):
                                new_solve = solve.try_element(element_enum=element_enum, corner=corner, state=state,
                                                              angle=angle)
                                if new_solve is None or \
                                        len(new_solve.corners) > 8 or \
                                        (len(new_solve.used_elements) > 3 and
                                         len(new_solve.corners) > len(new_solve.used_elements) + 1):
                                    continue
                                self.__queue.append(new_solve)


if __name__ == '__main__':
    dfs = Greedy(pentagon_type=0, result_queue=None)
    dfs.run()
