import numpy as np


def vec_from_points(point_start, point_end):
    """
    Создает вектор из двух точек
    :param point_start: массив с координатами первой точки
    :param point_end: массив с координатами второй точки
    :return: вектор в виде массива
    """
    return (point_end[0] - point_start[0],
            point_start[1] - point_end[1])


def vec_length(vec):
    """
    Возващает длину вектора
    :param vec: вектор в виде массива
    :return: длина вектора
    """
    return np.sqrt(vec[0] ** 2 + vec[1] ** 2)


def vec_direction(vec, to_degrees=False):
    """
    Возвращает угол между вектором и горизонтальной осью
    :param vec: вектор в виде массива
    :param to_degrees: флаг конвертации возвращаемого угла в градусы
    :return: float: угол между вектором и горизонтальной осью
    """
    angle = np.arctan2(vec[1], vec[0])
    if to_degrees:
        angle = np.degrees(angle)
    return angle


def distance_between_points(point1, point2):
    """
    Возвращает расстояние между двумя точками
    :param point1: координаты точки 1
    :param point2: координаты точки 2
    :return: float: расстояние между точками
    """
    return np.sqrt((point2[0] - point1[0]) ** 2 + (point2[1] - point1[1]) ** 2)