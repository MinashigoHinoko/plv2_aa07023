def rotate(point: tuple[float, float], angle: float):
    from math import cos, sin

    return (
        point[0] * cos(angle) - point[1] * sin(angle),
        point[1] * cos(angle) + point[0] * sin(angle),
    )


def translate(point: tuple[float, float], translation: tuple[float, float]):
    return point[0] + translation[0], point[1] + translation[1]


def create_transformations_between_odom_start_and_odom(
    start_pos_in_odom: tuple[float, float], start_ori_in_odom: float
):
    from math import fmod, pi

    def odom_start_to_odom(point_or_ori: tuple[float, float] | float):
        if isinstance(point_or_ori, tuple):
            point = point_or_ori
            return translate(
                rotate(point, start_ori_in_odom),
                start_pos_in_odom,
            )
        ori = point_or_ori
        return fmod(ori + start_ori_in_odom, pi)

    def odom_to_odom_start(point_or_ori: tuple[float, float] | float):
        # Reverse of `maze_to_odom`
        def minus(point):
            return -point[0], -point[1]

        if isinstance(point_or_ori, tuple):
            point = point_or_ori
            return rotate(
                translate(point, minus(start_pos_in_odom)),
                -start_ori_in_odom,
            )
        ori = point_or_ori

        return fmod(ori - start_ori_in_odom, pi)

    return odom_start_to_odom, odom_to_odom_start
