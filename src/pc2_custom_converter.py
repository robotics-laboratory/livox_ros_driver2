#!/usr/bin/env python3
import struct
from typing import List, Tuple

import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Time
from sensor_msgs.msg import PointCloud2, PointField

from livox_ros_driver2.msg import CustomMsg, CustomPoint


class Pc2CustomConverter(Node):
    # Инициализирует параметры, publishers/subscribers и состояние для выбранного режима.
    def __init__(self) -> None:
        super().__init__("pc2_custom_converter")

        self.declare_parameter("mode", "pc2cust")  # pc2cust | cust2pc | verify
        self.declare_parameter("pc2_layout", "pointcloud2")  # pointcloud2 | custommsg
        self.declare_parameter("pc2_auto_detect", True)
        self.declare_parameter("pc2_topic", "/livox/lidar")
        self.declare_parameter("custom_topic", "/livox/custom")
        # В режиме verify pc2_topic используется как вход, а verify_topic как выход round-trip.
        self.declare_parameter("verify_topic", "/livox/lidar_roundtrip")
        self.declare_parameter("verify_tolerance", 0.0) # точное сравнение байтов
        self.declare_parameter("verify_max_cache", 20)
        # Метаданные для формирования CustomMsg или PointCloud2.
        self.declare_parameter("lidar_id", 0)
        self.declare_parameter("default_frame_id", "livox_frame")

        # Читает параметры один раз, чтобы упростить runtime-логику.
        self.mode = str(self.get_parameter("mode").value).lower()
        self.pc2_layout = str(self.get_parameter("pc2_layout").value).lower()
        self.pc2_auto_detect = bool(self.get_parameter("pc2_auto_detect").value)
        self.pc2_topic = str(self.get_parameter("pc2_topic").value)
        self.custom_topic = str(self.get_parameter("custom_topic").value)
        self.verify_topic = str(self.get_parameter("verify_topic").value)
        self.verify_tolerance = float(self.get_parameter("verify_tolerance").value)
        self.verify_max_cache = int(self.get_parameter("verify_max_cache").value)
        self.lidar_id = int(self.get_parameter("lidar_id").value)
        self.default_frame_id = str(self.get_parameter("default_frame_id").value)

        # Настраивает подписки/публикации по направлению, чтобы можно было запускать два инстанса для round-trip проверки.
        if self.mode == "pc2cust":
            self.custom_pub = self.create_publisher(CustomMsg, self.custom_topic, 10)
            self.pc2_sub = self.create_subscription(
                PointCloud2, self.pc2_topic, self._on_pc2, 10
            )
            self.pc2_pub = None
            self.custom_sub = None
        elif self.mode == "cust2pc":
            self.pc2_pub = self.create_publisher(PointCloud2, self.pc2_topic, 10)
            self.custom_sub = self.create_subscription(
                CustomMsg, self.custom_topic, self._on_custom, 10
            )
            self.custom_pub = None
            self.pc2_sub = None
        elif self.mode == "verify":
            self.pc2_sub = self.create_subscription(
                PointCloud2, self.pc2_topic, self._on_verify_src, 10
            )
            self.custom_sub = None
            self.custom_pub = None
            self.pc2_pub = None
            self.pc2_verify_sub = self.create_subscription(
                PointCloud2, self.verify_topic, self._on_verify_rt, 10
            )
            self._pc2_src_cache = {}
            self._pc2_rt_cache = {}
            self._verify_total = 0
            self._verify_mismatch = 0
            self._verify_match = 0
        else:
            # Быстро завершается при неверном mode, чтобы избежать тихой неверной конвертации.
            raise ValueError("mode must be pc2cust, cust2pc, or verify")

        # Пишет минимальную конфигурацию в лог.
        self.get_logger().info(
            f"pc2_custom_converter ready. mode={self.mode} pc2_layout={self.pc2_layout} "
            f"pc2_topic={self.pc2_topic} custom_topic={self.custom_topic}"
        )
        self._logged_first_convert = False
        self._last_detected_layout = None
        if self.mode == "verify":
            self.get_logger().info(
                f"verify ready. src={self.pc2_topic} rt={self.verify_topic} "
                f"tolerance={self.verify_tolerance}"
            )

    @staticmethod
    # Преобразует ROS Time в целое число наносекунд.
    def _time_to_ns(stamp: Time) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    @staticmethod
    # Преобразует целое число наносекунд в ROS Time.
    def _ns_to_time(ns: int) -> Time:
        t = Time()
        t.sec = int(ns // 1_000_000_000)
        t.nanosec = int(ns % 1_000_000_000)
        return t

    @staticmethod
    # Возвращает компактную сигнатуру полей для строгого сравнения layout.
    def _fields_signature(fields: List[PointField]) -> List[Tuple[str, int, int, int]]:
        return [(f.name, f.offset, f.datatype, f.count) for f in fields]

    @staticmethod
    # Ожидаемые поля PointCloud2 для legacy layout с timestamp/intensity.
    def _expected_fields_custommsg() -> List[Tuple[str, int, int, int]]:
        return [
            ("x", 0, PointField.FLOAT32, 1),
            ("y", 4, PointField.FLOAT32, 1),
            ("z", 8, PointField.FLOAT32, 1),
            ("intensity", 12, PointField.FLOAT32, 1),
            ("tag", 16, PointField.UINT8, 1),
            ("line", 17, PointField.UINT8, 1),
            ("timestamp", 18, PointField.FLOAT64, 1),
        ]

    @staticmethod
    # Ожидаемые поля PointCloud2 для layout с offset_time/reflectivity.
    def _expected_fields_pointcloud2() -> List[Tuple[str, int, int, int]]:
        return [
            ("offset_time", 0, PointField.UINT32, 1),
            ("x", 4, PointField.FLOAT32, 1),
            ("y", 8, PointField.FLOAT32, 1),
            ("z", 12, PointField.FLOAT32, 1),
            ("reflectivity", 16, PointField.UINT8, 1),
            ("tag", 17, PointField.UINT8, 1),
            ("line", 18, PointField.UINT8, 1),
        ]

    @staticmethod
    # Формирует поля PointCloud2 для legacy layout с timestamp/intensity.
    def _build_fields_custommsg() -> List[PointField]:
        return [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="tag", offset=16, datatype=PointField.UINT8, count=1),
            PointField(name="line", offset=17, datatype=PointField.UINT8, count=1),
            PointField(name="timestamp", offset=18, datatype=PointField.FLOAT64, count=1),
        ]

    @staticmethod
    # Формирует поля PointCloud2 для layout с offset_time/reflectivity.
    def _build_fields_pointcloud2() -> List[PointField]:
        return [
            PointField(name="offset_time", offset=0, datatype=PointField.UINT32, count=1),
            PointField(name="x", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="reflectivity", offset=16, datatype=PointField.UINT8, count=1),
            PointField(name="tag", offset=17, datatype=PointField.UINT8, count=1),
            PointField(name="line", offset=18, datatype=PointField.UINT8, count=1),
        ]

    # Проверяет, что поля входного PointCloud2 точно совпадают с ожидаемой схемой.
    def _validate_fields(self, cloud: PointCloud2, expected: List[Tuple[str, int, int, int]]) -> bool:
        # Простая строгая проверка, чтобы не декодировать с неверным layout.
        sig = self._fields_signature(cloud.fields)
        if sig == expected:
            return True
        self.get_logger().error(
            f"PointCloud2 fields mismatch. Expected: {expected} Got: {sig}"
        )
        return False

    # Итерирует декодированные точки из сырых байтов PointCloud2.
    def _iter_points(self, cloud: PointCloud2, fmt: str, point_step: int):
        if cloud.is_bigendian:
            fmt = ">" + fmt[1:]
        height = max(1, int(cloud.height))
        width = int(cloud.width)
        row_step = int(cloud.row_step)
        data = cloud.data
        for row in range(height):
            row_start = row * row_step
            for col in range(width):
                base = row_start + col * point_step
                yield struct.unpack_from(fmt, data, base)

    # Конвертирует входной PointCloud2 в CustomMsg с учетом выбранного входного layout.
    def _on_pc2(self, cloud: PointCloud2) -> None:
        # Использует заданный PC2 layout (без автодетекта, чтобы логика была проще).
        fmt_param = self.pc2_layout
        if self.pc2_auto_detect:
            sig = self._fields_signature(cloud.fields)
            if sig == self._expected_fields_pointcloud2():
                fmt_param = "pointcloud2"
            elif sig == self._expected_fields_custommsg():
                fmt_param = "custommsg"
            else:
                self.get_logger().error(
                    f"Auto-detect failed. Fields: {sig}"
                )
                return
            if fmt_param != self._last_detected_layout:
                self.get_logger().info(f"Auto-detect pc2_layout={fmt_param}")
                self._last_detected_layout = fmt_param

        if fmt_param == "pointcloud2":
            # Layout pointcloud2: offset_time,x,y,z,reflectivity,tag,line
            if not self._validate_fields(cloud, self._expected_fields_pointcloud2()):
                return
            point_step = 19
            fmt = "<IfffBBB"
            points = self._iter_points(cloud, fmt, point_step)

            msg = CustomMsg()
            msg.header = cloud.header
            msg.timebase = self._time_to_ns(cloud.header.stamp)
            msg.lidar_id = self.lidar_id
            msg.rsvd = [0, 0, 0]
            msg.points = []
            for offset_time, x, y, z, reflectivity, tag, line in points:
                cp = CustomPoint()
                cp.offset_time = int(offset_time)
                cp.x = float(x)
                cp.y = float(y)
                cp.z = float(z)
                cp.reflectivity = int(reflectivity) & 0xFF
                cp.tag = int(tag) & 0xFF
                cp.line = int(line) & 0xFF
                msg.points.append(cp)
            msg.point_num = len(msg.points)
            self.custom_pub.publish(msg)
            if not self._logged_first_convert:
                self.get_logger().info("Converted first PointCloud2 -> CustomMsg")
                self._logged_first_convert = True
            return

        if fmt_param == "custommsg":
            # Layout custommsg: x,y,z,intensity,tag,line,timestamp
            if not self._validate_fields(cloud, self._expected_fields_custommsg()):
                return
            point_step = 26
            fmt = "<ffffBBd"
            timebase = self._time_to_ns(cloud.header.stamp)

            msg = CustomMsg()
            msg.header = cloud.header
            msg.timebase = timebase
            msg.lidar_id = self.lidar_id
            msg.rsvd = [0, 0, 0]
            msg.points = []
            for x, y, z, intensity, tag, line, timestamp in self._iter_points(
                cloud, fmt, point_step
            ):
                cp = CustomPoint()
                ts = int(timestamp)
                offset = ts - timebase
                if offset < 0:
                    offset = 0
                cp.offset_time = int(offset) & 0xFFFFFFFF
                cp.x = float(x)
                cp.y = float(y)
                cp.z = float(z)
                cp.reflectivity = int(max(0, min(255, int(intensity))))
                cp.tag = int(tag) & 0xFF
                cp.line = int(line) & 0xFF
                msg.points.append(cp)
            msg.point_num = len(msg.points)
            self.custom_pub.publish(msg)
            if not self._logged_first_convert:
                self.get_logger().info("Converted first PointCloud2 -> CustomMsg")
                self._logged_first_convert = True
            return

        self.get_logger().error(f"Unknown pc2_layout: {fmt_param}")

    # Конвертирует входной CustomMsg в PointCloud2 с учетом выбранного выходного layout.
    def _on_custom(self, msg: CustomMsg) -> None:
        fmt_param = self.pc2_layout
        if fmt_param not in ("pointcloud2", "custommsg"):
            self.get_logger().error("pc2_layout must be pointcloud2 or custommsg")
            return

        cloud = PointCloud2()
        cloud.header.stamp = self._ns_to_time(int(msg.timebase))
        cloud.header.frame_id = (
            msg.header.frame_id if msg.header.frame_id else self.default_frame_id
        )
        point_count = len(msg.points)
        if int(msg.point_num) != point_count:
            self.get_logger().warn(
                f"CustomMsg point_num ({int(msg.point_num)}) != points length ({point_count}), "
                "using points length"
            )

        cloud.height = 1
        cloud.width = point_count
        cloud.is_bigendian = False
        cloud.is_dense = True

        if fmt_param == "pointcloud2":
            # Layout pointcloud2: offset_time,x,y,z,reflectivity,tag,line
            cloud.fields = self._build_fields_pointcloud2()
            cloud.point_step = 19
            fmt = "<IfffBBB"
            buf = bytearray(cloud.point_step * cloud.width)
            for i, p in enumerate(msg.points):
                struct.pack_into(
                    fmt,
                    buf,
                    i * cloud.point_step,
                    int(p.offset_time) & 0xFFFFFFFF,
                    float(p.x),
                    float(p.y),
                    float(p.z),
                    int(p.reflectivity) & 0xFF,
                    int(p.tag) & 0xFF,
                    int(p.line) & 0xFF,
                )
        else:
            # Layout custommsg: x,y,z,intensity,tag,line,timestamp
            cloud.fields = self._build_fields_custommsg()
            cloud.point_step = 26
            fmt = "<ffffBBd"
            buf = bytearray(cloud.point_step * cloud.width)
            base = int(msg.timebase)
            for i, p in enumerate(msg.points):
                timestamp = float(base + int(p.offset_time))
                struct.pack_into(
                    fmt,
                    buf,
                    i * cloud.point_step,
                    float(p.x),
                    float(p.y),
                    float(p.z),
                    float(p.reflectivity),
                    int(p.tag) & 0xFF,
                    int(p.line) & 0xFF,
                    timestamp,
                )

        cloud.row_step = cloud.point_step * cloud.width
        cloud.data = bytes(buf)
        self.pc2_pub.publish(cloud)

    # Кладет сообщение в кэш по ключу timestamp и ограничивает размер кэша.
    def _cache_add(self, cache: dict, key: int, msg: PointCloud2) -> None:
        cache[key] = msg
        if len(cache) > self.verify_max_cache:
            for old_key in list(cache.keys())[: len(cache) - self.verify_max_cache]:
                del cache[old_key]

    # Обрабатывает source PointCloud2 сообщения в режиме verify.
    def _on_verify_src(self, cloud: PointCloud2) -> None:
        key = self._time_to_ns(cloud.header.stamp)
        self._cache_add(self._pc2_src_cache, key, cloud)
        self._try_verify_pair(key)

    # Обрабатывает round-trip PointCloud2 сообщения в режиме verify.
    def _on_verify_rt(self, cloud: PointCloud2) -> None:
        key = self._time_to_ns(cloud.header.stamp)
        self._cache_add(self._pc2_rt_cache, key, cloud)
        self._try_verify_pair(key)

    # Сопоставляет source/round-trip сообщения из кэша и обновляет статистику verify.
    def _try_verify_pair(self, key: int) -> None:
        if key not in self._pc2_src_cache or key not in self._pc2_rt_cache:
            return
        src = self._pc2_src_cache.pop(key)
        rt = self._pc2_rt_cache.pop(key)
        self._verify_total += 1
        ok, reason = self._compare_clouds(src, rt)
        if ok:
            self._verify_match += 1
        else:
            self._verify_mismatch += 1
            self.get_logger().error(
                f"verify mismatch at stamp={key}: {reason}"
            )
        if self._verify_total % 500 == 0:
            self.get_logger().info(
                f"verify stats: total={self._verify_total} "
                f"match={self._verify_match} mismatch={self._verify_mismatch}"
            )

    # Сравнивает два PointCloud2 по метаданным layout и полезной нагрузке.
    def _compare_clouds(self, a: PointCloud2, b: PointCloud2) -> Tuple[bool, str]:
        if self._fields_signature(a.fields) != self._fields_signature(b.fields):
            return False, "fields differ"
        if (
            int(a.point_step) != int(b.point_step)
            or int(a.row_step) != int(b.row_step)
            or int(a.width) != int(b.width)
            or int(a.height) != int(b.height)
            or bool(a.is_bigendian) != bool(b.is_bigendian)
            or bool(a.is_dense) != bool(b.is_dense)
        ):
            return False, "layout meta differ"

        if self.verify_tolerance <= 0.0:
            if bytes(a.data) != bytes(b.data):
                return False, "raw data differ"
            return True, "ok"

        layout = self.pc2_layout
        if layout == "pointcloud2":
            fmt = "<IfffBBB"
            point_step = 19
        elif layout == "custommsg":
            fmt = "<ffffBBd"
            point_step = 26
        else:
            return False, "unknown pc2_layout"

        for pa, pb in zip(
            self._iter_points(a, fmt, point_step),
            self._iter_points(b, fmt, point_step),
        ):
            for va, vb in zip(pa, pb):
                if isinstance(va, float) or isinstance(vb, float):
                    if abs(float(va) - float(vb)) > self.verify_tolerance:
                        return False, "float diff"
                else:
                    if int(va) != int(vb):
                        return False, "int diff"
        return True, "ok"


# Точка входа: создает node и крутит spin до завершения.
def main() -> None:
    rclpy.init()
    node = Pc2CustomConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
