import logging
import math
import threading
import time

from pioneer_sdk import Pioneer

MAX_FLIGHT_SPEED = 2.0
SLOWDOWN_DISTANCE = 1.2
MIN_APPROACH_SPEED = 0.6


class PioneerControl:
    def __init__(self, ip_addr, pioneer_port):
        logging.basicConfig(
            level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
        )
        self.logger = logging.getLogger(__name__)

        self.pioneer: Pioneer | None = None
        self.target_speed = 0  # для управления скоростью в функции __compute_speed
        self.low_distance = False

        self.__current_pos: tuple[float, float, float] = (
            0.0,
            0.0,
            0.0,
        )  # последняя полученная позиция дрона
        self.__command_pos: tuple[int] | None = None  # позиция для команды
        self.__yaw: float = 0.0  # рысканье
        self.__yaw_rate: float = 0.0  # скорость поворота рысканья
        self.__speed: tuple[float, float, float] = (
            0.0,
            0.0,
            0.0,
        )  # задание скоростей по x, y, z
        self.__coords_system: str = (
            "GLOBAL_COORDS"  # GLOBAL_COORDS / SPEED / SPEED_FIXED / SPEED_POINT
        )
        self.__armed: bool = False
        self.__takeoffed: bool = False
        self.__target_yaw: int | None = None

        self.__connect(ip_addr=str(ip_addr), port=int(pioneer_port))

        # --- Изменения для корректного завершения ---
        self.__stop_event = threading.Event()
        self.__command_thread = threading.Thread(
            target=self.__command_loop, daemon=True
        )
        self.__command_thread.start()

    def __command_loop(self):
        self.logger.info("Command loop started")
        prev_cmd = None
        # --- Цикл теперь проверяет событие остановки ---
        while not self.__stop_event.is_set():
            if not self.pioneer.connected():
                self.logger.error("PIONEER NOT CONNECTED")
                time.sleep(1)  # Пауза перед новой попыткой
                continue

            if self.__coords_system == "GLOBAL_COORDS":
                if self.__command_pos is not None and self.__armed and self.__takeoffed:
                    if prev_cmd != self.__command_pos:
                        self.pioneer.go_to_local_point(
                            x=self.__command_pos[0],
                            y=self.__command_pos[1],
                            z=self.__command_pos[2],
                            yaw=self.__yaw,
                        )
                        self.logger.info(f"Command sent {self.__command_pos}")
                        prev_cmd = self.__command_pos
            elif self.__coords_system == "SPEED":
                if self.__armed and self.__takeoffed:
                    if prev_cmd != (*self.__speed, self.__yaw_rate):
                        self.pioneer.set_manual_speed(
                            vx=self.__speed[0],
                            vy=self.__speed[1],
                            vz=self.__speed[2],
                            yaw_rate=self.__yaw_rate,
                        )
                        self.logger.info(
                            f"Command sent {self.__speed}, yaw_rate: {self.__yaw_rate}"
                        )
                        prev_cmd = (*self.__speed, self.__yaw_rate)
            elif self.__coords_system == "SPEED_FIXED":
                if self.__armed and self.__takeoffed:
                    if self.__target_yaw is not None and self.__is_yaw_reached():
                        self.__target_yaw = None
                        self.__yaw_rate = 0
                    if prev_cmd != (*self.__speed, self.__yaw_rate):
                        self.pioneer.set_manual_speed_body_fixed(
                            vx=self.__speed[0],
                            vy=self.__speed[1],
                            vz=self.__speed[2],
                            yaw_rate=self.__yaw_rate,
                        )
                        self.logger.info(
                            f"Command sent {self.__speed}, yaw_rate: {self.__yaw_rate}"
                        )
                        prev_cmd = (*self.__speed, self.__yaw_rate)
            elif self.__coords_system == "SPEED_POINT":
                if self.__command_pos is not None:
                    self.__speed = self.__compute_speed(
                        *self.__command_pos, wanted_speed=self.target_speed
                    )
                if self.__armed and self.__takeoffed:
                    self.pioneer.set_manual_speed_body_fixed(
                        vx=self.__speed[0],
                        vy=self.__speed[1],
                        vz=self.__speed[2],
                        yaw_rate=self.__yaw_rate,
                    )

            self.__update_position()
            # self.__update_yaw()
            time.sleep(0.05)
        self.logger.info("Command loop stopped.")

    def stop(self):
        """Останавливает поток управления и отключается от дрона."""
        self.logger.info("Stopping...")
        self.__stop_event.set()
        self.__command_thread.join()  # Ожидаем завершения потока
        if self.pioneer and self.pioneer.connected():
            self.pioneer.close()  # Закрываем соединение
            self.logger.info("Pioneer connection closed.")

    def __connect(self, ip_addr, port: str):
        """Подключение к пионеру"""
        try:
            self.pioneer = Pioneer(ip=ip_addr, mavlink_port=port, logger=False)
            if self.pioneer.connected():
                self.logger.info("Pioneer connected!")
            else:
                self.logger.warning("Can't connect to Pioneer!")
        except Exception as e:
            self.logger.error(f"Failed to connect to Pioneer: {e}")

    def __is_yaw_reached(self, dead_zone: int = 10) -> bool:
        if self.__coords_system == "SPEED_FIXED" and self.__target_yaw is not None:
            return abs(math.degrees(self.__yaw) - self.__target_yaw) < dead_zone
        return False

    def __is_point_reached(self, dead_zone: float = 0.7) -> bool:
        """Проверка на расстояние до точки"""
        if self.__command_pos is None:
            return False
        if self.__coords_system in ("GLOBAL_COORDS", "SPEED_POINT"):
            if self.__current_pos:
                distance = math.sqrt(
                    (self.__current_pos[0] - self.__command_pos[0]) ** 2
                    + (self.__current_pos[1] - self.__command_pos[1]) ** 2
                    + (self.__current_pos[2] - self.__command_pos[2]) ** 2
                )
                return distance < dead_zone
            return False
        else:
            self.logger.warning(
                "Can't check if point is reached in current coordinate system"
            )
            return True

    # def __update_yaw(self):
    #     yaw = self.pioneer.get_yaw(get_last_received=True)
    #     if yaw is not None:
    #         self.__yaw = yaw

    def __update_position(self):
        pos = self.pioneer.get_local_position_lps(get_last_received=True)
        if pos is not None:
            self.__current_pos = pos

    def __compute_speed(
        self, target_x: float, target_y: float, target_z: float, wanted_speed: float
    ) -> tuple[float, float, float]:
        """Вычисляет скорость, с которой дрону нужно лететь по x, y, z чтобы добраться до точки"""
        if self.__current_pos is None or self.__command_pos is None:
            return 0.0, 0.0, 0.0

        diff_x = target_x - self.__current_pos[0]
        diff_y = target_y - self.__current_pos[1]
        diff_z = target_z - self.__current_pos[2]

        distance = math.sqrt(diff_x**2 + diff_y**2 + diff_z**2)

        if distance < 0.05:  # Если мы уже в точке, скорость равна нулю
            return 0.0, 0.0, 0.0

        wanted_speed = max(0.0, min(float(wanted_speed), MAX_FLIGHT_SPEED))

        # Плавное замедление при приближении к цели
        current_speed = wanted_speed
        if distance < SLOWDOWN_DISTANCE:
            current_speed = max(
                MIN_APPROACH_SPEED,
                wanted_speed * (distance / SLOWDOWN_DISTANCE),
            )

        speed_scale = current_speed / distance
        vx = diff_x * speed_scale
        vy = diff_y * speed_scale
        vz = diff_z * speed_scale

        return vx, vy, vz

    def point_reached(self):
        if self.pioneer.connected():
            return self.__is_point_reached()
        else:
            self.logger.error("PIONEER NOT CONNECTED FOR POINT REACHED")
            return False

    def set_local_pose(self, x, y, z, yaw=None):
        """Ставит координаты для command_loop, yaw задается в градусах"""
        self.__coords_system = "GLOBAL_COORDS"
        self.__yaw_rate = 0.0
        self.__speed = (0.0, 0.0, 0.0)
        self.__command_pos = (x, y, z)
        if yaw is not None:
            self.__yaw = math.radians(yaw)
        self.logger.info(f"Coords set: {self.__command_pos}, yaw: {self.__yaw}")

    def set_speed(self, vx, vy, vz, yaw_rate=None):
        """Устанавливает скорость в глобальной системе координат"""
        self.__coords_system = "SPEED"
        self.__command_pos = None
        self.__speed = (vx, vy, vz)
        if yaw_rate is not None:
            self.__yaw_rate = math.radians(yaw_rate)
        self.logger.info(f"Speed set: {self.__speed}, yaw rate: {self.__yaw_rate}")

    def set_point_with_speed(self, target_x, target_y, target_z, speed):
        """Лететь к точке с заданной скоростью в связанной с телом системе координат"""
        self.__coords_system = "SPEED_POINT"
        self.__command_pos = (target_x, target_y, target_z)
        self.target_speed = speed

    def set_speed_fixed(self, vx: float, vy: float, vz: float, yaw_rate=None):
        """Устанавливает скорость в связанной с телом системе координат"""
        self.__coords_system = "SPEED_FIXED"
        self.__command_pos = None
        self.__speed = (vx, vy, vz)
        if yaw_rate is not None:
            self.__yaw_rate = math.radians(yaw_rate)
        self.logger.info(
            f"<<FIXED>> Speed set: {self.__speed}, yaw rate: {self.__yaw_rate}"
        )

    def get_local_pose(self):
        return self.__current_pos

    def get_yaw(self, degrees=False):
        return math.degrees(self.__yaw) if degrees else self.__yaw

    def arm(self):
        if self.pioneer.connected():
            self.pioneer.arm()
            self.__armed = True
            self.logger.info("ARMED")
        else:
            self.logger.error("PIONEER NOT CONNECTED FOR ARM")

    def takeoff(self):
        if self.pioneer.connected():
            self.pioneer.takeoff()
            self.__takeoffed = True
            self.logger.info("TAKEOFF")
        else:
            self.logger.error("PIONEER NOT CONNECTED FOR TAKEOFF")

    def connection(self):
        return self.pioneer and self.pioneer.connected()

    def land(self):
        if self.pioneer.connected():
            if self.__takeoffed:
                self.__command_pos = None
                self.__speed = (0.0, 0.0, 0.0)
                self.__yaw_rate = 0.0
                self.pioneer.land()
                self.__takeoffed = False
                self.__armed = False
                self.logger.info("LANDING")
            else:
                self.logger.warning("PIONEER NOT TAKEOFF FOR LANDING")
        else:
            self.logger.error("PIONEER NOT CONNECTED FOR LANDING")

    def set_yaw(self, yaw, yaw_rate=30):
        if self.__coords_system == "SPEED_FIXED":
            self.__target_yaw = yaw
            self.__yaw_rate = math.radians(
                yaw_rate if self.get_yaw(degrees=True) < yaw else -yaw_rate
            )
            self.logger.info(f"Set yaw: {yaw} with rate: {yaw_rate}")
