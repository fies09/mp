import hashlib
import time
import uuid
from datetime import datetime

from sqlalchemy import desc

from configs.log import logger
from schema.db_base import DbBase
from schema.models import RobotState
from utils.alchemy_encoder import parse_orm


class DBRobotState(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = RobotState

    def insert(self, info):
        data = RobotState(
            robot_id=info.get("robot_id", 1),
            task_type_id=info.get("task_type_id", 0),
            robot_temperature=info.get("robot_temperature",0),
            battery_level=info.get("battery_level",0),
            battery_temperature=info.get("battery_temperature",0),
            cpu_percent=info.get("cpu_percent",0),
            cpu_temperature=info.get("cpu_temperature",0),
            disk_percent=info.get("disk_percent",0),
            memory_percent=info.get("memory_percent",0),
            is_charged=info.get("is_charged",0),
            obstacle_state=info.get("obstacle_state",0),
            radar_state=info.get("radar_state",0),
            motor_state=info.get("motor_state",0),
            imu_state=info.get("imu_state",0),
            gas_sensor_state=info.get("gas_sensor_state"),
            lidar_state=info.get("lidar_state"),
            camera_state=info.get("camera_state"),
            thermal_state=info.get("thermal_state"),
            battery_state=info.get("battery_state"),
            wind_state=info.get("wind_state"),
            mic_state=info.get("mic_state"),
            discharge_state=info.get("discharge_state"),
            motion_info=info.get("motion_info", 1),
            info=info.get("info", 1),
            create_time=datetime.now(),
            update_time=info.get("update_time", datetime.now()),
            remark=info.get("remark"),

        )
        self.session.add(data)
        self.commit()

    def update(self, id, data):
        # data = {x:"",y:"",yaw:"航向",vx:"线速度",vyaw:"角速度"}

        self.session.query.filter_by(self.obj.robot_id == id).update(data)

        self.commit()



__all__ = ['DBRobotState']
