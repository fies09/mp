from datetime import datetime
from schema.db_base import DbBase
from schema.models import CoreIndcameraParam
from utils.alchemy_encoder import parse_orm


class DBCoreIndcameraParam(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = CoreIndcameraParam

    def get_core_param_id(self, core_device_id):
        query = self.session.query(self.obj.core_indcamera_param_id).filter(self.obj.core_device_id == core_device_id,
                                                                            self.obj.status == 0)
        result = query.all()

        self.commit()

        if result:
            return result[0][0]
        return None

    def get_indcamera_data(self, core_indcamera_param_id):
        query = self.session.query(self.obj).filter(
            self.obj.core_indcamera_param_id == core_indcamera_param_id,
            self.obj.status == 0
        )
        result = query.all()
        self.commit()
        if result:
            res = parse_orm(result)
            return res[0]
        return None

    def get_height(self, core_indcamera_param_id):
        query = self.session.query(self.obj.elevator_height).filter(
            self.obj.core_indcamera_param_id == core_indcamera_param_id,
            self.obj.status == 0)
        result = query.all()

        self.commit()
        if result:
            return result
        return []

    def get_robot_device_id(self, robot_device_id):
        query = self.session.query(self.obj).filter(
            self.obj.robot_device_id == robot_device_id,
            self.obj.status == 0
        )
        result = query.all()
        self.commit()
        if result:
            res = parse_orm(result)
            return res[0]
        return None

    def get_industry_id(self, core_id):
        query = self.session.query(self.obj).filter(
            self.obj.core_indcamera_param_id == core_id,
            self.obj.status == 0
        )
        result = query.all()
        self.commit()
        if result:
            res = parse_orm(result)
            return res[0]
        else:
            return ""

    def update(self, condition, info):
        results = self.judge_exist(condition)

        for result in results:
            if info.get('img_path'):
                result.img_path = info.get('img_path')
            if info.get('img_path'):
                result.img_pah_thumbnail = info.get('img_pah_thumbnail')
            result.update_time = datetime.now()

            self.commit()

        return results

__all__ = ['DBCoreIndcameraParam']
