from datetime import datetime
from schema.db_base import DbBase
from schema.models import AlarmManage
from utils.alchemy_encoder import parse_orm
from datetime import datetime
import traceback
from configs.log import logger

class DBAlarmManage_Hardware(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = AlarmManage

    def update(self, **kwargs):
        kwargs["update_time"] = datetime.now()
        self.session.query(self.obj).filter(
            self.obj.status != 0).update(kwargs)
        try:
            self.session.commit()
            return True, "item update success"
        except Exception as e:
            logger.error(
                f"写入失败:{traceback.format_exc()}行数:{e.__traceback__.tb_lineno}")
            self.session.rollback()
            return False, "item update failed"
        finally:
            self.session.close()

    def insert_data(self, **kwargs):

        entry = self.obj
        for k, v in kwargs.items():
            if hasattr(entry, k):
                setattr(entry, k, v)

        entry.create_time = datetime.now()
        entry.update_time = datetime.now()
        try:
            self.session.add(entry)
            self.commit()
            return True
        except Exception as e:
            self.session.rollback()
            logger.error(f"写入失败:{traceback.format_exc()}行数:{e.__traceback__.tb_lineno}")
            return False
        finally:
            self.session.close()


    def get_alarm_data(self, robot_position_id, robot_item_id):
        query = self.session.query(self.obj).filter(self.obj.robot_position_id == robot_position_id,
                                                    self.obj.robot_item_id == robot_item_id,
                                                    self.obj.status == 0,
                                                    self.obj.undo_status == 0)
        result = query.all()

        self.commit()

        if result:
            results = parse_orm(result)
            return results[0]
        return None


__all__ = ['DBAlarmManage']
