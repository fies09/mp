from datetime import datetime
from sqlalchemy import desc
from schema.db_base import DbBase
from schema.models import RobotCmd
from utils.alchemy_encoder import parse_orm


class DBRobotCmd(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = RobotCmd

    def update(self, condition, info):
        results = self.judge_exist(condition)

        for result in results:
            if info.get('status'):
                result.status = info.get('status')
                result.update_time = datetime.now()

            self.commit()

        return results

    def get_top1(self):
        query = self.session.query(self.obj)
        result = query.order_by(desc(self.obj.robot_cmd_id)).limit(1).all()
        results = parse_orm(result)

        self.commit()
        return results

    def get_top2(self):
        query = self.session.query(self.obj)
        result = query.order_by(desc(self.obj.robot_cmd_id)).limit(2).all()
        results = parse_orm(result)

        self.commit()
        return results

    def get_cmd_id(self, id):
        query = self.session.query(self.obj).filter(self.obj.robot_cmd_id == id)
        result = query.all()
        results = parse_orm(result)

        self.commit()
        return results


__all__ = ['DBRobotCmd']
