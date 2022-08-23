from datetime import datetime

from schema.db_base import DbBase
from sqlalchemy import desc, or_
from schema.models import InspectProjectDetail
from utils.alchemy_encoder import parse_orm


class DBInspectProjectDetail(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = InspectProjectDetail

    def update(self, condition, info):
        results = self.judge_exist(condition)

        for result in results:

            if info.get('status'):
                result.status = info.get('status')
            if info.get('task_status'):
                result.task_status = info.get('task_status')
            if info.get('power_consumption'):
                if info.get('power_consumption') < 0:
                    num = 0
                else:
                    num = info.get('power_consumption')
                result.power_consumption = num
            if info.get('exception_info'):
                result.exception_info = info.get('exception_info', "已完成")

            if info.get("status") == 1:
                pass
            if info.get('update_time'):
                result.update_time = info.get('update_time')
            else:
                result.update_time = datetime.now()
            self.commit()

        return results

    def get_by_id(self, inspect_project_detail_id):
        result = self.session.query(self.obj). \
            filter(self.obj.inspect_project_detail_id == inspect_project_detail_id).all()

        self.commit()
        if result:
            results = parse_orm(result)
            return results[0]
        return {}

    def get_top1(self):
        try:
            query = self.session.query(self.obj)

            result = query.order_by(desc(self.obj.create_time)).limit(1).all()
            self.commit()
            results = parse_orm(result)
            if results:
                return results[0]
            return {}
        except:
            return {}

    def get_task_state(self):
        # '0 等待开始，1执行中，2 已完成，3 未执行，6 执行失败, 4正在录像，5 已到达随工点',
        result = self.session.query(self.obj). \
            filter(or_(self.obj.status == 0, self.obj.status == 1, self.obj.status == 4, self.obj.status == 5)).all()

        self.commit()
        if result:
            results = parse_orm(result)
            return results
        return {}


__all__ = ['DBInspectProjectDetail']
