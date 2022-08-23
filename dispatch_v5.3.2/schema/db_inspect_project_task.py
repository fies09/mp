from schema.db_base import DbBase
from schema.models import InspectProjectTask
from utils.alchemy_encoder import parse_orm


class DBInspectProjectTask(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = InspectProjectTask

    def get_by_num(self, robot_id, inspect_project_task_id):
        result = self.session.query(self.obj). \
            filter(self.obj.inspect_project_task_id == inspect_project_task_id,
                   self.obj.robot_id == robot_id,
                   self.obj.status == 0,
                   self.obj.del_flag == 0).all()

        self.commit()

        results = parse_orm(result)
        if results:
            return results[0]
        return {}

    def get_by_project_task(self, inspect_project_task_id):
        result = self.session.query(self.obj). \
            filter(self.obj.inspect_project_task_id == inspect_project_task_id).all()
        self.commit()

        results = parse_orm(result)
        if results:
            return results[0]
        return {}


__all__ = ['DBInspectProjectTask']
