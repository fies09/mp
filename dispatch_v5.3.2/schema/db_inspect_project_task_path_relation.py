from schema.db_base import DbBase
from schema.models import InspectProjectTaskPathRelation
from utils.alchemy_encoder import parse_orm


class DBInspectProjectTaskPathRelation(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = InspectProjectTaskPathRelation

    def get_play_desc(self, robot_position_id, inspect_project_task_id):
        query = self.session.query(self.obj).filter(self.obj.robot_position_id == robot_position_id)\
            .filter(self.obj.inspect_project_task_id == inspect_project_task_id)
        result = query.all()
        self.commit()
        results = parse_orm(result)
        if results:
            return results[0]
        return {}


__all__ = ['DBInspectProjectTaskPathRelation']
