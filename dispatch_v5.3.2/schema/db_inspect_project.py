from schema.db_base import DbBase
from schema.models import InspectProject
from utils.alchemy_encoder import parse_orm


class DBInspectProject(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = InspectProject

    def get_task_id(self, inspect_project_id):
        result = self.session.query(self.obj). \
            filter(self.obj.inspect_project_id == inspect_project_id).all()
        self.commit()

        results = parse_orm(result)
        if results:
            return results[0]
        return {}


__all__ = ['DBInspectProject']
