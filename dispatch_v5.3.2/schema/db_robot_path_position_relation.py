from schema.db_base import DbBase
from schema.models import RobotPathPositionRelation
from utils.alchemy_encoder import parse_orm


class DBRobotPathPositionRelation(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = RobotPathPositionRelation

    def get_all_data(self):
        query = self.session.query(self.obj)
        result = query.all()
        self.commit()
        if result:
            results = parse_orm(result)
            return results[0]
        return None


    def get_position_id(self, robot_path_id):
        query = self.session.query(self.obj).filter(self.obj.robot_path_id == robot_path_id).order_by(self.obj.order_num)
        result = query.all()
        self.commit()
        results = parse_orm(result)
        return results

    def get_position_id_desc(self, robot_path_id):
        query = self.session.query(self.obj).filter(self.obj.robot_path_id == robot_path_id).order_by(self.obj.order_num.desc())
        result = query.all()
        self.commit()
        results = parse_orm(result)
        return results

    def get_alive_data(self, robot_path_id, robot_position_id):
        query = self.session.query(self.obj).filter(self.obj.robot_path_id == robot_path_id,
                                                    self.obj.robot_position_id == robot_position_id)
        self.commit()

        result = query.all()
        results = parse_orm(result)
        if results:
            return True
        else:
            return False

    def get_alive_data_one(self, robot_path_id, robot_position_id):
        query = self.session.query(self.obj).filter(self.obj.robot_path_id == robot_path_id,
                                                    self.obj.robot_position_id == robot_position_id)
        result = query.all()
        self.commit()

        results = parse_orm(result)
        return results[0]




__all__ = ['DBRobotPathPositionRelation']
