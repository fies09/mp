import time
from datetime import datetime

from sqlalchemy import desc, or_
from schema.db_base import DbBase
from schema.models import RobotPosition
from utils.alchemy_encoder import parse_orm


class DBRobotPosition(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = RobotPosition

    def get_list(self, ids):
        items = self.session.query(self.obj).filter(self.obj.del_flag == 0)

        if len(ids) > 0:
            items = items.filter(self.obj.robot_position_id.in_(ids))

        result = items.order_by(self.obj.robot_position_id).all()

        total = items.count()

        if result:
            results = parse_orm(result)
            return results, total
        else:
            return None, 0

    def update(self, condition, info):
        results = self.judge_exist(condition)

        for result in results:
            if info.get('current_state'):
                result.current_state = info.get('current_state')
            if info.get('update_time'):
                result.update_time = datetime.now()

        self.commit()

        return results

    def get_position_all(self):
        query = self.session.query(self.obj).filter().filter(
            self.obj.del_flag == 0)

        result = query.all()
        self.commit()
        if result:
            results = parse_orm(result)
            return results[0]
        return None

    def get_position_id(self, position_id):
        sql = """
            SELECT *
            FROM robot_path_position_relation A 
            left join robot_position B 
            on A.robot_position_id=B.robot_position_id 
            where A.robot_position_id = {} and B.del_flag = 0;
        """.format(position_id)
        results = self.get_execute(sql)
        return results

    # def get_position_name(self, robot_path_id):
    #     query = self.session.query(self.obj).filter(self.obj.robot_path_id == robot_path_id).filter(
    #         self.obj.type == 4).filter(self.obj.del_flag == 0)
    #
    #     result = query.all()
    #     self.commit()
    #
    #     if result:
    #         results = parse_orm(result)
    #         return results[0]
    #     return None

    def get_position_name(self, robot_path_id):
        sql = """
            SELECT *
            FROM robot_path_position_relation A 
            left join robot_position B 
            on A.robot_position_id=B.robot_position_id 
            where A.robot_path_id = {} and B.type = 4 and B.del_flag = 0;
        """.format(robot_path_id)
        results = self.get_execute(sql)
        if results:
            return results[0]
        else:
            return {}

    def get_battery_position(self):
        query = self.session.query(self.obj).filter(self.obj.type == 4).filter(self.obj.del_flag == 0)
        result = query.all()
        self.commit()

        if result:
            results = parse_orm(result)
            return results
        else:
            return []

    # def get_position(self, robot_id, robot_path_id):
    #     query = self.session.query(self.obj).filter(self.obj.robot_id == robot_id). \
    #         filter(self.obj.status == 0).filter(self.obj.del_flag == 0) \
    #         .filter(self.obj.robot_path_id == robot_path_id) \
    #         .order_by(self.obj.position_num)
    #
    #     result = query.all()
    #     self.commit()
    #     results = parse_orm(result)
    #     return results

    def get_position(self, robot_id, robot_path_id):
        sql = """
            SELECT *
            FROM robot_path_position_relation A 
            left join robot_position B 
            on A.robot_position_id=B.robot_position_id 
            where A.robot_path_id = {} and B.robot_id = {} and B.status = 0 and B.del_flag = 0
            order by A.order_num;
        """.format(robot_path_id, robot_id)
        results = self.get_execute(sql)
        if results:
            return results[0]
        else:
            return {}

    # def get_position_by_id(self, position_id):
    #     query = self.session.query(self.obj).filter(self.obj.robot_position_id == position_id).\
    #         filter(self.obj.del_flag == 0).filter(self.obj.status == 0)
    #     result = query.all()
    #     self.commit()
    #     results = parse_orm(result)
    #     if results:
    #         return results[0]
    #     return {}

    def get_position_by_id(self, robot_position_id, robot_path_id):
        sql = """
            SELECT *
            FROM robot_path_position_relation A 
            left join robot_position B 
            on A.robot_position_id=B.robot_position_id 
            where A.robot_position_id = {} and A.robot_path_id = {} and B.status = 0 and B.del_flag = 0
            order by A.order_num;
        """.format(robot_position_id, robot_path_id)
        results = self.get_execute(sql)
        if results:
            return results[0]
        else:
            return {}

    # def get_by_core_cabinet(self, robot_path_id, core_cabinet_id):
    #     query = self.session.query(self.obj).filter(self.obj.robot_path_id == robot_path_id) \
    #         .filter(self.obj.core_cabinet_id == core_cabinet_id).filter(self.obj.status == 0) \
    #         .filter(self.obj.del_flag == 0)
    #     result = query.all()
    #
    #     self.commit()
    #
    #     if result:
    #         results = parse_orm(result)
    #         return results[0]
    #     return None

    def get_by_core_cabinet(self, robot_path_id, core_cabinet_id):
        sql = """
            SELECT *
            FROM robot_path_position_relation A 
            left join robot_position B 
            on A.robot_position_id=B.robot_position_id 
            where A.robot_path_id = {} and B.core_cabinet_id = {} and B.status = 0 and B.del_flag = 0;
        """.format(robot_path_id, core_cabinet_id)
        results = self.get_execute(sql)
        if results:
            return results[0]
        else:
            return {}

    # def get_appoint_location(self, id, position_num, ret, robot_path_id):
    #     # 当前目标点 num，目的目标点position_num
    #     num = ret.get("position_num")
    #     if not num:
    #         num = 1
    #
    #     if int(position_num) > num:
    #         query = self.session.query(self.obj).filter(self.obj.robot_id == id) \
    #             .filter(self.obj.del_flag == 0) \
    #             .filter(self.obj.type == 1) \
    #             .filter(self.obj.robot_path_id == robot_path_id) \
    #             .filter(self.obj.position_num >= int(num)) \
    #             .filter(self.obj.position_num <= int(position_num)) \
    #             .order_by(self.obj.position_num)
    #     else:
    #         query = self.session.query(self.obj).filter(self.obj.robot_id == id) \
    #             .filter(self.obj.del_flag == 0) \
    #             .filter(self.obj.type == 1) \
    #             .filter(self.obj.robot_path_id == robot_path_id) \
    #             .filter(self.obj.position_num >= int(position_num)) \
    #             .filter(self.obj.position_num <= int(num)) \
    #             .order_by(self.obj.position_num.desc())
    #     result = query.all()
    #     self.commit()
    #     results = parse_orm(result)
    #     return results

    def get_appoint_location(self, id, position_num, ret, robot_path_id):
        # 当前目标点 num，目的目标点position_num
        num = ret.get("order_num")
        if not num:
            num = 1

        if int(position_num) > num:
            sql = """
                SELECT *
                FROM robot_path_position_relation A 
                left join robot_position B 
                on A.robot_position_id=B.robot_position_id 
                where A.robot_path_id = {}
                and B.robot_id = {}
                and B.del_flag = 0 
                and B.type = 1 
                and A.order_num >= {}
                and A.order_num <= {}
                order by A.order_num;
            """.format(robot_path_id, id, num, position_num)
        else:
            sql = """
                SELECT *
                FROM robot_path_position_relation A 
                left join robot_position B 
                on A.robot_position_id=B.robot_position_id 
                where A.robot_path_id = {}
                and B.robot_id = {}
                and B.del_flag = 0 
                and B.type = 1 
                and A.order_num >= {}
                and A.order_num <= {}
                order by A.order_num desc;
            """.format(robot_path_id, id, position_num, num)
        results = self.get_execute(sql)
        return results

    # def get_now_location(self, id, robot_path_id):
    #     query = self.session.query(self.obj).filter(self.obj.robot_id == id) \
    #         .filter(self.obj.robot_path_id == robot_path_id).filter(self.obj.current_state == "1").filter(
    #         self.obj.del_flag == 0)
    #     result = query.all()
    #
    #     self.commit()
    #
    #     if result:
    #         results = parse_orm(result)
    #         return results[0]
    #     return None
    def get_now_location(self, id, robot_path_id=None):
        if robot_path_id:
            sql = """
                SELECT *
                FROM robot_path_position_relation A 
                left join robot_position B 
                on A.robot_position_id=B.robot_position_id 
                where A.robot_path_id = {} and B.current_state = 1 and B.robot_id = {} and B.del_flag = 0;
            """.format(robot_path_id, id)
        else:
            sql = """
                SELECT *
                FROM robot_path_position_relation A 
                left join robot_position B 
                on A.robot_position_id=B.robot_position_id 
                where B.current_state = 1 and B.robot_id = {} and B.del_flag = 0;
            """.format(id)
        results = self.get_execute(sql)
        if results:
            return results[0]
        else:
            return {}

    def get_init_point(self, robot_id):
        query = self.session.query(self.obj).filter(self.obj.robot_id == robot_id) \
            .filter(self.obj.current_state == "1").filter(self.obj.del_flag == 0)
        result = query.all()

        self.commit()

        if result:
            results = parse_orm(result)
            return results[0]
        return None

    # def get_init_point(self, robot_id, robot_path_id):
    #     sql = """
    #         SELECT B.*
    #         FROM robot_path_position_relation A
    #         left join robot_position B
    #         on A.robot_position_id=B.robot_position_id
    #         where A.robot_path_id = {} and B.current_state = 1 and B.robot_id = {} and del_flag = 0;
    #     """.format(robot_path_id, robot_id)
    #     results = self.get_execute(sql)
    #     if results:
    #         return results[0]
    #     else:
    #         return {}

    def get_battery_point(self):
        try:
            query = self.session.query(self.obj).filter(self.obj.current_state == "1").filter(self.obj.del_flag == 0)
            result = query.all()

            self.commit()

            if result:
                results = parse_orm(result)
                return results[0]
            return None
        except:
            return None

    def get_now_location_all(self, id):
        try:
            query = self.session.query(self.obj).filter(self.obj.robot_id == id) \
                .filter(self.obj.current_state == "1").filter(self.obj.del_flag == 0)
            result = query.all()

            self.commit()

            if result:
                results = parse_orm(result)
                return results[0]
            return None
        except:
            return None

    # def get_location_all(self, id, robot_path_id):
    #     query = self.session.query(self.obj).filter(self.obj.robot_id == id) \
    #         .filter(self.obj.robot_path_id == robot_path_id) \
    #         .filter(self.obj.del_flag == 0).order_by(self.obj.position_num)
    #     result = query.all()
    #     self.commit()
    #     if result:
    #         results = parse_orm(result)
    #         return results[0]
    #     return None

    def get_location_all(self, id, robot_path_id):
        sql = """
        SELECT *
        FROM robot_path_position_relation A 
        left join robot_position B 
        on A.robot_position_id=B.robot_position_id 
        where A.robot_path_id = {} and B.robot_id = {} and B.del_flag = 0
        order by order_num;
        """.format(robot_path_id, id)
        results = self.get_execute(sql)
        if results:
            return results[0]
        else:
            return {}

    # def get_back_status(self, robot_id, robot_path_id):
    #     query = self.session.query(self.obj.position_num).filter(self.obj.robot_id == robot_id) \
    #         .filter(self.obj.robot_path_id == robot_path_id) \
    #         .filter(self.obj.current_state == "1").filter(self.obj.del_flag == 0)
    #     result = query.one()
    #     ret = result._asdict()
    #     query_2 = self.session.query(self.obj).filter(self.obj.robot_id == robot_id) \
    #         .filter(self.obj.del_flag == 0) \
    #         .filter(self.obj.robot_path_id == robot_path_id) \
    #         .filter(self.obj.position_num <= int(ret.get("position_num"))) \
    #         .order_by(self.obj.position_num.desc()).all()
    #     charging_list = self.get_charging_point(robot_id, robot_path_id)
    #     rets = parse_orm(query_2) + charging_list
    #     self.commit()
    #     return rets

    def get_back_status(self, robot_id):
        sql = """
        SELECT *
        FROM robot_path_position_relation A 
        left join robot_position B 
        on A.robot_position_id=B.robot_position_id 
        where B.robot_id = {} and B.del_flag = 0 and B.current_state = 1;
        """.format(robot_id)
        results = self.get_execute(sql)
        ret = results[0]
        sql_2 = """
        SELECT *
        FROM robot_path_position_relation A 
        left join robot_position B 
        on A.robot_position_id=B.robot_position_id 
        where A.robot_path_id = {} and B.robot_id = {} and B.del_flag = 0 and A.order_num <= {}
        order by order_num desc;
        """.format(int(ret.get("robot_path_id")), robot_id, int(ret.get("order_num")))
        charging_list = self.get_charging_point(robot_id, int(ret.get("robot_path_id")))
        rets = self.get_execute(sql_2) + charging_list
        return rets

    # def get_status(self, robot_id, robot_path_id):
    #     query = self.session.query(self.obj.position_num).filter(self.obj.robot_id == robot_id) \
    #         .filter(self.obj.robot_path_id == robot_path_id) \
    #         .filter(self.obj.current_state == "1").filter(self.obj.del_flag == 0)
    #     result = query.one()
    #     ret = result._asdict()
    #     results = self.get_best_route(robot_id, ret, robot_path_id)
    #     return results

    def get_status(self, robot_id, robot_path_id):
        sql = """
        SELECT *
        FROM robot_path_position_relation A 
        left join robot_position B 
        on A.robot_position_id=B.robot_position_id 
        where A.robot_path_id = {} and B.robot_id = {} and B.del_flag = 0 and B.current_state = 1;
        """.format(robot_path_id, robot_id)
        results = self.get_execute(sql)
        ret = results[0]
        results = self.get_best_route(robot_id, ret, robot_path_id)
        return results

    # def get_position_num_one(self, position_num, robot_path_id):
    #     query = self.session.query(self.obj).filter(self.obj.position_num == position_num) \
    #         .filter(self.obj.robot_path_id == robot_path_id) \
    #         .filter(self.obj.del_flag == 0)
    #     result = query.all()
    #     self.commit()
    #
    #     results = parse_orm(result)[0]
    #     return results

    def get_position_num_one(self, position_num, robot_path_id):
        sql = """
        SELECT *
        FROM robot_path_position_relation A 
        left join robot_position B 
        on A.robot_position_id=B.robot_position_id 
        where A.robot_path_id = {} and A.order_num = {} and B.del_flag = 0;
        """.format(robot_path_id, position_num)
        results = self.get_execute(sql)
        if results:
            return results[0]
        return {}

    # def get_best_route(self, robot_id, ret, robot_path_id):
    #     query_1 = self.session.query(self.obj).filter(self.obj.robot_id == robot_id) \
    #         .filter(self.obj.del_flag == 0) \
    #         .filter(self.obj.robot_path_id == robot_path_id) \
    #         .filter(self.obj.position_num >= int(ret.get("position_num"))) \
    #         .order_by(self.obj.position_num).all()
    #
    #     query_2 = self.session.query(self.obj).filter(self.obj.robot_id == robot_id) \
    #         .filter(self.obj.del_flag == 0) \
    #         .filter(self.obj.robot_path_id == robot_path_id) \
    #         .filter(self.obj.position_num <= int(ret.get("position_num"))) \
    #         .order_by(self.obj.position_num.desc()).all()
    #
    #     if len(query_1) - 2 > len(query_2):
    #         charging_list = self.get_charging_point(robot_id, robot_path_id)
    #         rets = parse_orm(query_2) + charging_list
    #         self.commit()
    #         return rets
    #     self.commit()
    #     return parse_orm(query_1)

    def get_best_route(self, robot_id, ret, robot_path_id):
        sql_1 = """
                SELECT *
                FROM robot_path_position_relation A 
                left join robot_position B 
                on A.robot_position_id=B.robot_position_id 
                where A.robot_path_id = {} and B.robot_id = {} and A.order_num >= {} and B.del_flag = 0
                order by A.order_num;
                """.format(robot_path_id, robot_id, int(ret.get("order_num")))
        sql_2 = """
                SELECT *
                FROM robot_path_position_relation A 
                left join robot_position B 
                on A.robot_position_id=B.robot_position_id 
                where A.robot_path_id = {} and B.robot_id = {} and A.order_num <= {} and B.del_flag = 0
                order by A.order_num desc;
                """.format(robot_path_id, robot_id, int(ret.get("order_num")))
        query_1 = self.get_execute(sql_1)
        query_2 = self.get_execute(sql_2)

        if len(query_1) - 2 > len(query_2):
            charging_list = self.get_charging_point(robot_id, robot_path_id)
            rets = query_2 + charging_list
            return rets
        return query_1

    # def get_charging_point(self, id, robot_path_id):
    #     query = self.session.query(self.obj). \
    #         filter(self.obj.robot_id == id, or_(self.obj.type == 3,
    #                                             self.obj.type == 4)) \
    #         .filter(self.obj.robot_path_id == robot_path_id) \
    #         .filter(self.obj.del_flag == 0).order_by(self.obj.type)
    #     result = query.all()
    #     results = parse_orm(result)
    #     return results

    def get_charging_point(self, id, robot_path_id):
        sql = """
            SELECT *
            FROM robot_path_position_relation A 
            left join robot_position B 
            on A.robot_position_id=B.robot_position_id 
            where A.robot_path_id = {}
            and B.robot_id = {}
            and B.del_flag = 0
            and (B.type =3 or B.type =4)
            order by B.type;
            """.format(robot_path_id, id)
        results = self.get_execute(sql)
        return results

    def get_by_position_id(self, robot_position_id:int):
        query = self.session.query(self.obj).filter(self.obj.robot_position_id == robot_position_id)
        results = parse_orm(query.all())
        self.commit()

        if not results:
            return results
        return results[0]

    def get_one(self, **kwargs):
        for key, val in kwargs.items():
            if hasattr(self.obj, key):
                query = self.session.query(self.obj).filter(getattr(self.obj, key) == val).filter(
                    self.obj.del_flag == 0).first()
                return query

    def get_alive_action_route(self, robot_id, robot_position_id):
        query = self.session.query(self.obj).filter(self.obj.robot_id == robot_id).filter(
            self.obj.robot_position_id == robot_position_id).filter(self.obj.current_state == "1")
        results = parse_orm(query.all())
        self.commit()

        if not results:
            return results
        return results[0]

    def get_now_position(self):
        query = self.session.query(self.obj).filter(self.obj.current_state == "1").filter(self.obj.del_flag == 0)
        result = query.all()

        self.commit()

        if result:
            results = parse_orm(result)
            return results[0].get("type")
        return None


__all__ = ['DBRobotPosition']

if __name__ == '__main__':
    pass
