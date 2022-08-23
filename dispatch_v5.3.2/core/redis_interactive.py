from core.redis_db import RedisClient


class RedisHelper(object):

    def __init__(self):
        self.__conn = RedisClient

    # 发布
    def public(self, pub_name, msg):
        self.__conn.publish(pub_name, msg)
        return True

    # 订阅
    def subscribe(self, pub_name):
        pub = self.__conn.pubsub()
        pub.subscribe(pub_name)
        pub.parse_response()
        return pub


class RedisPipe(object):

    def __init__(self, name=None):
        self.__conn = RedisClient
        self.queue_name = name

    # 推送数据
    def push_data(self, pub_name, msg):
        self.__conn.lpush(pub_name, msg)
        return True

    # 接收数据
    def pop_data(self, pub_name):
        ret = self.__conn.lpop(pub_name)
        return ret

    def del_list_data(self, pub_name):
        ret = self.__conn.delete(pub_name)
        return ret

    def del_data(self):
        ret = self.__conn.flushdb()
        return ret

    # time（s）
    def set_data(self, value, time_data=None):
        ret = self.__conn.set(self.queue_name, str(value), ex=time_data)
        return ret

    def get_data(self):
        ret = self.__conn.get(self.queue_name)
        return ret
