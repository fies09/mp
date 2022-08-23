# ! /usr/bin/env python3
# .-*- coding:utf-8 .-*-
import pika
import threading
import time

# rabbitmq 配置信息
from configs import db_rabbit_mq
from configs.log import logger
from utils.convert import dict_to_json


class RabbitMQServer(object):
    _instance_lock = threading.Lock()

    def __init__(self):
        self.recv_serverid = ""
        self.send_serverid = ""
        self.exchange = None
        self.connection = None
        self.channel = None
        self.queue = None

    def reconnect(self):
        try:
            if self.connection and not self.connection.is_closed:
                self.connection.close()

            credentials = pika.PlainCredentials(db_rabbit_mq.get("user"), db_rabbit_mq.get("passwd"))
            parameters = pika.ConnectionParameters(db_rabbit_mq.get("host"), db_rabbit_mq.get("port"),
                                                   db_rabbit_mq.get("vhost"),
                                                   credentials, heartbeat=60)
            self.connection = pika.BlockingConnection(parameters)

            self.channel = self.connection.channel()
            if self.exchange:
                self.channel.exchange_declare(exchange=self.exchange, exchange_type="direct", durable=True)
        except Exception as e:
            logger.error(e)


class RabbitPublisher(RabbitMQServer):
    def __init__(self):
        super(RabbitPublisher, self).__init__()

    def start_publish(self, data):
        try:
            self.reconnect()
            self.channel.queue_declare(queue=self.queue, durable=True)
            self.channel.queue_bind(queue=self.queue, exchange=self.exchange, routing_key=self.send_serverid, )
            self.channel.basic_publish(exchange=self.exchange, routing_key=self.send_serverid, body=data)
        except Exception as e:
            logger.error(e)
            self.reconnect()

    # 1)路由名为:warn，绑定队列 alarm,routingKey 为 warn_info
    # 路由 管道 数据
    @classmethod
    def run(cls, exchange, routing_key, queue, data):
        publish = cls()
        publish.send_serverid = routing_key
        publish.exchange = exchange
        publish.queue = queue
        ret = dict_to_json(data)
        publish.start_publish(ret)
