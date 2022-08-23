import json
import threading
import time

from pika.exceptions import ConnectionClosed, ChannelClosed

from configs import db_rabbit_mq
from configs.log import logger
from core.rabbitmq_db import RabbitMQServer, RabbitPublisher
from core.redis_interactive import RedisPipe
from utils.convert import dict_to_json


class RabbitComsumer(RabbitMQServer):

    def __init__(self):
        super(RabbitComsumer, self).__init__()
        # self.redis_pipe = RedisPipe()

    def consumer_callback(self, ch, method, properties, body):
        ch.basic_ack(delivery_tag=method.delivery_tag)
        data = json.loads(body)
        RedisPipe().push_data("TaskInfo", json.dumps(data))
        time.sleep(3)
        RedisPipe().del_list_data("TaskInfo")

    def start_consumer(self):
        while True:
            # try:
            #     self.reconnect()
            #     self.channel.queue_declare("/", exclusive=True)
            #     self.channel.queue_bind(exchange="robot", queue="task",
            #                             routing_key="taskInfo")
            #     self.channel.basic_consume(queue="task",
            #                                on_message_callback=self.consumer_callback, auto_ack=False)
            #     self.channel.start_consuming()
            # except ConnectionClosed as e:
            #     self.reconnect()
            #     logger.error(e)
            #     time.sleep(2)
            # except ChannelClosed as e:
            #     logger.error(e)
            #     self.reconnect()
            #     time.sleep(2)
            # except Exception as e:
            #     logger.error(e)
            #     self.reconnect()
            #     time.sleep(2)
            self.reconnect()
            self.channel.queue_declare("/", exclusive=True)
            self.channel.queue_bind(exchange="robot", queue="task",
                                    routing_key="taskInfo")
            self.channel.basic_consume(queue="task",
                                       on_message_callback=self.consumer_callback, auto_ack=False)
            self.channel.start_consuming()

    @classmethod
    def run(cls, recv_serverid):
        consumer = cls()
        consumer.recv_serverid = recv_serverid
        consumer.start_consumer()


def publish_mq(rout, routing_key, queue_name, data):
    rout = db_rabbit_mq.get(rout)
    routing_key = db_rabbit_mq.get(routing_key)
    warn_queue = db_rabbit_mq.get(queue_name)
    RabbitPublisher.run(rout, routing_key, warn_queue, data)


if __name__ == '__main__':
    # recv_serverid = db_rabbit_mq.get("rout_send_robotStatus")
    # RabbitComsumer.run(recv_serverid)
    data = {'inspect_project_detail_id': 1036738199, 'createTime': '2021-08-19 16:38:30', 'messageData': '自动巡检'}
    RedisPipe().push_data("TaskInfo", dict_to_json(data))
