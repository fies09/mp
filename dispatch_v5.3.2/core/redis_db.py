# -*- coding: utf-8 -*-
import redis

from configs import db_redis

pool = redis.ConnectionPool(**db_redis)
RedisClient = redis.Redis(connection_pool=pool)

__all__ = ['RedisClient']
