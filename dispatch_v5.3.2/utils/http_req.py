#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time       : 2022/5/20 9:56
# @Author     : xawangl
# @Email      : 17609265682@163.com
# @File       : http_req.py
# @description: ""
import json
import requests


class HttpClient(object):
    def __init__(self):
        self.timeout = 15

    def http_client(self, method: str, url: str, data: dict, header: dict):
        """
        http_client
        :param method:
        :param url:
        :param data:
        :param header:
        :return: http-response
        """
        if method == "POST":
            if header["Content-Type"] == "application/json":
                payload = json.dumps(data)
                response = requests.request("POST", url, headers=header, data=payload, timeout=self.timeout)
                return True, response
        elif method == "GET":
            payload = json.dumps(data)
            response = requests.request("GET", url, headers=header, data=payload, timeout=self.timeout)
            return True, response
        else:
            return False, None