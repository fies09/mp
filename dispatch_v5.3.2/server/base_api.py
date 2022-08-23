# -*- coding: utf-8 -*-

import json
from flask import make_response
from urllib import parse

class Dict2Obj(dict):
    """
    Dict2Obj
    字典转对象
    """
    __setattr__ = dict.__setitem__
    __getattr__ = dict.__getitem__


class ApiBase(object):
    """
    ApiBase
    封装了api请求和响应的基础方法,增加获取参数和响应请求的便利性
    """
    def __init__(self):
        super().__init__()
        # self.log = Logger().logger

    def make_url(self, url, params=None):
        if params is None:
            params = {}
        paramsStr = '?' + parse.urlencode(params) if len(params) > 0 else ''
        return url + paramsStr

    def get_input(self, request):
        """
        请求参数解析
        :param request:
        :return:
        """
        user_agent = request.headers.get("User-Agent")
        ip = request.remote_addr
        url = request.url

        if request.method == 'GET':
            return Dict2Obj(request.args.items())
        else:
            if request.headers.get("Content-Type") == "application/json":
                return Dict2Obj(request.get_json())
            elif request.headers.get("Content-Type") and ("multipart/form-data" in request.headers.get("Content-Type")):
                return Dict2Obj(dict(request.form.items()))
            else:
                return Dict2Obj({})

    # def response(self, http_code=200, data=None, total=0, code=ErrorCode.msg_10000, msg="Success！"):
        """
        响应请求
        :param http_code:
        :param data:
        :param code:
        :param msg:
        :return:
        """
        if msg == "Success！":
            new_msg = code[1]
        else:
            new_msg = msg

        res_data = {
            "code": code[0],
            "msg": new_msg,
            "data": json.loads(json.dumps(data, cls=JsonEncoder)),
            "total": total
        }
        res = make_response(res_data, http_code)
        return res


