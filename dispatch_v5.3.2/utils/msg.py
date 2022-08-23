# coding: utf-8

import json


class Msg(object):
    def __init__(self):
        self.msg = {
            'code': 1,
            'status': True,
            'msg': '',
            'data': {

            }
        }

    def convert_str(self):
        # data = json.dumps(self.msg)
        # return str(data)

        return str(self.msg)

    def json(self):
        return json.dumps(self.msg)

    def meta(self, data):
        self.msg['data']['meta'] = data
        return self

    def fail(self, msg):
        code = 1
        self.msg['status'] = False
        self.msg['code'] = code
        self.msg['msg'] = msg
        return self

    def success(self, result, msg):
        code = 0
        self.msg['code'] = code
        self.msg['data'] = result
        self.msg['msg'] = msg
        return self

    def api_not_exist(self):
        code = 2
        self.msg['status'] = False
        self.msg['code'] = code
        self.msg['msg'] = 'api is not exist'
        return self

    def system_error(self):
        code = 3
        self.msg['status'] = False
        self.msg['code'] = code
        self.msg['msg'] = 'system error'
        return self

    def unexpected_error(self):
        code = 4
        self.msg['status'] = False
        self.msg['code'] = code
        self.msg['msg'] = 'unexpected error'
        return self

    def params_error(self):
        code = 5
        self.msg['status'] = False
        self.msg['code'] = code
        self.msg['msg'] = 'params error'
        return self

    def illegal_access_error(self):
        code = 6
        self.msg['status'] = False
        self.msg['code'] = code
        self.msg['msg'] = 'illegal access error'
        return self

    def method_not_allowed_error(self):
        code = 7
        self.msg['status'] = False
        self.msg['code'] = code
        self.msg['msg'] = 'method not allowed error'
        return self

    def info(self, msg):
        code = 8
        self.msg['status'] = False
        self.msg['code'] = code
        self.msg['msg'] = msg
        return self
