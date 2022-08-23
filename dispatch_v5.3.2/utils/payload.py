import json


class PayLoad:
    def __init__(self):
        self.payload = {
            'pi'  : 1,
            'ps'  : 10,
            'sort': False,
            'data': []
        }

    def json(self):
        return json.dumps(self.payload)

    def dict(self):
        return self.payload

    # todo：判断是不是json，不是json要转化一下
    def data(self, d):
        self.payload['data'] = d
        return self

    def sort(self, d):
        self.payload['sort'] = d
        return self

    def page(self, pi=1, ps=10):
        self.payload['pi'] = pi
        self.payload['ps'] = ps
        return self
