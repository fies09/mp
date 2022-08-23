# coding:utf-8
#--------------------------------
# 服务端
# 单个应用场景具备多功能
# 支持 GET 和 POST 两种请求方式
# 部分功能接口需要传入参数
# ***采用gunicorn方式部署
#-------------------------------

from flask import Flask
from flask import request, json

app = Flask(__name__)

# 路由1--功能1
@app.route('/', methods=['GET', 'POST'])
def hello_world():
    return 'Hello, World!'

# 路由2--功能2
@app.route('/user/<name>', methods=['GET', 'POST'])
def hello(name):
    return 'Hello, %s!' % name


# 路由3--功能3-需要参数
@app.route('/login', methods=['GET', 'POST'])
def login():
    """输入接口字段
        {'name': STR}
    """
    # GET
    if request.method == 'GET':
        data = request.data.decode('utf-8')
        data = json.loads(data)
        print(f"GET: {request.headers.values} and get para:{data}")

    # POST请求
    if request.method == 'POST':
        data = request.data.decode('utf-8')
        data = json.loads(data)
        print(f"Post: {request.headers.values} and get para:{data}")

    # 参数校验
    if not isinstance(data, dict) or 'name' not in data.keys():
        return 'Failed to login, None!'        

    # 参数读取
    name = data['name']
    
    # 执行逻辑
    print(name)

    return 'Successfuly login %s!' % name


if __name__=='__main__':
    # 启动方式1 
    IP = '0.0.0.0'
    PORT = '8082'
    app.run(host=IP, port=PORT)
    
    # 启动方式2：
    """
    gunicorn -c config/gunicorn.py app3:app
    """



