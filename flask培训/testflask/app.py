# coding:utf-8
#--------------------------------
# 服务端
# 单个应用场景具备多功能
# ***支持 GET 和 POST 两种请求方式
#-------------------------------

from flask import Flask
app = Flask(__name__)

# 路由1--功能1
@app.route('/', methods=['GET', 'POST'])
def hello_world():
    return 'Hello, World!'

# 路由2--功能2
@app.route('/user/<name>', methods=['GET', 'POST'])
def hello(name):
    return 'Hello, %s!' % name

# 路由3--功能3 需传入参数
@app.route('/login', methods=['GET', 'POST'])
def hello(name):
    return 'Hello, %s!' % name


if __name__=='__main__':
    IP = '0.0.0.0'
    PORT = '8082'
    app.run(host=IP, port=PORT)



