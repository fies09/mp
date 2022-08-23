# coding:utf-8

#--------------------------------
# 服务端
# 单个应用场景多功能
#-------------------------------


# 第一行，导入包
from flask import Flask

# 第二行，创建对象
app = Flask(__name__)

# 第三行，创建路由
@app.route('/')

# 第四\五行，创建视图
def hello_world():
    return 'Hello, World!'


if __name__=='__main__':
    IP = '0.0.0.0'
    PORT = '8082'
    app.run(host=IP, port=PORT)

