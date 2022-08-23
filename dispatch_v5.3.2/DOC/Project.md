# 启动ros机器人 
```
roslaunch moss_launch moss_navigation.launch
roslaunch moss_launch moss_navigation1.launch

```
- 启动动环订阅
```/home/robot/dcatkin_ws/devel/lib/seveninone/seveninone_node```

- 杀死话题
```pkill ros```

# ubuntu环境配置
- 需要安装python插件，sqlalchemy 和 pymysql
- 当在ubuntu中数据库连接错误时候，修改sqlalchemy中的源码，对错误地方的格式进行修改。

# ros python3兼容问题：解决安装
- 需要安装catkin-tools rospkg  
- python文件第一行加上#!/user/bin/python3
-若调用python3的包出现报错是在python2.7路径下找时需在import上行加入sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

#动环问题
- 动环所有数据都从sub_sql中取，温湿度是单独的传感器不在七合一中 
- 现在能启动的传感器有七合一 噪声二氧化硫和硫化氢和电池的信息

#自动回充问题
- 1.tf包不支持python3 换成 quaternions==0.4 来计算求解欧拉角到四元数的变换结果
- 2.需要安装quaternions==0.4 和 numpy 。因为python版本为3.5，所以需要安装支持
的numpy版本,两种方法：
``` 
# 通过apt安装
sudo apt-get install python3-numpy

# 通过pip安装
# 需要安装numpy == 1.11.3 版本


'''

# 获取pip-python IP地址
$wget https://bootstrap.pypa.io/pip/3.5/get-pip.py

# pip版本
- pip == 20.3.4 

#  opencv的问题
## python3导入 cv2 出现依赖错误
- 1. pip3 install opencv-contrib-python == 4.4.0.42
- 2. pip3 install opencv-python == 3.4.0.12
- 3. sudo mv cv2.so cv2_ros.so

# 依赖的包
- timeout-decorator

# 海康sdk抓图出现的问题
- sdk报41错误
- 原因：主控openssl 版本为1.0.2，摄像头加密传输不支持此版本。
- 处理方法：
- 1. 不进行ssl加密，删除libssl.so文件
- 2. 将ubuntu系统openssl版本降低值1.0.0
- 3. 升级摄像头固件版本
# ros导入订阅数据类型问题
- 路径问题：默认从/ros/kinetic/lib/python2.7/下找，解决方法把包拷入到自己的工程目录下/lib/python2.7/kinetic/   就解决了