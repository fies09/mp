# api目录结构

####1. 目录结构描述 
├──  configs               // 配置文件目录
│    ├──  __init__.py        // 数据库连接，api接口，中间件等配置信息
│    ├──  log.py             // log的输出配置文件
├──  core                  // 封装调用方法目录
│    ├──  dict_config.py     // 配置需要编码转换文件
│    ├──  db.py              // 封装mysql数据库连接配置
│    ├──  redis_db.py        // 封装redis数据库连接配置
├──  data_pkg              // 文件存储目录
│    ├──  industry_path      // 工业相机存储路径
│    ├──  pointer_path       // 光学相机存储路径
│    ├──  web_path           // web调用是存储路径
├──  Industrial_camera     // 工业相机调度目录
│    ├──  MvImport           // 工业相机SDK存储
│    ├──  industry_camera.py // 工业相机调用方法
├──  logs                  // 日志存放目录
├──  modules               // 硬件交互模块方法存放目录
│    ├──  client.py          // 升降杆控制方法
├──  schema                // 和mysql数据库交互目录
│    ├──  models.py          // ORM关系映射模板文件
│    ├──  deploy.sh          // 生成models.py文件脚本
├──  test                  // 测试目录
├──  utils                 // 常用配置工具目录
│    ├──  alchemy_encoder.py  // 将sql查出的地址解析为字典脚本
│    ├──  convert.py          // 数据类型转换脚本
│    ├──  file_path.py        // 目录路径生成脚本
│    ├──  get_soc.py          // 获取主机状态信息脚本
│    ├──  msg.py              // api返回数据规范脚本
│    ├──  translator.py       // sql查询拼接条件脚本
├──  API.py                // API接口主程序


####2. api返回值：

返回值定义格式：
```python3
# 返回成功
msg = Msg()
return msg.meta({'key': 'val'}).success(msg='str', result=results).json()

# 返回失败
msg = Msg()
return msg.fail(msg='str失败').json()
```

返回值返回格式：
```python3
# 接收成功
{'code': 0, 'status': True, 'msg': 'str', 'data': {'meta': {'key': 'val'}, 'result': {}}}

# 接收失败
{'code': 1, 'status': False, 'msg': 'str失败', 'data': {}}
```
- 说明： 
- code：0表示正常 1表示异常 
- status：True表示正常 False表示异常
- msg：说明信息，str类型填写
- data: 返回数据，目前容量为两个，mete一般返回数量，result返回数据结果。
- 特殊code码：2表示硬件错误，3表示系统错误，4表示意外错误，5表示参数错误，6表示验证错误




