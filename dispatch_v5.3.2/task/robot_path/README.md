
# pathplan

路径规划算法工程
=======
## 全局最短路径规划算法说明(v5.3.2)
*updated han 20220513
```
   * 输入路径序列-> 数据库读取路由表 -> 路径规划 ->结果输出
```

# 1.工程组织
```
go_path_planning
├─ configs           # 配置文件目录
│  └─ planning.yaml
├─ env              # 全局最短路径算法环境说明(工控机)
│  └─ requirements.txt      
├─ log.py            # 日至文件
├─ logs              # 输出日至目录
│  └─ path_plan.log
├─ path_plan.py  # 路径规划算法脚本
├─ tools              # 依赖函数文件
│  └─ Dijkstra.py
│  └─ SA.py
├─  utils                # 依赖文件 
└  └─ load_config.py  # 加载配置文件

```

# 2.全局最短路径规划算法配置
在配置文件'configs/pathplan.yaml'：
```
# 数据库设置
mysql:
    host: "10.173.27.120"
    database: "v5.3_robot"
    user: "root"
    password: "mengpa@.."
    port: 3306
    charset: "utf8"

# 数据库表设置-构建路由表所需
table:
    position_table: "test_robot_position"        # 机器人位置表名称
    begin_id: "position_num"   # 巡检点编号
    position: "position"       # 巡检点位置
    stop_id: "stop_id"         # 巡检点的连通关系(可到达点)

    # 规划过程所需表
    route_table: "test_robot_position_route"   # 新建的路由表名称
    BeginId: "BeginId"        # 起点
    StopId: "StopId"          # 终点
    Distance: "Distance"      # 距离
```
# 3. 环境搭建和部署
在环境文件'env/requirements.txt'：
所用包目前和调度所用包版本一致，不需额外安装

```
# 路径规划PC部署python环境文档
#------------------------
# 版本：V5.3.2
# 更新日期：20220517
# 基础版本：python3.6.9
# 清华源：https://pypi.tuna.tsinghua.edu.cn/simple
# pip>=20.1.1
#------------------------

pyyaml == 6.0
pandas == 0.22.0
numpy == 1.19.5
pymysql == 1.0.2

# 服务所需
#flask == 2.0.1
#flask_cors == 3.0.10
```
# 4. 数据库路由表robot_position_route配置
```
  # 表存储为三个字段：BeginId、StopId、Distance表示起点编号，终点编号，两点间的距离
  (1)其中begin_id与stop_id与数据库表`robot_position`的'position_num'(巡检点应为编号(1...n)，唯一标识)相对应。
  (2)distance为两点间的距离。
     若两点间不可直接连通，distance = float(-1)；
     若两点可以直接连通，则通过数据库表`robot_position`的'position'计算。距离为两个巡检点各坐标数值差绝对值的最大值，具体公式为：
     distance = float(max|x2-x1|,|y2-y1|)，其中(x1，y1)为BeginId的坐标，(x2，y2)为StopId的坐标。
     
  # 设计表各字段为：
    BeginId   INT(11)    NOT NULL
    StopId    INT(11)    NOT NULL
    Distance    FLOAT    NOT NULL 
```

# 4. 脚本运行说明

```
  # 分为三个主函数，分别为：
  场景一：两点间路径规划
  (1)run_two_points_plan():
                  输入：两点路径序列，即：[起始点，目标点]
                  输出：结果码，巡检点顺序，规划结果，即：code,[...],[...]
                 
  场景二：多点巡检路径规划
  (2)run_multi_points_plan():
                  输入：多点路径序列，即：[起始点，目标点1，...，目标点N]
                  输出：结果码，巡检点顺序，规划结果，即：code,[...],[...]
                    
  场景三：机器人在两点间避障后路径规划
  (3)run_avoid_plan():
                  输入：([起始点，目标点]，机器人到达的上一个点，机器人本该到达的下一个点)
                  输出：结果码，巡检点顺序，规划结果，即：code,[...],[...]
                 
  返回说明：
         code        巡检点顺序    规划结果           说明
         1             [...]      [...]            规划成功，有结果输出
         0             [...]      []               规划失败，无结果输出
         -1            [...]      []               参数错误/不合规的参数（eg:巡检点编号不存在）
         -2            [...]      []               数据库连接失败
```

