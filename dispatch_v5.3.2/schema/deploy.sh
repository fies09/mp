#!/bin/bash

# pip3 install sqlacodegen
# pip3 install mysqlclient

sqlacodegen --outfile=models.py mysql://root:mengpa@..@192.168.10.100:3306/v5.3_robot?charset=utf8mb4
## table必须要有主键, 否则转化成的是Table类型而不是class

#sqlacodegen --outfile=models.py mysql://robot:123@10.170.10.140:3306/v5_robot?charset=utf8mb4


