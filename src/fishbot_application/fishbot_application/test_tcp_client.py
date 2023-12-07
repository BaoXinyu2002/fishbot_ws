import socket
import sys

# 创建 socket 对象
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# 获取本地主机名
host = socket.gethostname()

# 设置端口号
port = 12345

# 连接服务，指定主机和端口
client_socket.connect((host, port))

# 发送数据
client_socket.send("x=10, y=20, z=30".encode('utf-8'))

# 关闭连接
client_socket.close()
