./run_docker.sh
# 1. 彻底删除旧编译缓存
rm -rf build/ devel/

# 2. 建立一个软链接
ln -s /home/galbot/Ipopt_pkg/acados /home/galbot/acados

# 3. 重新进行纯净编译
catkin_make

# 4. 刷新环境变量
source devel/setup.bash

# 5. 启动仿真
roslaunch nmpc_ctrl gazebo_mpc.launch

# 拷贝本地版本的 casadi 和 acados 到当前目录，用于构建 Docker
cp -r /home/galbot/casadi-3.5.5 ./casadi_local
cp -r /home/galbot/Ipopt_pkg/acados ./acados_local
