cmake_minimum_required(VERSION 3.0.2)
project(galbot_s1_v1_description)  # 确保这里的名字和 package.xml 里的名字一致

find_package(catkin REQUIRED)

catkin_package()

# 下面这些是安装规则，保证 install 后也能找到模型文件
foreach(dir config launch meshes urdf)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()