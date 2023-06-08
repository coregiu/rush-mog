# 源码列表
# set(SRC_LIST "main.c")
file(GLOB SRC_LIST *.c)

# 将源代码添加到此项目的可执行文件。
add_executable (${PROJECT_NAME} ${SRC_LIST})
