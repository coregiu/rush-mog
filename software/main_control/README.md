# CMake-based STM32F103C8 Template (Blue Pill)
---
Based on https://github.com/ahessling/STM32F4Template

As an addition;
* style fixer tool (astyle) can be use with `./code-format.sh`
* `make monitor` will open picocom serial terminal
---
### Build
```
./build.sh
cd build
make
```
### Download binary
```
make flash  # this will run openocd if it is found during cmake process
```
### Serial terminal
```
make monitor # this will run picocom with 115200 8N1
```
Of course you can run all commands in one shot

`make flash monitor`

### Debug
```
make gdb # this will run gdb and openocd in the background.

todo: kill openocd upon gdb termination
```

# 手动编译烧录
### 编译
```shell
$ mkdir build
$ cd build
$ cmake -G "Unix Makefiles" -D "CMAKE_TOOLCHAIN_FILE=../CMake/GNU-ARM-Toolchain.cmake" ..
$ ll src
total 1.0M
drwxrwxr-x  3 eagle 4.0K 6月  11 23:15 .
drwxrwxr-x  6 eagle 4.0K 6月  11 23:15 ..
-rwxrwxr-x  1 eagle  18K 6月  11 23:15 ago_maincontrol.bin
-rwxrwxr-x  1 eagle 360K 6月  11 23:15 ago_maincontrol.elf
-rw-rw-r--  1 eagle  51K 6月  11 23:15 ago_maincontrol.hex
-rw-rw-r--  1 eagle 433K 6月  11 23:15 ago_maincontrol.lss
-rw-rw-r--  1 eagle 213K 6月  11 23:15 ago_maincontrol.map
drwxrwxr-x 11 eagle 4.0K 6月  11 23:15 CMakeFiles
-rw-rw-r--  1 eagle 1017 1月  14 08:26 cmake_install.cmake
-rw-rw-r--  1 eagle  27K 1月  14 08:26 Makefile
```

### 烧录
打开st-link软件

![st-link](st-link.png)

点击连接，选择ago_maincontrol.bin文件，点击烧录按钮。