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
