---------------直接命令编译-----------------------
1.首先安装MSCL库，具体方法参照https://github.com/LORD-MicroStrain/MSCL/blob/master/HowToUseMSCL.md

2.编译命令
g++ -I/usr/share/c++-mscl/source -I/usr/share/c++-mscl/Boost/include -o test main.cpp -L/usr/share/c++-mscl -lmscl -lstdc++ -lpthread -std=c++11

3.给端口赋予权限
sudo chmod 666 /dev/ttyACM0

4.设置环境变量
export LD_LIBRARY_PATH=/usr/share/c++-mscl:$LD_LIBRARY_PATH

5.运行可执行文件
./test

------------------CMake-----------------------------
使用CMakeList.txt可以避免复杂的编译命令，不需要在每次执行的时候都要手动设置环境变量

步骤：
0.安装库
1.打开终端，mkdir build
2.cd build
3.cmake ..
4.赋权限
5../test
