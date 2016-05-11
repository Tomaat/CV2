@echo off

set _libs=-IC:\Cstuff\Eigen\include -IC:\Cstuff\opencv3\opencv\build\include -IC:\Cstuff\boost_1_60_0
set _std=c++11
set _gcc=gcc

if %1 == all %_gcc% -std=%_std% %_libs% 
	

::gcc -std=c++11 -I -I; Frame3D.cpp -o ys_test.o