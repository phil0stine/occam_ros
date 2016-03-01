
==========================================================================================
== Visual Studio 2013 compilation
== assuming this directory is c:\indigosdk

mkdir msvc2013
cd msvc2013
c:/vs2013/VC/vcvarsall.bat
del CMakeCache.txt && cmake -G "Visual Studio 12" -DOpenCV_DIR=c:/opencv-git/msvc2013 -DUSE_OPENCV=1 -DUSE_OPENGL=1 ..
c:/vs2013/VC/vcvarsall.bat && cd c:/indigosdk/msvc2013 && msbuild /p:Configuration=Release /nologo /v:m indigosdk.sln

==========================================================================================
== Ubuntu compilation

On Ubuntu, if you configure with OpenGL support you'll need to install X11 and GL related libs, as well as libxxf86vm-dev.

## build with OpenCV and OpenGL support
mkdir -p release && cd -p release
rm -f CMakeCache.txt && cmake -DOpenCV_DIR=~/opencv/build -DUSE_OPENCV=1 -DUSE_OPENGL=1 ..

## build with OpenGL support but no OpenCV support
mkdir -p release && cd -p release
rm -f CMakeCache.txt && cmake -DUSE_OPENCV=0 -DUSE_OPENGL=1 ..

## build with neither OpenCV nor OpenGL support
mkdir -p release && cd -p release
rm -f CMakeCache.txt && cmake -DUSE_OPENCV=0 -DUSE_OPENGL=0 ..

