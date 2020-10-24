# OpenDRIVEViewer3D
A lightweight 3D viewer of OpenDRIVE map format

## Platform
Target to Linux, Windows. Start with Ubuntu only for the time being.

## Requirements

### Build Requirements
Toolchain:
* CMake >= 3.17
* conan >= 1.30.2
* C++14 compliant compiler

Pre-installed packages:
`
sudo apt install libxcb-render-util0-dev libxcb-xkb-dev xorg-dev libgl1-mesa-dev libxcb-icccm4-dev libxcb-image0-dev libxcb-keysyms1-dev libxcb-xinerama0-dev
`

## How to build

`
mkdir Build
cd Build
conan install .. --build missing
make
`
