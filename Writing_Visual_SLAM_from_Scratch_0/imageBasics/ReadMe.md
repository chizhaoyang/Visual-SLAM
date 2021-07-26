Example to show how to use OpenCV to open an image.

### Requirement
[OpenCV](https://github.com/opencv/opencv)

### Test Environment
OpenCV 4.5.0
Ubuntu 20.04

### How to use
```
mkdir build && cd build
cmake ..
make
# run image basics
./imageBasics ../ubuntu.png
# run undistort image
./undistortImage
```