# Stereo

## Compile
```bash
mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j4
```
### Dependencies
* Boost 1.65
* OpenCV 3.4
    * Intel MKL (optional)
    * CUDA (optional)


## Stereo viewer
```bash
./stereo_view
```

## Corner finder
```bash
./corner_finder [cam0] [cam1]
```

![alt text](https://github.com/phg1024/Stereo/blob/master/images/corners.png "Corner finder test")

## Single Camera Calibration