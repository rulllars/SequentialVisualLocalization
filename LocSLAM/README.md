# LocSLAM
**Authors:** Erik Stenborg, Torsten Sattler and [Lars Hammarstrand](https://www.chalmers.se/en/staff/Pages/lars-hammarstrand.aspx).

LocSLAM is a hybrid Localization/SLAM library for **multi-camera platforms** based on [ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2). It is a hybrid SLAM system in that it, not only computes a sparse 3D reconstruction of the environment, but also computes the platform trajectory to scale and in relation to a pre-built fixed map based on images from a set of cameras, a set of matched local features between the images and the pre-build map and an odometry signal (IMU). The purpose of the library is to demonstrate how odometry constraints (visual and IMU) from a sequence of images can be used to solve challenging visual localization problems, such as changing conditions due to day/night and/or seasonal changes.

We provide an example for how to run LocSLAM on the [sequential CMU Seasons](https://data.ciirc.cvut.cz/public/projects/2020VisualLocalization/) dataset. Using this example it is possible to, e.g., experiment with using different local features for the fixed map (D2-net and SIFT are provided) to improve the localization performance. The result can easily be evaluated using the public [visual localization benchmark](http://visuallocalization.net).

### Related Publications:

Erik Stenborg, Torsten Sattler and Lars Hammarstrand. **Using Image Sequences for Long-Term Visual Localization**, *2020 International Conference on 3D Vision (3DV)*, 2020, pp. 938-948, doi: 10.1109/3DV50981.2020.00104. **[PDF](https://research.chalmers.se/publication/522731/file/522731_Fulltext.pdf)**

[![](https://img.youtube.com/vi/W4gooWGs7Q0/0.jpg)](https://www.youtube.com/watch?v=W4gooWGs7Q0)

# 1. License

Our tools are released under a [GPLv3 license](LICENSE).

If you use any of the tools in an academic work, please cite:

    @inproceedings{stenborg2020using,
      title={Using Image Sequences for Long-Term Visual Localization},
      author={Stenborg, Erik and Sattler, Torsten and Hammarstrand, Lars},
      booktitle={2020 International Conference on 3D Vision (3DV)},
      pages={938--948},
      year={2020},
      organization={IEEE}
    }

# 2. Prerequisites
We have tested the library in **Ubuntu 16.04**, and **20.04**, but it should be easy to compile on other platforms. All dependencies below are inherited from ORB-SLAM2 so if you are able to run and compile ORB-SLAM2 you should be able to do the same for LocSLAM.

For your convenience, we provide a docker container config file that includes all required packages, see **Docker** below. We highly recommend that you build this docker container and use it to run and compile LocSLAM.   

## C++11 or C++0x Compiler
We use the new thread and chrono functionalities of C++11.

## Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Download and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

## OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **Required at least 2.4.3. Tested with OpenCV 2.4.11 and OpenCV 3.2**.

## Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

## DBoW2 and g2o (Included in Thirdparty folder)
We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. Both modified libraries (which are BSD) are included in the *Thirdparty* folder.

# 3. Building LocSLAM library and examples

Clone the repository:
```
git clone https://github.com/rulllars/SequentialVisualLocalization.git SeqVisLoc
```
We recommend that you clone the repository to a folder in your home directory in order to use the provided docker environment below.

## Docker
For you to get quickly up and running using LocSLAM we provide a docker configuration that contain all required dependencies. To build the docker container:
```
cd LocSLAM/Docker
chmod +x build_docker.sh
./build_docker.sh
```

Once the docker container is created, you can run the container using the provided script:
```
chmod +x run_docker.sh
./build_docker.sh
```
This will start the docker container as well as link in your home directory such that you can access the LocSLAM code.

## Building LocSLAM
We provide a script `build.sh` to build the *Thirdparty* libraries and *LocSLAM*. Assuming that you have installed all required dependencies, either directly (see section 2) or using the provided docker configuration (see **Docker** above). Make   
```
cd LocSLAM
chmod +x build.sh
./build.sh
```

This will create **libLocSLAM.so**  at *lib* folder and the executable **LocSLAM** in the *Examples* folder.

# 4. LocSLAM Examples

## Sequential CMU Seasons Dataset

Available soon.

<!--
1. Download the data from https://data.ciirc.cvut.cz/public/projects/2020VisualLocalization/Sequential-CMU-Seasons/ to *SeqVisLoc/Data*. Using wget you can run the following command line:

```
cd Data
wget -r -np -nH  --cut-dirs=3 https://data.ciirc.cvut.cz/public/projects/2020VisualLocalization/Sequential-CMU-Seasons/
```

2. Extract images from achieves using unzip
```
cd Sequential-CMU-Seasons
unzip data_collection_201*.zip
cd ../..
```

3. Execute the following command.
```
./Examples/locSLAM Vocabulary/ORBvoc.txt Examples/CMU.yaml
```

If you have downloaded the files to another folder then *Data* you need to make appropriate changes the */Examples/CMU.yaml* to point to the correct data directory.

# 8. Processing your own sequences
You will need to create a settings file similar to the provided *Exampels/CMU.yaml* with, e.g., data parameters, the calibration of your camera (we use the calibration model of OpenCV), etc. See the settings file provided.

For LocSLAM to work, the images need to be in sequence, have some images with matched local features to the fixed map and a odometry signal. See the examples to learn how to create a program that makes use of the LocSLAM library and how to pass images to the SLAM system. Compared to the original ORB-SLAM2, stereo and D-RGB cameras are not supported but could probably be supported by making suitable adjustments to the code using ORB-SLAM2 as a base.
 -->
