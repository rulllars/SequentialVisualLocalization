#FROM        ubuntu:18.04
FROM         nvidia/opengl:1.0-glvnd-runtime-ubuntu18.04
MAINTAINER  Lars Hammarstrand

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Stockholm
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES},display

#install libs first
RUN apt-get update && apt-get install -y apt-utils
RUN apt-get install -y gnupg2
#RUN apt-key update && apt-get update
RUN apt-get install -y build-essential gdb perl python git wget python-pip libpython2.7-dev
RUN python -mpip install numpy pyopengl Pillow pybind11
RUN apt-get install -y pkg-config

#RUN apt-get install -y libgl1-mesa-dev
RUN apt-get install -y cmake
RUN apt-get install -y libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev libv4l-dev
RUN apt-get install -y libavcodec-dev libavformat-dev libavutil-dev libpostproc-dev libswscale-dev libavdevice-dev libsdl-dev
RUN apt-get install -y libgtk2.0-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
RUN apt-get install -y libeigen3-dev nano
RUN apt-get install -y libblas-dev liblapack-dev
#RUN apt-get install -y libgl1-mesa-dev

#install opencv
RUN apt-get install -y libopencv-dev
#RUN cd && mkdir opencv && cd opencv && \
#  wget ftp://ftp.netbsd.org/pub/pkgsrc/distfiles/opencv-2.4.6.1.tar.gz && \
#  tar zxvf opencv-2.4.6.1.tar.gz && cd opencv-2.4.6.1 && mkdir release && \
#  cd release && cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local .. && \
#  make && make install
#RUN sudo echo "include /usr/local/lib" >> /etc/ld.so.conf
#RUN sudo ldconfig

#install glew
RUN apt-get install -y libglew-dev
#RUN cd && git clone https://github.com/nigels-com/glew.git glew && cd glew && make && make install

#Install libxkbcommon
RUN apt-get install -y libxkbcommon-dev
RUN apt-get install -y libwayland-dev wayland-protocols

#install pangolin
RUN cd /usr/local/src && git clone https://github.com/stevenlovegrove/Pangolin.git && \
  cd Pangolin && mkdir build && cd build && cmake -DCPP11_NO_BOOST=1 .. && make -j && make install

#test X11
RUN apt-get install -y x11-apps mesa-utils

RUN apt install -y libcanberra-gtk-module libcanberra-gtk3-module

RUN apt-get update && apt install -y valgrind
