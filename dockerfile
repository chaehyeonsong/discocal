FROM ubuntu:20.04

LABEL name="chaehyeon.song"
LABEL email="chaehyeon@snu.ac.kr"
LABEL version="2.0"

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get -y update \
&& apt-get -y install \
	build-essential\ 
	cmake\ 
	git\
	wget\
	python3\
	python3-pip \
	gdb\
	libgoogle-glog-dev\
	libgflags-dev\
	libatlas-base-dev\
	libeigen3-dev\
	libsuitesparse-dev\
	libopencv-dev\
	python3-opencv\
	locales\
&& apt-get autoclean

RUN locale-gen en_US.UTF-8

##install Ceres
RUN mkdir /temp && cd /temp && wget http://ceres-solver.org/ceres-solver-2.2.0.tar.gz && tar zxf ceres-solver-2.2.0.tar.gz
RUN mkdir /temp/ceres-bin && cd /temp/ceres-bin && cmake ../ceres-solver-2.2.0 && make -j4 && make test && make install

##install yaml-cpp
RUN cd /temp && git clone https://github.com/jbeder/yaml-cpp.git && cd yaml-cpp/ && mkdir build && cd build && cmake -DCMAKE_INSTALL_PREFIX=/usr/local ../ && make && make install

