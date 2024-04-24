FROM ubuntu:20.04

MAINTAINER chaehyeon.song

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
&& apt-get autoclean

RUN mkdir /temp && cd /temp && wget http://ceres-solver.org/ceres-solver-2.2.0.tar.gz && tar zxf ceres-solver-2.2.0.tar.gz
RUN mkdir /temp/ceres-bin && cd /temp/ceres-bin && cmake ../ceres-solver-2.2.0 && make -j4 && make test && make install
