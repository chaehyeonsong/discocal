# Unbiased Estimator for Distorted Conics in Camera Calibration (CVPR24)
## Description
![overview](./Figs/overview.png)

As shown above, the circle center is not projected to the centroid of the distorted ellipse under perspective transformation and distortion. Without considering geometery of the distorted ellipse, existing circular pattern-based calibration methods are biased, which leads low calibration accuracy than a checkerboard pattern. 

**Our unbiased estimator completes the missing piece in the conic-based calibration pipeline and outperforms the checkerboard pattern-based calibration.**

## Camera model
We assume pin-hole camera model with radial distortion.
![overview](./Figs/camera_model.png)


# How to use
## Dependency
- [Ceres-Solver](http://ceres-solver.org/index.html)
- [Eigen3](https://eigen.tuxfamily.org/dox/index.html)
- opencv4

## docker 
	docker pull chaehyeonsong/discocal:latest

## build
	mkdir build
	cd build
	cmake ..
	make
	
	
## run
	./main.out [n_x] [n_y] [n_d] [img_dir_path] [r(m)] [distance(m)] [0:rgb, 1:thermal]


# Bibtex
	@INPROCEEDINGS { chsong-2024-cvpr,
		AUTHOR = { Chaehyeon Song and Jaeho Shin and Myung-Hwan Jeon and Jongwoo Lim and Ayoung Kim },
		TITLE = { Unbiased Estimator for Distorted Conic in Camera Calibration },
		BOOKTITLE = { IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR) },
		YEAR = { 2024 },
		MONTH = { June. },
		ADDRESS = { Seattle },
	}