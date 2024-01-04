# docker 
docker pull chaehyeonsong/discocal:latest

# build
	mkdir build
	cd build
	cmake ..
	make
	
	
# run
	./main.out [n_x] [n_y] [n_d] [img_dir_path] [r(m)] [distance(m)] [0:rgb, 1:thermal]
