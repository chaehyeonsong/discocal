---
sort: 2
---

# How To Run

## 1. Prepare runfiles
**Option 1) Download runfile (Easy but only works on Ubuntu PC)**
* Ubuntu + x86_64 (amd64): 
	[[Download_link]](https://www.dropbox.com/scl/fo/m7ugu49aboonfk1o55spk/ADgaLJ8n3V_oks52XEz2Sts?rlkey=noidt7em84dtzfbbxl0j28wxk&st=nzrwslgt&dl=0)

* Ubuntu + Arm64: 
	[[Download_link]](https://www.dropbox.com/scl/fo/j0s4rr1bkzul7r8bptks6/ADmWkbheq0jjjIbGLno8gw4?rlkey=xaflec5h9591i0dct8akb3p9z&st=qnqwcoul&dl=0)

**Option 2) Build with docker (Supports all architectures)**
```bash
$ git clone https://github.com/chaehyeonsong/discocal.git
$ cd discocal
$ docker compose up
```
After build, runfiles will be created in discocal folder 

```tip
If docker compose command is not installed or doesn't work, try this

`$ docker run --rm --entrypoint sh chaehyeonsong/discocal -c "tar -C /app/dist -cf - ." | tar -xf -`
```



## 2. Revise the config files
This is template of config files

```
camera:
  img_dir : "write your image path" 
  n_d: 2 # number of distortion coefficients
  type: "circle"
  n_x: 4 # Number of circles in a row
  n_y: 3 # Number of circles in a column
  detection_mode: ""
  radius: 0.03 # circle radius
  distance: 0.09 # distance between circle centers

options:
  visualize : true # If true, you can check the detection results in real-time
  evaluation: true # If true, you can check the uncertainty map indicating calibration accuracy
  save_pose : false # If true, estimated target poses are saved
  save_rpe : false # If true, reprojection error of each measurement is saved
```


## 3. Run 
* Intrinsic calibration
```bash
$ sudo chmod +x run_mono && ./run_mono [config_path]
```
* Extrinsic calibration
```bash
$ sudo chmod +x run_stereo && ./run_stereo [config_path]
```

You can download sample images in [here](https://www.dropbox.com/scl/fo/mdy8xivja5wfwrjpculb3/ALXiShefmtTgfacgkOm7Zcw?rlkey=0ndgwesufd22f7i0mcfrtl8uo&st=s99ke8pt&dl=0)