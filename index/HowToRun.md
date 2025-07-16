---
sort: 3
---

# How To Run

## 0. Preprocess
Before start, take images for calbibration using a planar target with the circular pattern. See the [Guidance](./Guidance.md) section.

## 1. Prepare runfiles
### Option 1) Download runfile (Easy but only works on Ubuntu PC)
* Ubuntu + x86_64 (amd64): 
	[[Download_link]](https://www.dropbox.com/scl/fo/m7ugu49aboonfk1o55spk/ADgaLJ8n3V_oks52XEz2Sts?rlkey=noidt7em84dtzfbbxl0j28wxk&st=nzrwslgt&dl=0)

* Ubuntu + Arm64: 
	[[Download_link]](https://www.dropbox.com/scl/fo/j0s4rr1bkzul7r8bptks6/ADmWkbheq0jjjIbGLno8gw4?rlkey=xaflec5h9591i0dct8akb3p9z&st=qnqwcoul&dl=0)

### Option 2) Build with docker (Supports all architectures)
```bash
$ git clone https://github.com/chaehyeonsong/discocal.git
$ cd discocal
$ docker compose up
```
After build, runfiles will be created in discocal folder 

```tip
If docker compose command is not installed or doesn't work, try this

`$ docker build -t chaehyeonsong/discocal . -f dockerfile`  
`$ docker run --rm --entrypoint sh chaehyeonsong/discocal -c "tar -C /app/dist -cf - ." | tar -xf -`
```



## 2. Modify the config file
- [`intrinsic.yaml`](../codes/intrinsic.yaml): This is a config template of **instrinsic calibration**


- [`extrinsic.yaml`](../codes/extrinsic.yaml): This is a config template of **extrinsic calibration**.  
With DiscoCal, extrinsic calibration for **n** cameras can be performed simultaneously.  
The relative poses of all cameras are estimated with respect to the first camera.

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

<!-- ## 4. Results -->

## Demo video

[![Video Label](http://img.youtube.com/vi/75tsl_WARz4/0.jpg)](https://youtu.be/75tsl_WARz4)