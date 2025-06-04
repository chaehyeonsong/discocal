# Welcome to DiscoCal! (CVPR24, ***highlight***) </br> Super Accurate, Fast, and Robust Calibration tool.


For decades, the checkerboard pattern has been the go-to method for camera calibration, providing only pixel-level precision. But what if we could improve accuracy even further? Discocal reveals the power of the circular pattern: a game-changer offering subpixel precision to meet challenges even from unconventional visual sensors.

**Visit the official [Document](https://chaehyeonsong.github.io/discocal/) for details!**


[[Paper]](https://arxiv.org/abs/2403.04583)[[Video]](http://www.youtube.com/watch?v=87_R7Qkpczo)[[OpenCV Webinar]](https://www.youtube.com/live/MTMMoN6ogcY?si=22DAdrzM3p9kDQK4)[[BibTex]](#Citation)

<img src="./figs/shorts1.gif">


Discocal supports:
- RGB camera calibration
- Thermal infrared camera calibration
- Extrinsic calibration of N cameras
- RGB-TIR extrinsic calibartion
- (Comming soon) LiDAR-Camera extrinsic calibration

## News
<!-- :round_pushpin: :Patch notes,    :tada:: awards -->
- 25.05.30. :round_pushpin: **The uncertainty-aware version is released.** It is more accurate and robost.
- 24.07.19. :tada: This work is invited to [OpenCV Webinar](https://www.youtube.com/live/MTMMoN6ogcY?si=22DAdrzM3p9kDQK4)
- 24.06.17. :round_pushpin:Add a description of how to undisort images using our method.
- 24.04.17. :round_pushpin:We update circular pattern detector! Now, you don't need to tune hyperparameters for detections
- 24.04.05. :tada: Discocal is selected for highlight poster. (11.9% of accepted papers, 2.8% of total submissions.)

<!-- --------------------
## Why DiscoCal? 

Sub-pixel accuracy and detection robustness are virtues of the conic features. But why do we use a checkerboard, not a circular pattern?

> :cry: Conic is ***not*** conic anymore under distortion!!

As shown below, the circle center is not projected to the centroid of the distorted ellipse under perspective transformation and distortion.

<img src="./docs/figs/overview.png" width="600" height="300">

Without considering geometery of the distorted ellipse, existing circular pattern-based calibration methods are biased, which leads low calibration accuracy than a checkerboard pattern.

> :pushpin: **Our unbiased estimator completes the missing piece in the conic-based calibration pipeline**

Supports:
* RGB camera calibration
* Thermal infrared camera calibration
* Extrinsic calibration of N cameras
* RGB-TIR extrinsic calibartion
* (Comming soon) LiDAR-Camera extrinsic calibration -->

----------------------
# How to use
## 1. Prepare runfile
**Option 1) Download runfile (Easy but only works on Ubuntu PC)**
* Ubuntu + x86_64 (amd64): 
	[[Download_link]](https://www.dropbox.com/scl/fo/m7ugu49aboonfk1o55spk/ADgaLJ8n3V_oks52XEz2Sts?rlkey=noidt7em84dtzfbbxl0j28wxk&st=nzrwslgt&dl=0)

* Ubuntu + Arm64: 
	[[Download_link]](https://www.dropbox.com/scl/fo/j0s4rr1bkzul7r8bptks6/ADmWkbheq0jjjIbGLno8gw4?rlkey=xaflec5h9591i0dct8akb3p9z&st=qnqwcoul&dl=0)

**Option 2) Build with docker (Supports all architectures)**
```bash
git clone https://github.com/chaehyeonsong/discocal.git
cd discocal
docker compose up --build
```
After build, runfiles will be created in discocal folder 

## 2. Run 
Note: Revise the config file before run
* Intrinsic calibration
	```bash
	chmod +x run_mono && ./run_mono [config_path]
	```
* Extrinsic calibration
	```bash
	chmod +x run_stereo && ./run_stereo [config_path]
	```

You can download sample images in [here](https://www.dropbox.com/scl/fo/mdy8xivja5wfwrjpculb3/ALXiShefmtTgfacgkOm7Zcw?rlkey=0ndgwesufd22f7i0mcfrtl8uo&st=s99ke8pt&dl=0)

<!-- # Applications

### Thermal Infrared Camera calibration

We can leverage the detection robustness of the circular patterns, particularly for unconventional cameras, such as thermal cameras. Watch the demo video!

[![Video Label](http://img.youtube.com/vi/j86pyBZe7t0/0.jpg)](https://youtu.be/j86pyBZe7t0) -->

# Citation

```
@INPROCEEDINGS{chsong-2024-cvpr,  
    author    = {Song, Chaehyeon and Shin, Jaeho and Jeon, Myung-Hwan and Lim, Jongwoo and Kim, Ayoung},
    title     = {Unbiased Estimator for Distorted Conics in Camera Calibration},
    booktitle = {IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR)},
    month     = {June},
    year      = {2024},
    pages     = {373-381}
}
```
# License
 <a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by-nc-sa/4.0/88x31.png" /></a><br />This work is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/4.0/">Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License</a>.

- This work is protected by a patent.
- All codes on this page are copyrighted by Seoul National University published under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 License. You must attribute the work in the manner specified by the author. You may not use the work for commercial purposes, and you may only distribute the resulting work under the same license if you alter, transform, or create the work.
- For commercial purposes, please contact to <a href="mailto:chaehyeon@snu.ac.kr">chaehyeon@snu.ac.kr</a>
