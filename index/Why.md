---
sort: 1
---

# Why DiscoCal?

For decades, the checkerboard pattern has been regarded as the standard method for camera calibration, offering only **pixel-level precision**.  
But what if we could push the limits of accuracy even further?  
**DiscoCal** reveals the power of the **circular pattern** — a game-changer that enables **subpixel precision**, even for challenging sensors such as thermal or fisheye cameras.


## Advantages of the Circular Pattern

The checkerboard pattern uses the corners of squares as control points.  
However, these corners are only accurate to the **pixel level**, as shown below:

<img src="../figs/control_points.png" height="300">

In contrast, the circular pattern uses the **centroids of circular shapes**, which can be detected with **subpixel accuracy**.


## Unbiased Projection Model

Subpixel accuracy and detection robustness are key strengths of [conics](https://en.wikipedia.org/wiki/Conic_section) features.  
So why has the checkerboard remained dominant over the circular pattern?

> 😢 **Conics are no longer true conics under lens distortion!**

As shown below, due to perspective and lens distortion, the **projected center of a circle** no longer coincides with the **centroid of the resulting ellipse**.

<img src="../figs/overview.png" height="300">

Most calibration methods using circualr patterns **ignore the geometry of the distorted ellipse**, leading to **bias** and lower calibration accuracy than checkerboard methods.

> 📌 **Our unbiased estimator fills the missing piece in conic-based calibration.**

[This paper](https://arxiv.org/abs/2403.04583) introduces an **unbiased projection model** for circular patterns using a probabilistic concept called [moments](https://en.wikipedia.org/wiki/Moment_(mathematics)).  
Although there is no simple analytic form for distorted ellipses, **moment representations always exist**.

<img src="../figs/moments_transformation.png" height="300">