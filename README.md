# wind_nmhe_regression
ROS package for nonlinear moving horizon estimation (NMHE)-based regression of the wind disturbance. The resulting NMHE estimates the corresponding disturbance forces to improve the tracking accuracy of the controller (NMPC). The three disturbance forces are $F^{x_dist}$, $F^{y_dist}$, and $F^{z_dist}$ and they are computed in the body-frame of the aerial robot. 

This NMHE package is utilized in the following work. Please don't forget to consider citing it if you use these codes in your work.

**Plain text:**
```
M. Mehndiratta and E. Kayacan, "Gaussian Process-based Learning Control of Aerial Robots for Precise Visualization of Geological Outcrops," 2020 European Control Conference (ECC), Saint Petersburg, Russia, 2020, pp. 10-16.
```
**Bibtex:**
```
@INPROCEEDINGS{9143655,
  author={M. {Mehndiratta} and E. {Kayacan}},
  booktitle={2020 European Control Conference (ECC)}, 
  title={Gaussian Process-based Learning Control of Aerial Robots for Precise Visualization of Geological Outcrops}, 
  year={2020},
  volume={},
  number={},
  pages={10-16}
}
```
