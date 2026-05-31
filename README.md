---

# Demonstration

<p align="center">
  <img src="media/ukf_highway_tracked.gif" width="850">
</p>

The animation shows the UKF tracking multiple vehicles on a simulated highway using radar and lidar measurements.

---

# Highway Scenario

<p align="center">
  <img src="media/ukf_highway.png" width="850">
</p>

The simulation contains a straight three-lane highway with an ego vehicle and multiple traffic vehicles.

- Green vehicle → ego vehicle
- Blue vehicles → tracked traffic vehicles
- Red markers → lidar detections
- Purple lines → radar measurements
- RMSE values → live tracking accuracy

Each traffic vehicle has its own UKF instance, updated at every timestep.
