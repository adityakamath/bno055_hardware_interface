---
layout: page
title: BNO055 Hardware Interface
subtitle: ros2_control SensorInterface for the Bosch BNO055 9-DOF IMU
---

<style>
  .feature-box {
    transition: all 0.2s ease;
  }

  .feature-box:hover {
    transform: translateY(-2px);
    box-shadow: 0 4px 12px rgba(0,0,0,0.2) !important;
  }
</style>

<div style="width: 100vw; margin-left: calc(-50vw + 50%); padding-left: 5%; padding-right: 5%; box-sizing: border-box;" markdown="1">

![Project Status](https://img.shields.io/badge/Status-Active-green)
![ROS 2](https://img.shields.io/badge/ROS%202-Kilted%20(Ubuntu%2024.04)-blue?style=flat&logo=ros&logoSize=auto)
![ROS 2 Control](https://img.shields.io/badge/ros2__control-SensorInterface-blue?style=flat&logo=ros&logoSize=auto)
![Repository](https://img.shields.io/badge/Repo-adityakamath%2Fbno055__hardware__interface-purple?style=flat&logo=github&logoSize=auto)
![Dependency](https://img.shields.io/badge/Dep-BoschSensortec%2FBNO055__SensorAPI-purple?style=flat&logo=github&logoSize=auto)
[![Ask DeepWiki (Experimental)](https://deepwiki.com/badge.svg)](https://deepwiki.com/adityakamath/bno055_hardware_interface)
![C++](https://img.shields.io/badge/C++-17-blue?style=flat&logo=cplusplus&logoColor=white)
![License](https://img.shields.io/github/license/adityakamath/bno055_hardware_interface?label=License)

</div>

<div style="width: 100vw; margin-left: calc(-50vw + 50%); padding-left: 5%; padding-right: 5%; box-sizing: border-box;" markdown="1">

> `ros2_control` `SensorInterface` plugin for the Bosch BNO055 9-DOF IMU over I2C (Linux `i2c-dev`).
>
> **⚠️ Status:** Tested and validated on Raspberry Pi 5 running ROS 2 Kilted (Ubuntu 24.04, aarch64) with real BNO055 hardware.

</div>

<div style="width: 100vw; margin-left: calc(-50vw + 50%); padding-left: 5%; padding-right: 5%; box-sizing: border-box;">
<div style="display: flex; flex-wrap: wrap; gap: 0.6em; margin: 2em 0; align-items: stretch;">
  <div class="feature-box" style="flex: 1 1 calc(33.333% - 0.4em); min-width: 280px; display: flex; align-items: flex-start; gap: 0.8em; background: #ffffff; padding: 0.7em 0.9em; border-radius: 8px; box-shadow: 0 4px 12px rgba(0,0,0,0.2);">
    <span style="font-size: 1.8em;">🧭</span>
    <div style="flex: 1;">
      <strong>NDOF Sensor Fusion</strong>
      <br/><br/>
      <span style="font-size: 0.95em;">
        Absolute orientation from on-chip gyroscope + accelerometer + magnetometer fusion — no quaternion integration drift.
      </span>
    </div>
  </div>
  <div class="feature-box" style="flex: 1 1 calc(33.333% - 0.4em); min-width: 280px; display: flex; align-items: flex-start; gap: 0.8em; background: #ffffff; padding: 0.7em 0.9em; border-radius: 8px; box-shadow: 0 4px 12px rgba(0,0,0,0.2);">
    <span style="font-size: 1.8em;">📊</span>
    <div style="flex: 1;">
      <strong>10 State Interfaces</strong>
      <br/><br/>
      <span style="font-size: 0.95em;">Orientation quaternion (x, y, z, w), angular velocity (rad/s), and linear acceleration (m/s²) — fully compatible with imu_sensor_broadcaster.</span>
    </div>
  </div>
  <div class="feature-box" style="flex: 1 1 calc(33.333% - 0.4em); min-width: 280px; display: flex; align-items: flex-start; gap: 0.8em; background: #ffffff; padding: 0.7em 0.9em; border-radius: 8px; box-shadow: 0 4px 12px rgba(0,0,0,0.2);">
    <span style="font-size: 1.8em;">🔀</span>
    <div style="flex: 1;">
      <strong>Axis Remapping</strong>
      <br/><br/>
      <span style="font-size: 0.95em;">8 standard mounting orientations (P0–P7) configurable at launch, matching BNO055 datasheet §3.4.</span>
    </div>
  </div>
  <div class="feature-box" style="flex: 1 1 calc(33.333% - 0.4em); min-width: 280px; display: flex; align-items: flex-start; gap: 0.8em; background: #ffffff; padding: 0.7em 0.9em; border-radius: 8px; box-shadow: 0 4px 12px rgba(0,0,0,0.2);">
    <span style="font-size: 1.8em;">💾</span>
    <div style="flex: 1;">
      <strong>Calibration Persistence</strong>
      <br/><br/>
      <span style="font-size: 0.95em;">Save sensor calibration offsets to YAML and load them automatically at boot. Offsets are written in CONFIG mode before entering NDOF.</span>
    </div>
  </div>
  <div class="feature-box" style="flex: 1 1 calc(33.333% - 0.4em); min-width: 280px; display: flex; align-items: flex-start; gap: 0.8em; background: #ffffff; padding: 0.7em 0.9em; border-radius: 8px; box-shadow: 0 4px 12px rgba(0,0,0,0.2);">
    <span style="font-size: 1.8em;">🧪</span>
    <div style="flex: 1;">
      <strong>Mock Mode</strong>
      <br/><br/>
      <span style="font-size: 0.95em;">Run the complete ros2_control lifecycle and publish zero/identity values without any hardware.</span>
    </div>
  </div>
  <div class="feature-box" style="flex: 1 1 calc(33.333% - 0.4em); min-width: 280px; display: flex; align-items: flex-start; gap: 0.8em; background: #ffffff; padding: 0.7em 0.9em; border-radius: 8px; box-shadow: 0 4px 12px rgba(0,0,0,0.2);">
    <span style="font-size: 1.8em;">🔄</span>
    <div style="flex: 1;">
      <strong>TF Broadcasting</strong>
      <br/><br/>
      <span style="font-size: 0.95em;">Optional imu_tf_broadcaster relay node republishes IMU orientation as a dynamic TF transform for RViz / Foxglove.</span>
    </div>
  </div>
</div>
</div>

<div style="width: 100vw; margin-left: calc(-50vw + 50%); padding-left: 5%; padding-right: 5%; box-sizing: border-box;">

<style>
  .si-table {
    transition: all 0.2s ease;
  }

  .si-table:hover {
    transform: translateY(-2px);
    box-shadow: 0 6px 16px rgba(0,0,0,0.25) !important;
  }
</style>

<table class="si-table" style="width: 100%; border-collapse: separate; border-spacing: 0; margin: 2em auto; border-radius: 8px; overflow: hidden; box-shadow: 0 4px 12px rgba(0,0,0,0.2); border: none;">
  <thead>
    <tr>
      <th colspan="3" style="text-align: center; padding: 0.6em; background: #f8f9fa; border: none;">🧩 State Interfaces</th>
    </tr>
    <tr>
      <th style="text-align: left; padding: 0.6em; background: #e9ecef; border: none;">Interface</th>
      <th style="text-align: left; padding: 0.6em; background: #e9ecef; border: none;">Unit</th>
      <th style="text-align: left; padding: 0.6em; background: #e9ecef; border: none;">Description</th>
    </tr>
  </thead>
  <tbody>
    <tr style="background: #ffffff;">
      <td style="padding: 0.6em; border: none;"><code>orientation.x</code></td>
      <td style="padding: 0.6em; border: none;">–</td>
      <td style="padding: 0.6em; border: none;">Quaternion X (raw ÷ 16384, datasheet §3.6.5.5)</td>
    </tr>
    <tr style="background: #f0f0f0;">
      <td style="padding: 0.6em; border: none;"><code>orientation.y</code></td>
      <td style="padding: 0.6em; border: none;">–</td>
      <td style="padding: 0.6em; border: none;">Quaternion Y</td>
    </tr>
    <tr style="background: #ffffff;">
      <td style="padding: 0.6em; border: none;"><code>orientation.z</code></td>
      <td style="padding: 0.6em; border: none;">–</td>
      <td style="padding: 0.6em; border: none;">Quaternion Z</td>
    </tr>
    <tr style="background: #f0f0f0;">
      <td style="padding: 0.6em; border: none;"><code>orientation.w</code></td>
      <td style="padding: 0.6em; border: none;">–</td>
      <td style="padding: 0.6em; border: none;">Quaternion W (scalar)</td>
    </tr>
    <tr style="background: #ffffff;">
      <td style="padding: 0.6em; border: none;"><code>angular_velocity.x</code></td>
      <td style="padding: 0.6em; border: none;">rad/s</td>
      <td style="padding: 0.6em; border: none;">Gyroscope X — unit set to RPS</td>
    </tr>
    <tr style="background: #f0f0f0;">
      <td style="padding: 0.6em; border: none;"><code>angular_velocity.y</code></td>
      <td style="padding: 0.6em; border: none;">rad/s</td>
      <td style="padding: 0.6em; border: none;">Gyroscope Y</td>
    </tr>
    <tr style="background: #ffffff;">
      <td style="padding: 0.6em; border: none;"><code>angular_velocity.z</code></td>
      <td style="padding: 0.6em; border: none;">rad/s</td>
      <td style="padding: 0.6em; border: none;">Gyroscope Z</td>
    </tr>
    <tr style="background: #f0f0f0;">
      <td style="padding: 0.6em; border: none;"><code>linear_acceleration.x</code></td>
      <td style="padding: 0.6em; border: none;">m/s²</td>
      <td style="padding: 0.6em; border: none;">Accelerometer X — unit set to m/s²</td>
    </tr>
    <tr style="background: #ffffff;">
      <td style="padding: 0.6em; border: none;"><code>linear_acceleration.y</code></td>
      <td style="padding: 0.6em; border: none;">m/s²</td>
      <td style="padding: 0.6em; border: none;">Accelerometer Y</td>
    </tr>
    <tr style="background: #f0f0f0;">
      <td style="padding: 0.6em; border: none;"><code>linear_acceleration.z</code></td>
      <td style="padding: 0.6em; border: none;">m/s²</td>
      <td style="padding: 0.6em; border: none;">Accelerometer Z</td>
    </tr>
  </tbody>
</table>

</div>
