# Particle Filter Localization

To run the particle filter with visualization:

* `ros2 launch pf_localization pf_foxglove.launch.py serial_port:=/dev/ttyUSB0`
* Open [Foxglove](https://app.foxglove.dev/dashboard) in a browser
* Find the rover's IP address with `tailscale status`
* Open a connection with the WebSocket ws://[rover ip address]:8765
* Click the `/particle_filter/visualization` topic

To run the particle filter without visualization

* `ros2 launch pf_localization/launch/pf_headless.launch.py serial_port:=/dev/ttyUSB0`