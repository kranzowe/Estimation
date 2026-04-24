# Particle Filter Localization

To run the particle filter with visualization:

* `ros2 launch pf_localization/launch/pf_foxglove.launch.py`
* Open [Foxglove](https://app.foxglove.dev/dashboard) in a browser
* Open a connection with the WebSocket ws://localhost:8765
* Click the `/particle_filter/visualization` topic

To run the particle filter without visualization

* `ros2 launch pf_localization/launch/pf_headless.launch.py`