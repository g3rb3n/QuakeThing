# QuakeThing
A first attempt at detecting earth quakes with a MPU9250.

It won't be as good as professional seismometers but the MPU9250 should be able to measure light shocks that generate a few milli G's.

It currently samples at 1008 Hz without low pass filtering and is able to detect light vibrations like closing a door on the other side of the house or typing on the keyboard on the other side of the table. For filtering out noise a network of sensors is used.

# ToDo #
- Figure out what are the best hardware settings for the MPU9250 to detect earth quakes.
- Write code to detect low frequency waves. Low pass filter? Discrete Fourier transform?
- MQTT is designed for small messages, It would be nice to have the full wave signal for further central analysis.
