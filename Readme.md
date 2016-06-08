### BB Blue APIs

This repository consists of easy­-to-­use APIs for the hardware on the ​Beaglebone Blue. These APIs will be rewritten from the original Strawson APIs which were written for the Strawson Robotics Cape which used the Cape's hardware by the userspace drivers and mmap. On the other hand, Beaglebone Blue APIs uses kernel-API approach to use the Beaglebone Blue hardware. Currently testing for the sensors with the kernel drivers is under progress.


##Testing using Beaglebone Black and Robotics Cape

Robotics Cape mainly uses the following sensors:

1. MPU-9150 9 axix IMU
2. BMP-180 Pressure sensor
3. TI-eQEP Rotatory encoders

More info and detailed approach on how these sensors can be installed on the 4.4 mainline is given in the wiki page.
