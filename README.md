# AKAMAI_Internship_Work

Collection of files worked on during 2022 AKAMAI internship term at Canada France Hawaii Telescope (CFHT)
taucamServ and taugrab work as a server and client respectively to capture images from a FLIR Tau 2 infrared camera and service them to the CFHT network. 
Likewise, zwocamServ and zwograb do the same for a ZWO ASI 178 mm astronomy all-sky visible-light camera. 
taucamLocal was an experiment early on in the project to become familiar with the framework.

The internship work was comprised of two stages: 1) ASIVA visible-light camera replacement and 2) development of the DualCam system.

1) The ASIVA visible-light camera had been down for nearly a decade. I was tasked with installing a replacement in the form of a commercially-available all-sky camera (ZWO ASI 178 mm). This involved designing the housing to mount the camera as well as a Raspberry Pi into the ASIVA as well as developing software written in C to interface with it and service images to the CFHT network. The installation also added two temperature sensors to the ASIVA which also service data to the CFHT network.

2) After my internship work was over (replacing the visible-light camera for ASIVA), I was offered the opportunity to continue working at CFHT to complete the development of the DualCam system which is comprised of both a visible-light and infrared camera controlled by a Raspberry Pi. This camera system was to be mounted within the CFHT dome facing outward into the slit such that it can take images of the night sky. This project involved designing and manufacturing the housing for the instrument, designing and manufacturing the mounting system, and developing software to service images to the CFHT network. The final version of the instrument also includes a combination temperature/humidity/pressure sensor which sends data to the CFHT network.

For additional information on the project, see Abstract-Taylor-Final.pdf, Final.pdf

See also:

- ASIVA images
https://www.cfht.hawaii.edu/~asiva/
