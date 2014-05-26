controlPTZ
==========


This software implements the calibration and the auto-calibration of two AXIS cameras PTZ.


Setup
-----

To compile this package, it is recommended to create a separate build
directory:

    mkdir _build
    cd _build
    cmake [OPTIONS] ..
    make install

Please note that CMake produces a `CMakeCache.txt` file which should
be deleted to reconfigure a package from scratch.


### Dependencies

The calibration depends on several packages which
have to be available on your machine.

 - Libraries:
   - opencv (>= 2.4.2)
   - curl
   - lapack
   - Eigen (>= 3.2.0)
   - [controlPTZ][controlPTZ] (>=1.0)
     The calibration uses the handling of both cameras.
   - levmar
 - System tools:
   - CMake (>= 2.6)
   - pkg-config
   - usual compilation tools (GCC/G++, make, etc.)

[controlPTZ]: https://github.com/elie-moussy/controlPTZ.git