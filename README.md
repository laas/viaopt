VIA OPT
===========

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

 - Libraries:
   - eigen3
 - System tools:
   - CMake (>=2.6)
   - pkg-config
   - usual compilation tools (GCC/G++, make, etc.)


Copyright
---------
Open source software written by LAAS-CNRS. LGPL licence. See COPYING file for
details.