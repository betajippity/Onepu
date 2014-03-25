Onepu
=====

About
-----

A basic implementation of Featherstone's Articulated Bodies Algorithm for simulating large articulated kinematic chains in O(n) time, as described in Roy Featherstone's book [Robot Dynamics Algorithms](http://books.google.com/books/about/Robot_Dynamics_Algorithms.html?id=c6yz7f_jpqsC). Also includes an implementation of the 6-dimensional spatial math system described in Featherstone's book, built on top of Eigen.

Written as a hacky, fast implementation. Currently works with extremely large single chains and supports basic gravity; algorithms for multi-DOF and prismatic joints are also implemented, but currently are not exposed to the core simulation loop. That's something I might work on later.

Dependencies
------------

* Eigen 3.2.x
* GLFW 3.0.x
* GLM 0.9.x
* OpenMP 2.0 

All external libraries/code can be found bundled together in my [Nuparu](https://github.com/betajippity/Nuparu) repository. OpenMP means that Onepu must be compiled with a compiler supporting OpenMP, such as GCC 4.x, MSVC 10.0 or higher, or the experimental OpenMP/Clang.

Building
--------

Onepu can be built via [CMake](http://www.cmake.org/) (and has been tested) on OSX, Windows, or Fedora. Latest support Nuparu version is v0.1.14.09. 

