# Build My World

This is the first project from Robotics Nanodegree at Udacity.
The project consists of building a simulation environment or world in Gazebo for all the upcoming projects.
In order to pass this project one must complete the [rubric points][Rubric Points].

![My Office]

## Directory structure

```
.build-my-world                    # Build My World Project 
├── docs
├── images
├── model                          # Model files 
│   ├── floor-blueprint
│   |    ├── model.config
|   |    └── model.sdf
│   |
|   ├── kart 
│   |    ├── model.config
|   |    └── model.sdf
│   |
|   └── robot
│        ├── model.config
|        └── model.sdf      
├── script                         # Gazebo World plugin C++ script      
│   └── welcome.cpp
├── world                          # Gazebo main World containing models 
│   └── my-office.world
└── CMakeLists.txt                 # Link libraries 
```

## Prerequisites

* `gazebo => 7.16.0`
* `cmake >= 3.5.1`
  * For all OSes click [here][CMake] for installation instructions
* `make >= 4.1.0`
  * For Linux `make` is installed by default on most distros.
  * For macOS install [Xcode] command line tools to get make.
  * For Windows click [here][Make for Windows] for installation instructions.
* `gcc/g++ >= 5.4`
  * For Linux `gcc` and `g++` are installed by default on most distros.
  * For macOS same deal as `make`, install [Xcode] command line tools.
  * For Windows is recommended using [MinGW].

## Code Style

This project follows [Google's C++ Style Guide].

[My Office]: images/my-office.png
[CMake]: https://cmake.org/install
[Xcode]: https://developer.apple.com/xcode/features
[Make for Windows]: http://gnuwin32.sourceforge.net/packages/make.htm
[MinGW]: http://www.mingw.org
[Google's C++ Style Guide]: https://google.github.io/styleguide/cppguide.html
[Rubric Points]: docs/RubricPoints.md
[.clang-format]: .clang-format
