# Fiesta
> Incremental ESDF Map for motion planning

Euclidean Signed Distance Field (ESDF) is useful for online motion planning of aerial robots
since it can easily query the distance_ and gradient information against obstacles.
Fast incrementally built ESDF map is the bottleneck for conducting real-time motion planning.
In this paper, we investigate this problem and propose a mapping system called *Fiesta* to build
global ESDF map incrementally. By introducing two independent updating queues for inserting and
deleting obstacles separately, and using Indexing Data Structures and Doubly Linked Lists for
map maintenance, our algorithm updates as few as possible nodes using a BFS framework. Our ESDF
map has high computational performance and produces near-optimal results.
We show our method outperforms other up-to-date methods in term of performance and accuracy
by both theory and experiments. We integrate Fiesta into a completed quadrotor system and validate
it by both simulation and onboard experiments. We release our method as open-source software for the community. 

The paper of this method is submitted to the 2019 IEEE/RSJ International Conference on
Intelligent Robots and Systems (IROS 2019).  The draft is shown on arxiv
[here](https://arxiv.org/abs/1903.02144).

<p align="center">
<a href="http://www.youtube.com/watch?feature=player_embedded&v=pgRi8LOnT6Y
" target="_blank"><img src="figure/shortIntroduction.png"
alt="Fiesta short introduction video" width="720" height="540" /></a>
</p>

## Installation
### Required Library
- Eigen3
- PCL 1.7
- OpenCV
- ROS

Note: C++ 17 is required to run FIESTA. Please install and configure C++ 17.
```bash
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt-get update
sudo apt-get install gcc-7 g++-7
``` 

use the following command to check your gcc compilers

```
$ ls -lh /usr/bin/g++*
```

for **ubuntu 14.04**:

```
$ sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.8 60 --slave /usr/bin/g++ g++ /usr/bin/g++-4.8
```

for **ubuntu 16.04**:
```
$ sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 60 --slave /usr/bin/g++ g++ /usr/bin/g++-5
```

then

```
$ sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 70 --slave /usr/bin/g++ g++ /usr/bin/g++-7
```

use the folling command to change the default compiler (choose 0 as default)

```
$ sudo update-alternatives --config gcc
```

check your current gcc compiler

```
$ gcc --version
```



### Download & Compile
```sh
cd ~/catkin_ws/src
git clone https://github.com/hlx1996/Fiesta.git
cd ../
catkin_make
source ~/catkin_ws/devel/setup.bash
```



## Usage example

```sh
roslaunch fiesta cow_and_lady.launch
rosbag play data.bag
```

Cow and lady data set can be downloaded [here](http://robotics.ethz.ch/~asl-datasets/iros_2017_voxblox/data.bag).
A `rviz` will be opened with the visualization of occupancy grid map and a slice of esdf map.

_For more examples, usage and FAQ, please refer to the [Wiki][wiki].



## Release History
Prediction: 1.0.0 will be our first elegant version.
* 0.6.0
    * CHANGE: Refactor code according Google C++ Code Style
* 0.5.3
    * CHANGE: Improve the performance of the depth conversion process
* 0.5.2
    * ADD: Support local map
    * ADD: Support raw depth image as input, and depth filter of consistency
* 0.5.1
    * CHANGE: Improve performance of raycasting a lot
* 0.5.0
    * ADD: Give Local / global update / visualization options
    * ADD: Show Performance visualization directly on rviz
    * CHANGE: Deprecate older visualization, will be deleted in next release
* 0.4.1
    * CHANGE: Give better Visualization based on a slice of ESDF and pointcloud of obstacles
* 0.4.0
    * ADD: Support LIDAR Input
    * ADD: Support Deterministic Occupancy Grid Map
* 0.3.2
    * ADD: Implement Multi-thread Raycasting for array implementation
* 0.3.1
    * ADD: Implement Hash table with blocks
* 0.3.0
    * ADD: Implement Hash table   
* 0.2.0
    * ADD: Support Raycasting from point cloud_
    * ADD: Support Probabilistic Occupancy Grid Map Fusion
    * CHANGE: Patch code for limited observations
* 0.1.0
    * The first proper release, support ESDF for fully dynamic case and implemented in array
* 0.0.1
    * Support ESDF for insert-only case

## Meta

[hlx1996](https://github.com/hlx1996/) – hlx1996@example.com

Distributed under the MIT license. See ``LICENSE`` for more information.

## Contributing

1. Fork it (<https://github.com/hlx1996/Fiesta/fork>)
2. Create your feature branch (`git checkout -b feature/fooBar`)
3. Commit your changes (`git commit -am 'Add some fooBar'`)
4. Push to the branch (`git push origin feature/fooBar`)
5. Create a new Pull Request

<!-- Markdown link & img_ dfn's -->
[wiki]: https://github.com/hlx1996/Fiesta/wiki
