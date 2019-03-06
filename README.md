# FIESTA
> Incremental ESDF Map for motion planning

Euclidean Signed Distance Field (ESDF) is useful for online motion planning of aerial robots
since it can easily query the distance and gradient information against obstacles.
Fast incrementally built ESDF map is the bottleneck for conducting real-time motion planning.
In this paper, we investigate this problem and propose a mapping system called *FIESTA* to build
global ESDF map incrementally. By introducing two independent updating queues for inserting and
deleting obstacles separately, and using Indexing Data Structures and Doubly Linked Lists for
map maintenance, our algorithm updates as few as possible nodes using a BFS framework. Our ESDF
map has high computational performance and produces near-optimal results.
We show our method outperforms other up-to-date methods in term of performance and accuracy
by both theory and experiments. We integrate FIESTA into a completed quadrotor system and validate
it by both simulation and onboard experiments. We release our method as open-source software for the community. 

The paper of this method is submitted to the 2019 IEEE/RSJ International Conference on
Intelligent Robots and Systems (IROS 2019), under review.  The draft is shown on arxiv
[here](https://arxiv.org/submit/2599481/view).


<a href="http://www.youtube.com/watch?feature=player_embedded&v=pgRi8LOnT6Y
" target="_blank"><img src="https://i9.ytimg.com/vi/pgRi8LOnT6Y/maxresdefault.jpg?sqp=CJD5_-MF&rs=AOn4CLBjiSiAZX9OoNn-iYOXa9KTqWq_fg&time=1551891870311" 
alt="FIESTA short introduction video" width="240" height="180" border="10" /></a>

## Installation

```sh
cd ~/catkin_ws/src
git clone https://github.com/hlx1996/FIESTA.git
cd ../
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## Usage example

```sh
roslaunch FIESTA demo.launch
```

A few motivating and useful examples of how your product can be used. Spice this up with code blocks and potentially more screenshots.

_For more examples and usage, please refer to the [Wiki][wiki]._



## Release History
* 0.5.0
    * ADD: Local / global update / visulization options
    * ADD: Performance visulization directly on rviz
    * DELETE: older visualization
* 0.4.1
    * CHANGE: Give better visualization based on a slice of ESDF and pointcloud of obstacles
* 0.4.0
    * ADD: Support LIDAR Input
    * ADD: Support Deterministic Occupancy Grid Map
* 0.3.2
* 0.3.1
    * ADD: Implement Hash table with blocks
* 0.3.0
    * ADD: Implement Hash table   
* 0.2.0
    * ADD: Support Raycasting for pointclouds
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

1. Fork it (<https://github.com/hlx1996/FIESTA/fork>)
2. Create your feature branch (`git checkout -b feature/fooBar`)
3. Commit your changes (`git commit -am 'Add some fooBar'`)
4. Push to the branch (`git push origin feature/fooBar`)
5. Create a new Pull Request

<!-- Markdown link & img dfn's -->
[wiki]: https://github.com/hlx1996/FIESTA/wiki
