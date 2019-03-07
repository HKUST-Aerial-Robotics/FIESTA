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


![](header.png)

## Installation

OS X & Linux:

```sh
npm install my-crazy-module --save
```

Windows:

```sh
edit autoexec.bat
```

## Usage example

A few motivating and useful examples of how your product can be used. Spice this up with code blocks and potentially more screenshots.

_For more examples and usage, please refer to the [Wiki][wiki]._

## Development setup

Describe how to install all development dependencies and how to run an automated test-suite of some kind. Potentially do this for multiple platforms.

```sh
make install
npm test
```

## Release History

* 0.2.1
    * CHANGE: Update docs (module code remains unchanged)
* 0.2.0
    * CHANGE: Remove `setDefaultXYZ()`
    * ADD: Add `init()`
* 0.1.1
    * FIX: Crash when calling `baz()` (Thanks @GenerousContributorName!)
* 0.1.0
    * The first proper release
    * CHANGE: Rename `foo()` to `bar()`
* 0.0.1
    * Work in progress

## Meta

Your Name – [@YourTwitter](https://twitter.com/dbader_org) – YourEmail@example.com

Distributed under the XYZ license. See ``LICENSE`` for more information.

[https://github.com/yourname/github-link](https://github.com/dbader/)

## Contributing

1. Fork it (<https://github.com/yourname/yourproject/fork>)
2. Create your feature branch (`git checkout -b feature/fooBar`)
3. Commit your changes (`git commit -am 'Add some fooBar'`)
4. Push to the branch (`git push origin feature/fooBar`)
5. Create a new Pull Request

<!-- Markdown link & img dfn's -->
[npm-image]: https://img.shields.io/npm/v/datadog-metrics.svg?style=flat-square
[npm-url]: https://npmjs.org/package/datadog-metrics
[npm-downloads]: https://img.shields.io/npm/dm/datadog-metrics.svg?style=flat-square
[travis-image]: https://img.shields.io/travis/dbader/node-datadog-metrics/master.svg?style=flat-square
[travis-url]: https://travis-ci.org/dbader/node-datadog-metrics
[wiki]: https://github.com/yourname/yourproject/wiki
