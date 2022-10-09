# HBST: Hamming Binary Search Tree | [Wiki](https://gitlab.com/srrg-software/srrg_hbst/wikis/home) | [Example code](https://gitlab.com/srrg-software/srrg_hbst_examples)
Lightweight and lightning-fast header-only library for binary descriptor-based VPR: `85 kB with 1'000 lines of C++ code`
  
[<img src="https://img.youtube.com/vi/N6RspfFdrOI/0.jpg" width="250">](https://www.youtube.com/watch?v=N6RspfFdrOI)
[<img src="https://img.youtube.com/vi/MwmzJygl8XE/0.jpg" width="250">](https://www.youtube.com/watch?v=MwmzJygl8XE)
[<img src="https://img.youtube.com/vi/f3h398t_zWo/0.jpg" width="250">](https://www.youtube.com/watch?v=f3h398t_zWo)

## [Example code repository](https://gitlab.com/srrg-software/srrg_hbst_examples) ([catkin](https://catkin-tools.readthedocs.io) ready!)
Due to the large number of provided example binaries and since we want 
to keep this repository as lightweight as possible, <br>
we created a separate repository for HBST example code. <br>
The example binaries include fully self-contained integrations with `Eigen`, `OpenCV`, `QGLViewer` and `ROS`.

## Build your own Descriptor/Node types!
The 2 base classes: `BinaryNode` and `BinaryMatchable` (see `srrg_hbst/types/`) can easily be inherited. <br>
Users might specify their own, augmented binary descriptor and node classes with specific leaf spawning. <br>
Two variants of subclassing are already provided in `srrg_hbst/types_probabilistic/`. <br>
Others are available in the [example code](https://gitlab.com/srrg-software/srrg_hbst_examples).

## Related publications
Please cite our most recent article when using the HBST library: <br>

    @article{2018-schlegel-hbst, 
      author  = {D. Schlegel and G. Grisetti}, 
      journal = {IEEE Robotics and Automation Letters}, 
      title   = {{HBST: A Hamming Distance Embedding Binary Search Tree for Feature-Based Visual Place Recognition}}, 
      year    = {2018}, 
      volume  = {3}, 
      number  = {4}, 
      pages   = {3741-3748}
    }

> RA-L 2018 'HBST: A Hamming Distance Embedding Binary Search Tree for Feature-Based Visual Place Recognition' <br>
> https://ieeexplore.ieee.org/document/8411466/ (DOI: 10.1109/LRA.2018.2856542)

Prior works:

    @inproceedings{2016-schlegel-hbst, 
      author    = {D. Schlegel and G. Grisetti}, 
      booktitle = {2016 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)}, 
      title     = {Visual localization and loop closing using decision trees and binary features}, 
      year      = {2016}, 
      pages     = {4616-4623}, 
    }

> IROS 2016 'Visual localization and loop closing using decision trees and binary features' <br>
> http://ieeexplore.ieee.org/document/7759679/ (DOI: 10.1109/IROS.2016.7759679)
