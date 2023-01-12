# Sequential Localization
**Authors:** Erik Stenborg, Torsten Sattler and [Lars Hammarstrand](https://www.chalmers.se/en/staff/Pages/lars-hammarstrand.aspx).

This repository consist of a set of tools to perform sequential visual localization of a moving camera platform (multi-camera rig) using odometry and a fixed map. The tool set includes 2 set of tools. The first is [ĹocSLAM](LocSLAM/) which is a hybrid SLAM system based on [ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2) that computes a precise platform (body) trajectory in relation to a pre-built fixed map using images from a set of cameras, odometry, and fixed map matches (local features). The second is a set of [Matlab scripts](CoarseLocFilters/) to perform coarse localization using odometry and coarse positioning information from either GPS or image retrieval, e.g. DenseVLAD. Please view the specific sub-repository for more details regarding each tool.

[![](https://img.youtube.com/vi/W4gooWGs7Q0/0.jpg)](https://www.youtube.com/watch?v=W4gooWGs7Q0)


### Related Publications:

Erik Stenborg, Torsten Sattler and Lars Hammarstrand. **Using Image Sequences for Long-Term Visual Localization**, *2020 International Conference on 3D Vision (3DV)*, 2020, pp. 938-948, doi: 10.1109/3DV50981.2020.00104. **[PDF](https://research.chalmers.se/publication/522731/file/522731_Fulltext.pdf)**

# 1. License

Our tools are released under a [GPLv3 license](LICENSE).

If you use any of the tools in an academic work, please cite:

    @inproceedings{stenborg2020using,
      title={Using Image Sequences for Long-Term Visual Localization},
      author={Stenborg, Erik and Sattler, Torsten and Hammarstrand, Lars},
      booktitle={2020 International Conference on 3D Vision (3DV)},
      pages={938--948},
      year={2020},
      organization={IEEE}
    }

# 2. Prerequisites
We have tested the library in **Ubuntu 16.04**, and **20.04**, but it should be easy to compile/run on other platforms.
