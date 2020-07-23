### c++

    cd TEASER-plusplus && mkdir build && cd build
    cmake .. && make
    sudo make install
    cd .. && cd examples/teaser_cpp_ply && mkdir build && cd build
    cmake .. && make
    OMP_NUM_THREADS=9 ./teaser_cpp_ply



implemented on two frames of mm-wave radar point cloud
<img src="https://github.com/Enzo-MiMan/TEASER-plusplus-1/blob/master/images/mm-wave.png" width="650" height="300"/>






--------------------

cite:
```bibtex
@article{Yang20arXiv-TEASER,
    title={TEASER: Fast and Certifiable Point Cloud Registration},
    author={Yang, Heng and Shi, Jingnan and Carlone, Luca},
    year={2020},
    eprint={2001.07715},
    archivePrefix={arXiv},
    primaryClass={cs.RO},
    url = {https://github.com/MIT-SPARK/TEASER-plusplus},
    pdf = {https://arxiv.org/abs/2001.07715}
}
```
