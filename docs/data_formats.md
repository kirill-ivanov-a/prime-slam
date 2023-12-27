# Data Formats
The data formats described below are currently supported.
### `icl`
```
/data
├── scene_0.depth — depth in ICL format
├── scene_0.png — image
├── scene_0.txt — camera parameters
├── scene_1.depth
├── scene_1.png
├── scene_1.txt
...
├── scene_N.depth
├── scene_N.png
└── scene_N.txt
```
File names may differ from those shown above. Additional information can be found [here](https://www.doc.ic.ac.uk/~ahanda/VaFRIC/iclnuim.html).
### `tum`
```
/data
├── /rgb — depth in ICL format
    ├── image_0.png
    ...
    └── image_N.png
├── /depth
    ├── depth_0.png
    ...
    └── depth_M.png
└── groundtruth.txt — gt poses in TUM format
```
Additional information can be found [here](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/file_formats).
### `icl_tum`
ICL data presented in `tum` format.
