# Rubric points

This document gathers information about all the rubric points met for this project.

## Data Buffer

| Point                          | Files                              | Lines          |
|--------------------------------|------------------------------------|----------------|
| MP.1 Data Buffer Optimization  | [ring_buffer.h], [ring_buffer.cpp] | -              |

## Keypoints

| Point                   | Files               | Lines        |
|-------------------------|---------------------|--------------|
| MP.2 Keypoint Detection | [matching_2d.cpp]   | 67, 101, 152 |
| MP.3 Keypoint Removal   | [camera_fusion.cpp] | 69           |

## Descriptors

| Point                          | Files             | Lines        |
|--------------------------------|-------------------|--------------|
| MP.4 Keypoint Descriptors      | [matching_2d.cpp] | 43           |
| MP.5 Descriptor Matching       | [matching_2d.cpp] | 7            |
| MP.6 Descriptor Distance Ratio | [matching_2d.cpp] | 33           |     

## Performance

### MP.7 Performance Evaluation 1

- **Harris**

| Image            | Number of keypoints | Distribution |
|------------------|---------------------|--------------|
| `0000000000.png` | 53                  | Fair         |
| `0000000001.png` | 63                  | Fair         |
| `0000000002.png` | 64                  | Fair         |
| `0000000003.png` | 73                  | Fair         |
| `0000000004.png` | 87                  | Fair         |
| `0000000005.png` | 76                  | Fair         |
| `0000000006.png` | 69                  | Fair         |
| `0000000007.png` | 111                 | Fair         |
| `0000000008.png` | 84                  | Fair         |
| `0000000009.png` | 68                  | Fair         |

- **Shi Tomasi**

| Image            | Number of keypoints | Distribution |
|------------------|---------------------|--------------|
| `0000000000.png` | 1370                | Good         |
| `0000000001.png` | 1301                | Good         |
| `0000000002.png` | 1361                | Good         |
| `0000000003.png` | 1358                | Good         |
| `0000000004.png` | 1333                | Good         |
| `0000000005.png` | 1284                | Good         |
| `0000000006.png` | 1322                | Good         |
| `0000000007.png` | 1366                | Good         |
| `0000000008.png` | 1389                | Good         |
| `0000000009.png` | 1339                | Good         |

- **FAST**

| Image            | Number of keypoints | Distribution |
|------------------|---------------------|--------------|
| `0000000000.png` | 1824                | Good         |
| `0000000001.png` | 1832                | Good         |
| `0000000002.png` | 1810                | Good         |
| `0000000003.png` | 1817                | Good         |
| `0000000004.png` | 1793                | Good         |
| `0000000005.png` | 1796                | Good         |
| `0000000006.png` | 1788                | Good         |
| `0000000007.png` | 1695                | Good         |
| `0000000008.png` | 1749                | Good         |
| `0000000009.png` | 1770                | Good         |

- **BRISK**

| Image            | Number of keypoints | Distribution |
|------------------|---------------------|--------------|
| `0000000000.png` | 2757                | Good         |
| `0000000001.png` | 2777                | Good         |
| `0000000002.png` | 2741                | Good         |
| `0000000003.png` | 2735                | Good         |
| `0000000004.png` | 2757                | Good         |
| `0000000005.png` | 2695                | Good         |
| `0000000006.png` | 2715                | Good         |
| `0000000007.png` | 2628                | Good         |
| `0000000008.png` | 2639                | Good         |
| `0000000009.png` | 2672                | Good         |

- **ORB**

| Image            | Number of keypoints | Distribution |
|------------------|---------------------|--------------|
| `0000000000.png` | 750                 | Good         |
| `0000000001.png` | 750                 | Good         |
| `0000000002.png` | 750                 | Good         |
| `0000000003.png` | 750                 | Good         |
| `0000000004.png` | 750                 | Good         |
| `0000000005.png` | 750                 | Good         |
| `0000000006.png` | 750                 | Good         |
| `0000000007.png` | 750                 | Good         |
| `0000000008.png` | 750                 | Good         |
| `0000000009.png` | 750                 | Good         |

- **AKAZE**

| Image            | Number of keypoints | Distribution |
|------------------|---------------------|--------------|
| `0000000000.png` | 1374                | Good         |
| `0000000001.png` | 1346                | Good         |
| `0000000002.png` | 1332                | Good         |
| `0000000003.png` | 1377                | Good         |
| `0000000004.png` | 1384                | Good         |
| `0000000005.png` | 1368                | Good         |
| `0000000006.png` | 1390                | Good         |
| `0000000007.png` | 1351                | Good         |
| `0000000008.png` | 1383                | Good         |
| `0000000009.png` | 1354                | Good         |

- **SIFT**

| Image            | Number of keypoints | Distribution |
|------------------|---------------------|--------------|
| `0000000000.png` | 1098                | Good         |
| `0000000001.png` | 1092                | Good         |
| `0000000002.png` | 1066                | Good         |
| `0000000003.png` | 1036                | Good         |
| `0000000004.png` | 1038                | Good         |
| `0000000005.png` | 1072                | Good         |
| `0000000006.png` | 1049                | Good         |
| `0000000007.png` | 1086                | Good         |
| `0000000008.png` | 1088                | Good         |
| `0000000009.png` | 1129                | Good         |

### MP.8 Performance Evaluation 2

| Detector      | Descriptor   | Number of matches |
|---------------|--------------|-------------------|
| `BRISK`       | `BRIEF`      | 1704              |
| `BRISK`       | `BRISK`      | 1570              |
| `BRISK`       | `FREAK`      | 1524              |
| `BRISK`       | `ORB`        | 1514              |
| `AKAZE`       | `BRIEF`      | 1266              |
| `AKAZE`       | `AKAZE`      | 1259              |
| `AKAZE`       | `BRISK`      | 1215              |
| `AKAZE`       | `FREAK`      | 1187              |
| `AKAZE`       | `ORB`        | 1182              |
| `FAST`        | `BRIEF`      | 1099              |
| `FAST`        | `ORB`        | 1071              |
| `FAST`        | `BRISK`      | 899               |
| `FAST`        | `FREAK`      | 878               |
| `SHITOMASI`   | `BRIEF`      | 944               | 
| `SHITOMASI`   | `ORB`        | 908               |
| `SHITOMASI`   | `FREAK`      | 768               |
| `SHITOMASI`   | `BRISK`      | 767               |
| `ORB`         | `ORB`        | 763               |
| `ORB`         | `BRISK`      | 751               |
| `ORB`         | `BRIEF`      | 545               |
| `ORB`         | `FREAK`      | 420               |
| `SIFT`        | `BRIEF`      | 702               |
| `SIFT`        | `FREAK`      | 593               |
| `SIFT`        | `BRISK`      | 592               |
| `HARRIS`      | `ORB`        | 146               |
| `HARRIS`      | `BRIEF`      | 146               |
| `HARRIS`      | `BRISK`      | 129               |
| `HARRIS`      | `FREAK`      | 128               |

### MP.9 Performance Evaluation 3

| Detector      | Descriptor   | Average processing time |
|---------------|--------------|-------------------------|
| `BRISK`       | `BRIEF`      | 0.503373                |
| `BRISK`       | `BRISK`      | 0.955101                |
| `BRISK`       | `FREAK`      | 0.559509                |
| `BRISK`       | `ORB`        | 0.509693                |
| `AKAZE`       | `BRIEF`      | 0.110748                |
| `AKAZE`       | `AKAZE`      | 0.201497                |
| `AKAZE`       | `BRISK`      | 0.564802                |
| `AKAZE`       | `FREAK`      | 0.166581                |
| `AKAZE`       | `ORB`        | 0.119912                |
| `FAST`        | `BRIEF`      | 0.003399                |
| `FAST`        | `ORB`        | 0.004997                |
| `FAST`        | `BRISK`      | 0.455582                |
| `FAST`        | `FREAK`      | 0.060688                |
| `SHITOMASI`   | `BRIEF`      | 0.024083                |
| `SHITOMASI`   | `ORB`        | 0.026113                |
| `SHITOMASI`   | `FREAK`      | 0.074561                |
| `SHITOMASI`   | `BRISK`      | 0.480870                |
| `ORB`         | `ORB`        | 0.020924                |
| `ORB`         | `BRISK`      | 0.462060                |
| `ORB`         | `BRIEF`      | 0.012005                |
| `ORB`         | `FREAK`      | 0.067033                |
| `SIFT`        | `BRIEF`      | 0.193730                |
| `SIFT`        | `FREAK`      | 0.246861                |
| `SIFT`        | `BRISK`      | 0.614790                |
| `HARRIS`      | `ORB`        | 0.020289                |
| `HARRIS`      | `BRIEF`      | 0.017496                |
| `HARRIS`      | `BRISK`      | 0.468443                |
| `HARRIS`      | `FREAK`      | 0.074067                |


## Final thoughts

The top 3 combinations are:
 
 1. FAST-BRIEF 
 2. FAST-ORB
 3. ORB-BRIEF
 
This is in terms of processing time since most of the detectors have a really good
distribution on the proceeding vehicle and number of keypoints.

[ring_buffer.h]: include/ring_buffer.h
[ring_buffer.cpp]: src/ring_buffer.cpp
[matching_2d.cpp]: src/matching_2d.cpp
[camera_fusion.cpp]: src/camera_fusion.cpp
