# Compile / Run Instruction

## 1. Plane Segmentation - RANSAC
In `src/quiz/ransac`:
```bash
mkdir build
cd build
cmake ..
make
./quizRansac
```

## 2. Eucledian Clustering - KDTree
In `src/quiz/cluster`:
```bash
mkdir build
cd build
cmake ..
make
./quizCluster
```

Note:
* To empty out or remove all files in `build` from its directory, use `rm -rf ./*`