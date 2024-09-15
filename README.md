# Object Detection with RANSAC

This project implements RANSAC-based methods to detect geometric shapes—specifically cylinders and spheres—in 3D point cloud data. The program reads a point cloud from an input file and outputs the results as a PLY file with detected shapes color-coded.

## Overview

The code processes 3D point cloud data to detect either cylindrical or spherical shapes. It involves filtering points based on distance, applying RANSAC to detect shapes, and outputting the results in a PLY file format for visualization.

## Features

- **Point Filtering:** Filters out points based on a minimum distance from the origin.
- **Cylinder Detection:** Detects cylindrical shapes using RANSAC and colors the inliers.
- **Sphere Detection:** Detects spherical shapes using RANSAC and colors the inliers.
- **PLY File Output:** Saves the detected shapes and point cloud data in a PLY file format for visualization.

## Dependencies

- **OpenCV:** Required for handling 3D point cloud data and visualizing results. Install with:

  ```bash
  pip install opencv-python
  ```
C++ Compiler: Ensure you have a C++ compiler that supports C++11 or later.
Compilation and Execution
- Clone the Repository

```bash
git clone https://github.com/yourusername/object-detection-ransac.git
cd object-detection-ransac
```
```bash
g++ -o ObjectDetection main.cpp -lopencv_core -lopencv_highgui -lopencv_imgproc
```

## Run the Program

```bash
./ObjectDetection input.xyz output.ply objType
```
Replace input.xyz with the path to your input file containing the point cloud data.
Replace output.ply with the path where the output PLY file will be saved.
Replace objType with cylinder or sphere to specify the type of shape to detect.

## Code Explanation
### main Function
The main function validates the input arguments, reads and filters points, and calls the appropriate detection function based on the specified object type.

```bash cpp
int main(int argc, char** argv) {
    if (argc != 4) {
        printf("Usage:\nObjectDetection input.xyz output.ply objType\n");
        exit(EXIT_FAILURE);
    }

    MatrixReaderWriter mrw(argv[1]);
    vector<Point3f> points = ReadAndFilterPoints(mrw);

    string objType(argv[3]);
    if (objType == "cylinder") {
        DetectCylinders(points, argv[2]);
    } else if (objType == "sphere") {
        DetectSpheres(points, argv[2]);
    } else {
        printf("Usage:\nObjectDetection input.xyz output.ply objType\n");
        exit(EXIT_FAILURE);
    }
}
```
### ReadAndFilterPoints Function
Filters out points based on their distance from the origin.

```bash cpp
vector<Point3f> ReadAndFilterPoints(MatrixReaderWriter& mrw) {
    vector<Point3f> points;
    for (int idx = 0; idx < mrw.rowNum; idx++) {
        double x = mrw.data[3 * idx];
        double y = mrw.data[3 * idx + 1];
        double z = mrw.data[3 * idx + 2];
        float distFromOrigo = sqrt(x * x + y * y + z * z);
        if (distFromOrigo > FILTER_LOWEST_DISTANCE) {
            points.push_back(Point3f(x, y, z));
        }
    }
    return points;
}
```

### DetectCylinders Function
Detects and colors cylindrical shapes using RANSAC.

```bash cpp
void DetectCylinders(vector<Point3f>& points, const char* outputFile) {
    int cylinderCount = 0;
    vector<Point3f> remainingPoints(points);
    vector<int> originalIndices(points.size());
    iota(originalIndices.begin(), originalIndices.end(), 0);
    vector<Point3i> RANSAC_Colors(points.size(), Point3i(255, 0, 0));
    vector<Point3i> cylinder_Colors = { ... }; // Defined colors

    while (cylinderCount < 4 && !remainingPoints.empty()) {
        auto params = EstimateCylinderRANSAC(remainingPoints, THERSHOLD, RANSAC_ITER);
        auto differences = CylinderPointRANSACDifferences(remainingPoints, params, THERSHOLD);
        // Color and filter points based on detected cylinders
        remainingPoints = FilterRemainingPoints(remainingPoints, originalIndices, differences, cylinder_Colors[cylinderCount]);
        cylinderCount++;
    }
    WritePLY(outputFile, points, RANSAC_Colors);
}
```
### DetectSpheres Function
Detects and colors spherical shapes using RANSAC.

```bash cpp
void DetectSpheres(vector<Point3f>& points, const char* outputFile) {
    auto params = EstimateSphereRANSAC(points, THERSHOLD, RANSAC_ITER);
    auto differences = SpherePointRANSACDifferences(points, params, THERSHOLD);
    vector<Point3i> RANSAC_Colors;
    for (const auto& point : points) {
        RANSAC_Colors.push_back(differences.isInliers[&point - &points[0]] ? Point3i(0, 255, 0) : Point3i(255, 0, 0));
    }
    points.push_back(Point3f(params[0], params[1], params[2]));
    RANSAC_Colors.push_back(Point3i(0, 0, 255));
    WritePLY(outputFile, points, RANSAC_Colors);
}
```

## File Formats

- Input File: Must be a matrix of 3D points in XYZ format.
- Output File: PLY file containing the processed point cloud with detected shapes color-coded.
vbnet

