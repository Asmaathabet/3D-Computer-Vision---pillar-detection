#include "MatrixReaderWriter.h"
#include "PlaneEstimation.h"
#include "CylinderEstimation.h"
#include "SphereEstimation.h"
#include "PLYWriter.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

using namespace cv;

#define FILTER_LOWEST_DISTANCE 0.3 //threshold for pre-filtering

//1- Define RANSAC Parameters
#define RANSAC_ITER 3000 // iteration
#define THERSHOLD 2    //threshold [when threshold < 1 , there are many cylinders not detected by SEQUENTIAL RANSAC YET]


int main(int argc, char** argv) {

    if (argc != 4) {
        printf("Usage:\nObjectDetection input.xyz output.ply objType\n");
        exit(EXIT_FAILURE);

    }

    MatrixReaderWriter mrw(argv[1]);

    int num = mrw.rowNum;

    cout << "Rows:" << num << endl;
    cout << "Cols:" << mrw.columnNum << endl;

    // Read data from text file   
    vector<Point3f> points;

    for (int idx = 0;idx < num;idx++) {
        double x = mrw.data[3 * idx];
        double y = mrw.data[3 * idx + 1];
        double z = mrw.data[3 * idx + 2];

        float distFromOrigo = sqrt(x * x + y * y + z * z);


        // First filter: minimal work distance for a LiDAR limited.        
        if (distFromOrigo > FILTER_LOWEST_DISTANCE) {
            Point3f newPt;
            newPt.x = x;
            newPt.y = y;
            newPt.z = z;
            points.push_back(newPt);
        }

    }

    // Number of points:
    num = points.size();
    bool fitCylinder;
    string objType(argv[3]);
    if (objType == "cylinder") {
        fitCylinder = true;
    }
    else if (objType == "sphere") {
        fitCylinder = false;
    }
    else {
        printf("Usage:\nObjectDetection input.xyz output.ply objType\n");
        exit(EXIT_FAILURE);
    }
    RANSACDiffs differences;
    array<float, 4> params;

    if (fitCylinder) {
        //*********************************************
          // Using sequential RANSAC to detect pillars
        //*********************************************

        //1- in defining threshold and RANSAC iteractions above
        // 
        //2- declaring - clyninder count, all points and the original indices
        int cylinderCount = 0;
        vector<Point3f> remainingPoints(points);
        vector<int> originalIndices(points.size()); // the original index of each point
        iota(originalIndices.begin(), originalIndices.end(), 0); // [0, 1, 2, 3,4 ,..]

        //3- declaring- Ransac colors & cylinder colors
        vector<Point3i> RANSAC_Colors(points.size(), Point3i(255, 0, 0)); // all are red initially
        vector<Point3i> cylinder_Colors = {
            Point3i(160, 32, 240), // purple
            Point3i(255, 192, 203), // pink
            Point3i(255, 255, 0),   // yellow
            Point3i(0, 0, 255)      // blue
        };

        while (cylinderCount < 4 && !remainingPoints.empty()) {
            array<float, 4> params = EstimateCylinderRANSAC(remainingPoints, THERSHOLD, RANSAC_ITER);
            RANSACDiffs differences = CylinderPointRANSACDifferences(remainingPoints, params, THERSHOLD);

            // for the next iteration
            vector<Point3f> nextRemainingPoints;
            vector<int> nextOriginalIndices;

            // Color the inliers
            for (int i = 0; i < remainingPoints.size(); i++) {
                if (differences.isInliers.at(i)) {
                    RANSAC_Colors[originalIndices[i]] = cylinder_Colors[cylinderCount]; // Color the inliers with the current cylinder color
                }
                else {
                    nextRemainingPoints.push_back(remainingPoints[i]); // Save the point for the next iteration
                    nextOriginalIndices.push_back(originalIndices[i]); // Save the original index for the next iteration
                }
            }

            remainingPoints = nextRemainingPoints; // prepare the remaining points count
            originalIndices = nextOriginalIndices; // Update the original indices count

            cylinderCount++; // Increment the cylinder count
        }

        // [Output] Write the points and their colors to a PLY file
        WritePLY(argv[2], points, RANSAC_Colors);

    } else {
        // Sphere cylinder fitting
        // RANSAC-based robust sphere estimation
        params = EstimateSphereRANSAC(points, THERSHOLD, RANSAC_ITER);
        printf("Sphere params RANSAC:\n px:%f py:%f pz:%f r:%f \n", params[0], params[1], params[2], params[3]);

        // Compute differences of the fitted sphere in order to separate inliers from outliers
        differences = SpherePointRANSACDifferences(points, params, THERSHOLD);
  

        // Inliers and outliers are coloured by green and red, respectively
        vector<Point3i> RANSAC_Colors;

        for (int i = 0;i < num;i++) {
            Point3i newColor;

            if (differences.isInliers.at(i)) {
                newColor.x = 0;
                newColor.y = 255;
                newColor.z = 0;
            }
            else {
                newColor.x = 255;
                newColor.y = 0;
                newColor.z = 0;
            }
            RANSAC_Colors.push_back(newColor);
        }

        // Add the axis point of the sphere center with blue
        points.push_back(Point3f(params[0], params[1], params[2]));
        RANSAC_Colors.push_back(Point3i(0, 0, 255));

        // Write the points and their colors to a PLY file
        WritePLY(argv[2], points, RANSAC_Colors);

    }
}

