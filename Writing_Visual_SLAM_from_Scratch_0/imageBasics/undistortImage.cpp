// with opencv, you can use cv::Undistort()
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <string>

using namespace std;
string image_file = "../distorted.png"; // make sure the path is right

int main(int argc, char **argv) {
    // parameters of distortion
    double k1 = -0.28340811, k2 = 0.07395907, p1 = 0.00019359, p2 = 1.76187114e-05;
    // camera intrinsics
    double fx = 458.654, fy = 457.296, cx = 367.215, cy = 248.375;

    cv::Mat image = cv::imread(image_file, 0); // image is gray image, CV_8UC1

    int rows = image.rows, cols = image.cols;
    cv::Mat image_undistort = cv::Mat(rows, cols, CV_8UC1); // undistorted image

    // get undistorted image
    for (int v = 0; v < rows; v++) {
        for (int u = 0; u < cols; u++) {
            // calculate (u,v) in distorted image
            double x = (u - cx) / fx, y = (v - cy) / fy;
            double r = sqrt(x * x + y * y);
            double x_distorted = x * (1 + k1 * r * r + k2 * r * r * r * r) + 2 * p1 * x * y + p2 * (r * r + 2 * x * x);
            double y_distorted = y * (1 + k1 * r * r + k2 * r * r * r * r) + p1 * (r * r + 2 * y * y) + 2 * p2 * x * y;
            double u_distorted = fx * x_distorted + cx;
            double v_distorted = fy * y_distorted + cy;

            // get value (interpolation)
            if (u_distorted >= 0 && v_distorted >= 0 && u_distorted < cols && v_distorted < rows) {
                image_undistort.at<uchar>(v, u) = image.at<uchar>((int) v_distorted, (int) u_distorted);
            } else {
                image_undistort.at<uchar>(v, u) = 0;
            }
        }
    }

    // draw images
    cv::imshow("distorted", image);
    cv::imshow("undistorted", image_undistort);
    // cv::waitKey(0);

    // using OpenCV for undistortion
    cv::Mat image_undistort_opencv = cv::Mat(rows, cols, CV_8UC1); // undistorted image
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    cv::Mat dist_coeffs = (cv::Mat_<double>(1,4) << k1, k2, p1, p2); 

    cv::undistort(image, image_undistort_opencv, camera_matrix, dist_coeffs);
    cv::imshow("undistorted_opencv", image_undistort_opencv);
    cv::waitKey(0);
    
    return 0;
}