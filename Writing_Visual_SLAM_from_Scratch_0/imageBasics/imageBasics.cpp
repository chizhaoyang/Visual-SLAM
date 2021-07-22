#include <iostream>
#include <chrono>

using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv) {
    // read image from path argv[1]
    cv::Mat image;
    image = cv::imread(argv[1]);

    // check if image is read correctly
    if (image.data == nullptr) {
        cerr << "File " << argv[1] << " does not exist." << endl;
        return 0;
    }

    cout << "Image width: " << image.cols << ", height: " << image.rows
         << ", number of channels: " << image.channels() << endl;

    cv::imshow("image", image); // show image using cv:imshow
    cout << "Press any key to continue..." << endl;
    cv::waitKey(0); // pause code, waiting for any key

    return 0;
}