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

    // check the type of the image
    if (image.type() != CV_8UC1 && image.type() != CV_8UC3) {
        cout << "Please input a color image or gray image!" << endl;
        return 0;
    }

    // timing using std::chrono
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    for (size_t y = 0; y < image.rows; y++) {
        // using cv::Mat::ptr for pointer of row
        unsigned char *row_ptr = image.ptr<unsigned char>(y); // row_ptr is the head pointer of yth row
        for (size_t x = 0; x < image.cols; x++) {
            // get information of pixel at (x, y)
            unsigned char *data_ptr = &row_ptr[x * image.channels()]; // data_prt point to information
            // output channel information of this pixel
            for (int c = 0; c != image.channels(); c++) {
                unsigned char data = data_ptr[c];
            }
        }
    }
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast <chrono::duration<double>> (t2 - t1);
    cout << "Time used for over the image: " << time_used.count() << " s." << endl;

    // copy of cv::Mat
    // using = cannot copy data
    cv::Mat image_another = image;
    // modify image_another will lead the change of image
    image_another(cv::Rect(0, 0, 100, 100)).setTo(0); // set 0 to left top corner
    cv::imshow("image", image);
    cout << "Press any key to continue..." << endl;
    cv::waitKey(0); 

    // using clone to copy data
    cv::Mat image_clone = image.clone();
    image_clone(cv::Rect(0, 0, 100, 100)).setTo(255);
    cv::imshow("image", image);
    cv::imshow("image_clone", image_clone);
    cout << "Press any key to continue..." << endl;
    cv::waitKey(0);

    cv::destroyAllWindows();
    return 0;
}