#include <iostream>
#include <fstream>
#include <vector>
#include <structs.h>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

// Provided Image Details
constexpr int32_t image_width = 1392, image_height = 550;
constexpr std::string_view image_path = "res/image.raw";

int main()
{
	Image img("res/image.raw", image_width, image_height);
	img.show();
	cv::waitKey();
}