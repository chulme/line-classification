#include <iostream>
#include <fstream>
#include <vector>

#include <structs.h>
#include "line-classifier.h"

#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

// Provided Image Details
constexpr int32_t image_width = 1392, image_height = 550;
constexpr std::string_view image_path = "res/image.raw";

void binarize(Image& img, uint32_t threshold)
{
	int i = 0;
	for (uint8_t& sample : img.samples)
	{
		sample = (sample > threshold) ? 255 : 0;
	}
}

void noise_removal(Image& img, const int n)
{
	int i, key, j;
	for (i = 1; i < n; i++)
	{
		key = img.samples[i];
		j = i - 1;

		/* Move elements of arr[0..i-1], that are
		greater than key, to one position ahead
		of their current position */
		while (j >= 0 && img.samples[j] > key)
		{
			img.samples[j + 1] = img.samples[j];
			j = j - 1;
		}
		img.samples[j + 1] = key;
	}
}

int main()
{
	Image img("res/image.raw", image_width, image_height);

	binarize(img, 150);
	noise_removal(img, 90);

	cv::Mat base = img.convert_to_mat();
	cv::Mat blurred;
	cv::blur(base, blurred, cv::Size(10, 10));
	cv::Mat edges;
	cv::Canny(blurred, edges, 100, 200);
	//cv::imwrite("processed.tif", edges);

	cv::Mat flat = edges.reshape(1, edges.total() * edges.channels());
	std::vector<uint8_t> vec = edges.isContinuous() ? flat : flat.clone();
	Image processed_img(vec, image_width, image_height);

	LineClassifier classifier;
	classifier.detect_lines(processed_img, 100, 100, true);
	cv::waitKey();
}