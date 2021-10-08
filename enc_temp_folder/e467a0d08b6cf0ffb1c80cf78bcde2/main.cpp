#include <iostream>
#include <fstream>
#include <vector>

#include <structs.h>
#include <hough.h>
#include "line-classifier.h"

#include <opencv2/opencv.hpp>

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

/*
	cv::Mat original = base.convert_to_mat();
	cv::Mat reduced, repositioned(cv::Size(image_width, image_height), CV_8UC1, cv::Scalar(0));
	cv::resize(original, reduced, cv::Size(image_width * scale_factor, image_height * scale_factor));
	reduced.copyTo(repositioned(cv::Rect(150, 0, reduced.cols, reduced.rows)));
	//Image cropped()
	cv::imshow("Original", repositioned);

	std::vector<uchar> v(repositioned.data, repositioned.data + repositioned.total() * repositioned.channels());
	Image img(v, image_width, image_height);

	binarize(img, 150);
	cv::imshow("Binarised", img.convert_to_mat());

	LineClassifier classifier;
	classifier.detect_lines(img, 200, 100, true);
	cv::waitKey();
*/
int main()
{
	Image img("res/image.raw", image_width, image_height);
	cv::imshow("Original", img.convert_to_mat());

	/*
	constexpr int horz_crop = 5;
	cv::Mat mat = base.convert_to_mat();
	cv::Mat cropped(mat, cv::Rect(horz_crop, 0, image_width - horz_crop, image_height));
	std::vector<uchar> v(cropped.data, cropped.data + cropped.total() * cropped.channels());
	//v.assign(cropped.data,cropped.data+cropped.total()*cropped.channels());
	Image img(v,image_width,image_height);

	//Image cropped()
	cv::imshow("Original", cropped);
	*/

	binarize(img, 150);

	Hough hough_transformer;
	auto hough_transform = hough_transformer.create_hough_transform(img, true);
	auto hough_lines = hough_transformer.get_hough_lines(img, hough_transform, 200,true);

	LineClassifier classifier;
	classifier.classify_lines(img, hough_lines, true);
	cv::waitKey();
}