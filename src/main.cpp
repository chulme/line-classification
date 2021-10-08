#include <structs.h>
#include <hough.h>
#include <line-classifier.h>
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

int main()
{
	Image img("res/image.raw", image_width, image_height);

	binarize(img, 150);

	Hough hough_transformer;
	auto hough_transform = hough_transformer.create_hough_transform(img, true);
	auto hough_lines = hough_transformer.get_hough_lines(img, hough_transform, 200,true);

	LineClassifier classifier;
	classifier.classify_lines(img, hough_lines);
	cv::waitKey();
}