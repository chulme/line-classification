#include "structs.h"
#include <opencv2/opencv.hpp>
#include <cmath>
#include <algorithm>

Line::Line(const Coordinate::Polar &polar) : polar(polar) {}

Coordinate::Cartesian Line::polar_to_cartesian() const
{
	return Coordinate::Cartesian(polar.r * std::cos(polar.theta), polar.r * std::sin(polar.theta));
}

/**
 * @brief Constructs image object from raw file.
 * @param path - Path to .raw file.
 * @param width - Width of image.
 * @param height - Height of image.
**/
Image::Image(const std::string_view path, const uint32_t width, const uint32_t height)
	: samples(getImageBuffer(path, width, height)), width(width), height(height)
{
}

Image::Image(const std::vector<uint8_t> &vec, const uint32_t width, const uint32_t height) : samples(vec), width(width), height(height)
{
}

/**
 * @brief Reads .raw image into buffer of bytes.
 * @param path - Path to .raw file.
 * @return Image buffered as bytes
**/
std::vector<uint8_t> Image::getImageBuffer(const std::string_view path, const uint32_t width, const uint32_t height)
{
	uint8_t *buffer = new uint8_t[width * height];

	FILE *fp = fopen(path.data(), "rb");
	if (fp)
	{
		fread(buffer, width * height, 1, fp);
		fclose(fp);
	}
	std::vector<uint8_t> vec(buffer, buffer + (width * height));
	return vec;
}

cv::Mat Image::convert_to_mat() const
{
	return cv::Mat(height, width, CV_8UC1, const_cast<uint8_t *>(samples.data()));
}

/**
 * @brief Displays the image using OpenCV, until user presses key on OpenCV window.
**/
void Image::show(const std::string_view image_name) const
{
	cv::Mat cv_image = this->convert_to_mat();
	cv::imshow(image_name.data(), cv_image);
}

Coordinate::Cartesian Image::index_to_coordinate(const int32_t index) const
{
	return Coordinate::Cartesian(index / width, index % width);
}