#include "structs.h"
#include <opencv2/opencv.hpp>

/**
 * @brief Constructs image object from raw file.
 * @param path - Path to .raw file.
 * @param width - Width of image.
 * @param height - Height of image.
**/
Image::Image(const std::string_view path, const uint32_t width, const uint32_t height)
	: number_of_pixels(width* height), data(getImageBuffer(path)), width(width), height(height)
{
}

/**
 * @brief Reads .raw image into buffer of bytes.
 * @param path - Path to .raw file.
 * @return Image buffered as bytes
**/
std::vector<std::byte> Image::getImageBuffer(const std::string_view path)
{
	std::byte* buffer = new std::byte[number_of_pixels];

	FILE* fp = fopen(path.data(), "rb");
	if (fp)
	{
		fread(buffer, number_of_pixels, 1, fp);
		fclose(fp);
	}
	return std::vector<std::byte>(buffer, buffer + number_of_pixels);
}

/**
 * @brief Displays the image using OpenCV.
**/
void Image::show() const
{
	cv::Mat cv_image(height, width, CV_8UC1, const_cast<std::byte*>(data.data()));
	cv::imshow("image", cv_image);
}