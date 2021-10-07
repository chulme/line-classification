#include "Image.h"


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

Image::Image(const std::vector<uint8_t>& vec, const uint32_t width, const uint32_t height) : samples(vec), width(width), height(height)
{
}

/**
 * @brief Reads .raw image into buffer of bytes.
 * @param path - Path to .raw file.
 * @return Image buffered as bytes
**/
std::vector<uint8_t> Image::getImageBuffer(const std::string_view path, const uint32_t width, const uint32_t height)
{
	uint8_t* buffer = new uint8_t[width * height];

	FILE* fp = fopen(path.data(), "rb");
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
	return cv::Mat(height, width, CV_8UC1, const_cast<uint8_t*>(samples.data()));
}

/**
 * @brief Displays the image using OpenCV, until user presses key on OpenCV window.
**/
void Image::show(const std::string_view image_name) const
{
	cv::Mat cv_image = this->convert_to_mat();
	cv::imshow(image_name.data(), cv_image);
}

bool Image::does_block_contain_samples(const int32_t index, const int32_t horz_size, const int32_t vert_size) const
{
	bool valid_samples_in_block = false;
	Coordinate::Cartesian coords = index_to_coordinate(index);
	coords.x -= horz_size / 2;
	coords.y -= vert_size / 2; //centers search around index.

	for (int r = coords.y; r < coords.y + vert_size; r++)		  //row
		for (int c = coords.x; c < coords.x + horz_size; c++)	  //cols
			if (r * this->width + c < this->width * this->height) //prevent oob exception
				if (this->samples[r * this->width + c] != 0)
					valid_samples_in_block = true;

	return valid_samples_in_block;
}

Coordinate::Cartesian Image::index_to_coordinate(const int32_t index) const
{
	return Coordinate::Cartesian(index / width, index % width);
}

size_t Image::coordinate_to_index(const Coordinate::Cartesian coord) const
{
	return this->width * coord.x + coord.y;
}