#include "Image.h"

/**
 * @brief Constructs image object from raw file.
 * @param[in] path - Path to .raw file.
 * @param[in] width - Width of image.
 * @param[in] height - Height of image.
 */
Image::Image(const std::string_view path, const uint32_t width, const uint32_t height)
	: samples(getImageBuffer(path, width, height)), width(width), height(height)
{
}

/**
 * @brief Constructs image object from vector of samples.
 * @param[in] vec - Vector of image samples.
 * @param[in] width - Width of image.
 * @param[in] height - Height of image.
 */
Image::Image(const std::vector<uint8_t> &vec, const uint32_t width, const uint32_t height) : samples(vec), width(width), height(height)
{
}

/**
 * @brief Reads .raw image into buffer of bytes.
 * @param[in] path - Path to .raw file.
 * @param[in] width - Width of image.
 * @param[in] height - Height of image.
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

/**
 * @brief Converts image to OpenCV Mat object for visualisation.
 * @return OpenCV Mat object.
 */
cv::Mat Image::convert_to_mat() const
{
	return cv::Mat(height, width, CV_8UC1, const_cast<uint8_t *>(samples.data()));
}

/**
 * @brief Displays the image using OpenCV, until user presses key on OpenCV window.
 * @param[in] image_name - The name of the image OpenCV window.
**/
void Image::show(const std::string_view image_name) const
{
	cv::Mat cv_image = this->convert_to_mat();
	cv::imshow(image_name.data(), cv_image);
}

/**
 * @brief Scans ROI of image to determine if any valid (non-0) samples exist.
 * @param[in] index - Sample index which acts as the ROI centre point.
 * @param[in] horz_size - Horizontal size of the ROI.
 * @param[in] vert_size - Vertical size of the ROI.
 * @return Boolean flag indicating if ROI contains valid samples.
 */
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

/**
 * @brief Converts 1D index to Cartesian coordinate.
 * @param[in] index- 1D index to convert.
 * @return Cartesian coordinate of index.
 */
Coordinate::Cartesian Image::index_to_coordinate(const int32_t index) const
{
	return Coordinate::Cartesian(index / width, index % width);
}
/**
 * @brief Converts cartesian coordinate to 1D index.
 * @param[in] coord - Cartesian coordinate to convert.
 * @return 1D index of the associated cartesian coordinate of the image.
 */
size_t Image::coordinate_to_index(const Coordinate::Cartesian coord) const
{
	return this->width * coord.x + coord.y;
}