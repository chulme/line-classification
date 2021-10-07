#include <string_view>
#include <vector>

#include<structs.h>

#pragma once
class Image
{
public:
	Image(const std::string_view path, const uint32_t width, const uint32_t height);
	Image(const std::vector<uint8_t>& vec, const uint32_t width, const uint32_t height);

	void show(const std::string_view image_name) const;
	std::vector<uint8_t> samples;
	Coordinate::Cartesian index_to_coordinate(const int32_t index) const;
	size_t coordinate_to_index(const Coordinate::Cartesian coord) const;

	cv::Mat convert_to_mat() const;
	const uint32_t width, height;
	bool does_block_contain_samples(const int32_t index, const int32_t horz_size, const int32_t vert_size) const;

private:
	std::vector<uint8_t> getImageBuffer(const std::string_view path, const uint32_t width, const uint32_t height);
};
