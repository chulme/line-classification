#pragma once

#include <string_view>
#include <vector>

typedef double Degrees;

namespace Coordinate
{
	struct Cartesian
	{
		int x, y;
	};

	struct Polar
	{
		double r;
		Degrees theta;
	};
}

class Line
{
};

class Image
{
public:
	Image(const std::string_view path, const uint32_t width, const uint32_t height);
	void show() const;

private:
	const uint32_t width, height,number_of_pixels;
	std::vector<std::byte> data;
	std::vector<std::byte> getImageBuffer(const std::string_view path);
};
