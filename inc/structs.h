#pragma once

#include <string_view>
#include <vector>
#include <opencv2/opencv.hpp>
#include <numbers>

typedef double Degrees;
typedef double Radians;

inline constexpr Radians deg_to_radians(const Degrees& degrees)
{
	return degrees * (std::numbers::pi / 180.0);
}

inline constexpr Degrees rad_to_degrees(const Radians& radians)
{
	return radians * 180.0 / std::numbers::pi;
}

inline Degrees angle_difference_d(const Degrees& a, const Degrees& b)
{
	double diff = fmod((b - a + 180), 360.0) - 180;
	return (180 - std::abs(diff) < diff) ? std::abs(180 - std::abs(diff)) : diff;
}

inline Radians angle_difference_r(const Radians& a, const Radians& b) {
	return  deg_to_radians(angle_difference_d(rad_to_degrees(a), rad_to_degrees(b)));
}

struct Colour {
	int16_t r, g, b;
};
namespace Coordinate
{
	struct Cartesian
	{
		int64_t x, y;
	};

	struct Polar
	{
		double r;
		Degrees theta;
	};
}

class Line
{
public:
	Coordinate::Polar polar;
	bool is_vertical() const { return !(polar.theta < 150 && polar.theta >45); }
	Line(const Coordinate::Polar& polar);
	Coordinate::Cartesian polar_to_cartesian() const;

	std::array<Coordinate::Cartesian, 2>to_line_segment() const;
private:
	double gradient, x, y, c;
};

class Image
{
public:
	Image(const std::string_view path, const uint32_t width, const uint32_t height);
	Image(const std::vector<uint8_t>& vec, const uint32_t width, const uint32_t height);

	void show(const std::string_view image_name) const;
	std::vector<uint8_t> samples;
	Coordinate::Cartesian index_to_coordinate(const int32_t index) const;
	cv::Mat convert_to_mat() const;
	const uint32_t width, height;

private:
	std::vector<uint8_t> getImageBuffer(const std::string_view path, const uint32_t width, const uint32_t height);
};
