#pragma once

#include <array>
#include <vector>
#include <structs.h>

class Hough
{
public:
	std::vector<std::vector<double>> create_hough_transform(const Image& image, const bool debug = false);
	std::vector<Line> get_hough_lines(const Image& img, const std::vector<std::vector<double>>& hough_transform, const double threshold = 200, const bool debug = false) const;

private:
	static constexpr std::array<Degrees, 270> angles = []
	{
		std::array<Degrees, 270> angles;
		for (Degrees i = 0; i < angles.size(); i++)
			angles[i] = deg_to_radians(i - 90.0);
		return angles;
	}();

	std::vector<Coordinate::Cartesian> find_valid_sample_indices(const Image& image);
	double find_max_element(const std::vector<std::vector<double>>& two_dim_vec) const;
	void prune_lines(std::vector<Line>& lines) const;
	bool is_similar(const Line& line_a, const Line& line_b) const;
	void show_hough_transform(const std::vector<std::vector<double>>& hough_transform) const;
	void show_hough_lines(const std::vector<Line>& hough_lines, const Image& image) const;
};
