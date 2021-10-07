#pragma once

#include <array>
#include <vector>
#include <structs.h>

class Hough
{
public:
	std::vector<std::vector<double>> create_hough_transform(const Image& image);
	std::vector<Line> get_hough_lines(const std::vector<std::vector<double>>& hough_transform, const double threshold) const;

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
};
