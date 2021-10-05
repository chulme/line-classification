#pragma once

#include <structs.h>
#include <numbers>
#include <unordered_map>

class LineClassifier
{
public:
	std::vector<Line> detect_lines(const Image& image, const uint64_t threshold = 100, const uint64_t line_threshold = 100, const bool debug = false);
private:
	static constexpr std::array<Degrees, 270> angles = []
	{
		std::array<Degrees, 270> angles;
		for (Degrees i = 0; i < angles.size(); i++)
			angles[i] = deg_to_radians(i - 90.0);
		return angles;
	}();

	std::vector<std::vector<double>> create_hough_transform(const Image& image);
	std::vector<Coordinate::Cartesian> find_valid_sample_indices(const Image& image);
	std::vector<Line> get_hough_lines(const std::vector<std::vector<double>>& hough_transform, const double threshold) const;
	double find_max_element(const std::vector<std::vector<double>>& two_dim_vec) const;

	std::vector<Coordinate::Cartesian> get_intersections(const std::vector<Line>& lines, const Image& image);
	void show_hough_transform(const std::vector<std::vector<double>>& hough_transform) const;
	void show_hough_lines(const std::vector<Line>& hough_lines, const Image& image) const;
};
