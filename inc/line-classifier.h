#pragma once

#include <structs.h>
#include <numbers>
#include <unordered_map>
#include <utility>

enum class LineClasses
{
	BASE_LINE,
	SERVICE_LINE,
	CENTRE_SERVICE_LINE,
	SINGLES_SIDELINE,
	DOUBLES_SIDELINE
};
struct container_hash
{
	std::size_t operator()(Line const &c) const
	{
		return std::hash<int>{}((c.polar.r * c.polar.theta) + c.polar.theta * c.polar.theta);
	}
};

struct container_equal
{
	bool operator()(Line const &c1, Line const &c2) const
	{
		return c1.polar.r == c2.polar.r && c1.polar.theta == c2.polar.theta;
	}
};

class LineClassifier
{
public:
	std::vector<Line> detect_lines(const Image &image, const uint64_t threshold = 100, const uint64_t line_threshold = 100, const bool debug = false);

private:
	static constexpr std::array<Degrees, 270> angles = []
	{
		std::array<Degrees, 270> angles;
		for (Degrees i = 0; i < angles.size(); i++)
			angles[i] = deg_to_radians(i - 90.0);
		return angles;
	}();

	std::vector<std::vector<double>> create_hough_transform(const Image &image);
	std::vector<Coordinate::Cartesian> find_valid_sample_indices(const Image &image);
	std::vector<Line> get_hough_lines(const std::vector<std::vector<double>> &hough_transform, const double threshold) const;
	double find_max_element(const std::vector<std::vector<double>> &two_dim_vec) const;

	void prune_lines(std::vector<Line> &lines);
	bool is_similar(const Line &line_a, const Line &line_b);

	std::unordered_map<Line, std::vector<Coordinate::Cartesian>, container_hash, container_equal> get_intersections(const std::vector<Line> &lines, const Image &image);
	void remove_false_intersections(std::unordered_map<Line, std::vector<Coordinate::Cartesian>, container_hash, container_equal> &intersections, const Image &image);

	std::unordered_map<LineClasses, Line> classify_lines(const std::unordered_map<Line, std::vector<Coordinate::Cartesian>, container_hash, container_equal> &lines);
	Coordinate::Cartesian get_intersection(const Line &lineA, const Line &lineB);
	void show_hough_transform(const std::vector<std::vector<double>> &hough_transform) const;
	void show_hough_lines(const std::vector<Line> &hough_lines, const Image &image) const;
};
