#pragma once

#include <structs.h>
#include <numbers>
#include <unordered_map>
#include <utility>
#include <image.h>

struct LineSegment;
class Line;

struct container_hash
{
	std::size_t operator()(Line const& c) const
	{
		return std::hash<int>{}((c.polar.r * c.polar.theta) + c.polar.theta * c.polar.theta);
	}
};

struct container_equal
{
	bool operator()(Line const& c1, Line const& c2) const
	{
		return c1.polar.r == c2.polar.r && c1.polar.theta == c2.polar.theta;
	}
};

class LineClassifier
{
public:
	std::vector<Line> classify_lines(const Image& image, std::vector<Line> hough_lines, const bool debug = false);

private:
	static constexpr int8_t NUMBER_OF_HOUGH_INTERSECTIONS_FOR_HORZ_LINES = 5;
	static constexpr int8_t NUMBER_OF_INTERSECTIONS_FOR_BASE_LINE = 5;
	static constexpr int8_t NUMBER_OF_INTERSECTIONS_FOR_SERVICE_LINE = 3;

	std::unordered_map<Line, std::vector<Coordinate::Cartesian>, container_hash, container_equal> get_intersections(const std::vector<Line>& lines);
	void remove_false_horz_line_intersections(std::unordered_map<Line, std::vector<Coordinate::Cartesian>, container_hash, container_equal>& intersections, const Image& image);

	std::vector<LineSegment> classify_horz_lines(std::unordered_map<Line, std::vector<Coordinate::Cartesian>, container_hash, container_equal>& lines);
	std::vector<LineSegment> classify_vert_lines(const std::unordered_map<Line, std::vector<Coordinate::Cartesian>, container_hash, container_equal>& intersections, const std::vector<LineSegment>& horz_lines);

	Coordinate::Cartesian get_intersection(const Line& lineA, const Line& lineB);
	LineSegment get_target_line(const std::vector<LineSegment>& lines, const LineClasses target_class) const;

	void show_hough_transform(const std::vector<std::vector<double>>& hough_transform) const;
	void show_hough_lines(const std::vector<Line>& hough_lines, const Image& image) const;
	void show_classified_lines(const std::vector<LineSegment>& lines, const Image& image) const;
};
