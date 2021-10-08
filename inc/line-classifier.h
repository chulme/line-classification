#pragma once

#include <structs.h>
#include <numbers>
#include <unordered_map>
#include <utility>
#include <image.h>

/**
 * @brief Primative hashing function.
 */
struct container_hash
{
	std::size_t operator()(Line const &c) const
	{
		return std::hash<int>{}((c.polar.r * c.polar.theta) + c.polar.theta * c.polar.theta);
	}
};

/**
 * @brief Basic equality for hash map.
 */
struct container_equal
{
	bool operator()(Line const &c1, Line const &c2) const
	{
		return c1.polar.r == c2.polar.r && c1.polar.theta == c2.polar.theta;
	}
};

/**
 * @brief Class to classify lines of tennis court, using the number of intersections of each line.
 */
class LineClassifier
{
public:
	std::vector<Line> classify_lines(const Image &image, std::vector<Line> hough_lines, const bool debug = false);

private:
	static constexpr int8_t NUMBER_OF_HOUGH_INTERSECTIONS_FOR_HORZ_LINES = 5;
	static constexpr int8_t NUMBER_OF_INTERSECTIONS_FOR_BASE_LINE = 5;
	static constexpr int8_t NUMBER_OF_INTERSECTIONS_FOR_SERVICE_LINE = 3;

	std::unordered_map<Line, std::vector<Coordinate::Cartesian>, container_hash, container_equal> get_intersections(const std::vector<Line> &lines);
	void remove_false_horz_line_intersections(std::unordered_map<Line, std::vector<Coordinate::Cartesian>, container_hash, container_equal> &intersections, const Image &image);

	std::vector<ClassifiedLineSegment> classify_horz_lines(std::unordered_map<Line, std::vector<Coordinate::Cartesian>, container_hash, container_equal> &lines);
	std::vector<ClassifiedLineSegment> classify_vert_lines(const std::unordered_map<Line, std::vector<Coordinate::Cartesian>, container_hash, container_equal> &intersections, const std::vector<ClassifiedLineSegment> &horz_lines);

	Coordinate::Cartesian get_intersection(const Line &lineA, const Line &lineB);
	ClassifiedLineSegment get_target_line(const std::vector<ClassifiedLineSegment> &lines, const LineClasses target_class) const;
	Coordinate::Cartesian get_upper_image_intercept(const Coordinate::Cartesian p1, const Coordinate::Cartesian p2) const;

	void show_hough_transform(const std::vector<std::vector<double>> &hough_transform) const;
	void show_hough_lines(const std::vector<Line> &hough_lines, const Image &image) const;
	void show_classified_lines(const std::vector<ClassifiedLineSegment> &lines, const Image &image, const bool show_markers, const std::vector<Coordinate::Cartesian> &intersections = std::vector<Coordinate::Cartesian>()) const;
};
