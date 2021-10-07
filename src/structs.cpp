#include "structs.h"
#include <opencv2/opencv.hpp>
#include <cmath>
#include <algorithm>

Line::Line(const Coordinate::Polar &polar) : polar(polar) {}

/**
 * @brief Converts a line to a line segment, containing start-end points.
 * @return Converted line segment.
 */
const LineSegment Line::to_line_segment() const
{
	static constexpr int64_t lineVal = 5000; //hard coded value to scale line, 5000 long so effectively covers most images.
	Radians theta = deg_to_radians(180.0 - polar.theta);

	Coordinate::Cartesian origin, destination;

	if (std::sin(theta) == 0)
	{
		origin = {static_cast<int64_t>(polar.r), static_cast<int64_t>(polar.r)};
		destination = {0, lineVal};
	}
	else
	{
		origin = {0, static_cast<int64_t>(polar.r / std::sin(theta))};
		destination = {lineVal, static_cast<int64_t>((polar.r - lineVal * std::cos(theta)) / std::sin(theta))};
	}
	return {LineClasses::UNKNOWN, origin, destination};
}
