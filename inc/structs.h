#pragma once

#include <string_view>
#include <vector>
#include <opencv2/opencv.hpp>
#include <numbers>

typedef double Degrees;
typedef double Radians;

/**
 * @brief Converts degrees to radians.
 */
inline constexpr Radians deg_to_radians(const Degrees &degrees)
{
	return degrees * (std::numbers::pi / 180.0);
}

/**
 * @brief Converts radians to degrees.
 */
inline constexpr Degrees rad_to_degrees(const Radians &radians)
{
	return radians * 180.0 / std::numbers::pi;
}

/**
 * @brief Calculates the smallest angle between 2 degree angles
 * @param[in] a - First angle to calculate difference
 * @param[in] b - Second radian to calculate difference
 * @return Angle difference, in radians.
 */
inline Degrees angle_difference_d(const Degrees &a, const Degrees &b)
{
	double diff = fmod((b - a + 180), 360.0) - 180;
	return (180 - std::abs(diff) < diff) ? std::abs(180 - std::abs(diff)) : diff;
}

/**
 * @brief Calculates the smallest angle between 2 radian angles.
 * @param[in] a - First angle to calculate difference.
 * @param[in] b - Second angle to calculate difference.
 * @return Angle difference, in radians.
 */
inline Radians angle_difference_r(const Radians &a, const Radians &b)
{
	return deg_to_radians(angle_difference_d(rad_to_degrees(a), rad_to_degrees(b)));
}

namespace Coordinate
{
	/**
	 * @brief Cartesian x-y coordinate
	 */
	struct Cartesian
	{
		int64_t x, y;

		Cartesian operator/=(const int64_t denominator)
		{
			return {x /= denominator, y /= denominator};
		}

		Cartesian operator+=(const Cartesian &c)
		{
			return {x += c.x, y += c.y};
		}

		Cartesian operator/(const int64_t denominator)
		{
			return {x /= denominator, y /= denominator};
		}

		Cartesian operator/(const Cartesian c)
		{
			return {x / c.x, y / c.y};
		}

		Cartesian operator=(const Cartesian c)
		{
			return {x = c.x, y = c.y};
		}
		bool operator==(const Cartesian c)
		{
			return (x == c.x && y == c.y);
		}

		friend Cartesian operator+(const Cartesian &c1, const Cartesian &c2) { return {c1.x + c2.x, c1.y + c2.y}; };
	};

	/**
	 * @brief Polar (r-theta) coordinate.
	 */
	struct Polar
	{
		double r;
		Degrees theta;
	};
}

/**
 * @brief Enum of possible line classifications.
 * @note Contains both user-relevant classifications, and implementation-specific classifications.
 */
enum class LineClasses
{
	// User-relevant classes
	UNKNOWN,
	BASE_LINE,
	SERVICE_LINE,
	CENTRE_SERVICE_LINE,
	SINGLES_SIDELINE,
	DOUBLES_SIDELINE,

	// Implementation-specific classes
	INNER_BASE_LINE,
	INNER_BASE_HALF_LINE,

	SERVICE_LINE_HALF,
	SERVICE_LINE_DOUBLES,
};

/**
 * @brief Class representing line segment, containing start-end points and a classification.
 */
struct ClassifiedLineSegment
{
public:
	LineClasses line_class = LineClasses::UNKNOWN;
	Coordinate::Cartesian origin = {0, 0};
	Coordinate::Cartesian destination = {0, 0};

	ClassifiedLineSegment();
	ClassifiedLineSegment(const LineClasses &line_class, const Coordinate::Cartesian &origin, const Coordinate::Cartesian &destination) : line_class(line_class), origin(origin), destination(destination) {}
	ClassifiedLineSegment(const LineClasses &line_class, const ClassifiedLineSegment &segment) : line_class(line_class), origin(segment.origin), destination(segment.destination) {}
	ClassifiedLineSegment(const Coordinate::Cartesian &origin, const Coordinate::Cartesian &destination) : line_class(LineClasses::UNKNOWN), origin(origin), destination(destination) {}

	bool does_intersect(const ClassifiedLineSegment &line_segment) const
	{
		return origin.x <= line_segment.destination.x && destination.x >= origin.x && origin.y <= line_segment.destination.y && destination.y >= origin.y;
	}

	ClassifiedLineSegment operator=(const ClassifiedLineSegment &s)
	{
		return {line_class = s.line_class, origin = s.origin, destination = s.destination};
	}

	bool operator==(const ClassifiedLineSegment &s)
	{
		return (line_class == s.line_class && origin == s.origin && destination == s.destination);
	}
};

/**
 * @brief Class representing line in Polar form.
 */
class Line
{
public:
	Coordinate::Polar polar;
	bool is_vertical() const { return !(polar.theta < 150 && polar.theta > 45); }
	Line(const Coordinate::Polar &polar);

	const ClassifiedLineSegment to_line_segment() const;

private:
	bool operator<(const Line &l)
	{
		return this->polar.theta < l.polar.theta;
	}
};
