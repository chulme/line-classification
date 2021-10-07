#include "line-classifier.h"
#include <cmath>
#include <algorithm>
#include <array>
#include <opencv2/opencv.hpp>
#include <numbers>
#include <fstream>
#include <structs.h>

/**
 * @brief Classifies lines of a tennis court using hough lines.
 * @details This is achieved in the following steps
 * 	1) Determine intersections between horizontal and vertical hough lines
 * 	2) Classify horizontal lines and determine the actual start-end points of these lines.
 * 	3) Classify vertical lines, using the classified horizontal lines and their associated start-end points
 * @param[in] image - The image of which lines are being classified.
 * @param[in] hough_lines - The hough lines which represent clear lines in the image.
 * @param[in] debug - Optional argument to enable visualisation of preprocessed data.
 * @return Vector of line segments, which contain a classification.
 */
std::vector<Line> LineClassifier::classify_lines(const Image& image, std::vector<Line> hough_lines, const bool debug)
{
	auto line_intersections_map = get_intersections(hough_lines);
	remove_false_horz_line_intersections(line_intersections_map, image);

	std::vector<LineSegment> classified_lines = classify_horz_lines(line_intersections_map);
	classified_lines = classify_vert_lines(line_intersections_map, classified_lines);

	cv::Mat debug_image = image.convert_to_mat();
	cv::cvtColor(debug_image, debug_image, cv::COLOR_GRAY2BGR);

	if (debug)
	{
		for (auto& it : line_intersections_map)
			for (auto intersection : it.second)
				cv::drawMarker(debug_image, cv::Point(intersection.x, intersection.y), cv::Scalar(0, 255, 0), 0, 20, 8);

		cv::imshow("Classified", debug_image);
	}
	std::vector<Coordinate::Cartesian> intersection_coords(line_intersections_map.size());
	for (auto i : line_intersections_map)
		intersection_coords.insert(intersection_coords.begin(), i.second.begin(), i.second.end());
	show_classified_lines(classified_lines, image);
	cv::waitKey();

	return std::vector<Line>();
}

/**
 * @brief Calculates intersections between horizontal and vertical hough lines.
 * @see LineClassifier::get_intersection()
 * @param[in] lines - Vector of lines to calculate intersections.
 * @return Map of lines to their associated intersections, represented as cartesian coordinates.
 */
std::unordered_map<Line, std::vector<Coordinate::Cartesian>, container_hash, container_equal> LineClassifier::get_intersections(const std::vector<Line>& lines)
{
	std::unordered_map<Line, std::vector<Coordinate::Cartesian>, container_hash, container_equal> coords;

	std::vector<Line> vertical_lines;
	std::vector<Line> horizontal_lines;

	//Seperate horizontal and vertical lines
	for (const Line& line : lines)
		if (line.is_vertical())
			vertical_lines.push_back(line);
		else
			horizontal_lines.push_back(line);

	// Determine all intersections for all horizontal lines
	for (const Line& horz_line : horizontal_lines)
	{
		std::vector<Coordinate::Cartesian> intersections_of_line;
		for (const Line& vert_line : vertical_lines)
		{
			intersections_of_line.push_back(get_intersection(horz_line, vert_line));
		}
		coords.insert({ horz_line, intersections_of_line });
	}

	// Determine all intersections for all vertical lines
	for (const Line& vert_line : vertical_lines)
	{
		std::vector<Coordinate::Cartesian> intersections_of_line;
		for (const Line& horz_line : horizontal_lines)
		{
			intersections_of_line.push_back(get_intersection(vert_line, horz_line));
		}
		coords.insert({ vert_line, intersections_of_line });
	}
	return coords;
}

/**
 * @brief Calculates the intersection point of 2 polar form lines.
 * @param[in] lineA - First line to find intersection for.
 * @param[in] lineB - Second line to find intersection for
 * @return Intersection coordinate
 */
Coordinate::Cartesian LineClassifier::get_intersection(const Line& lineA, const Line& lineB)
{
	Radians thetaA = deg_to_radians(lineA.polar.theta);
	Radians thetaB = deg_to_radians(lineB.polar.theta);

	double ct1 = std::cos(thetaA);
	double st1 = std::sin(thetaA);
	double ct2 = std::cos(thetaB);
	double st2 = std::sin(thetaB);
	double d = ct1 * st2 - st1 * ct2;

	return Coordinate::Cartesian(
		static_cast<int64_t>(std::abs((st2 * lineA.polar.r - st1 * lineB.polar.r) / d)),
		static_cast<int64_t>(std::abs((-ct2 * lineA.polar.r + ct1 * lineB.polar.r) / d)));
}

/**
 * @brief Removes false intersections of the service line.
 * @details As hough lines do not inately contain start-end points, the service line create intersections in the court boundary.
 * The strategy for classification requires these to be removed, and only consider actual intersections.
 *
 * This is achieved by selecting the outer detections on both the LHS and RHS, an average of these two is determined, and a small ROI around the average
 * coordinate is placed on the image, and is checked for non-0 elements. If any non-0 elements are detected, then it implies the line continues.
 * @param[in,out] intersections - Map of lines to their intersections, where false intersections will be removed from.
 * @param[in] image - Base image used to determine if a given line continues or not at each intersection. If the line does not continue,
 * a false intersection has occured.
 */
void LineClassifier::remove_false_horz_line_intersections(std::unordered_map<Line, std::vector<Coordinate::Cartesian>, container_hash, container_equal>& intersections, const Image& image)
{
	for (auto it = intersections.begin(); it != intersections.end(); it++)
	{
		if (!it->first.is_vertical())
		{
			if (it->second.size() >= NUMBER_OF_HOUGH_INTERSECTIONS_FOR_HORZ_LINES)
			{
				Coordinate::Cartesian avg_l = (it->second[0] + it->second[1]) / 2;
				Coordinate::Cartesian avg_r = (it->second[4] + it->second[3]) / 2;

				if (!image.does_block_contain_samples(image.coordinate_to_index(avg_l), 20, 50))
					it->second.erase(it->second.begin());

				if (!image.does_block_contain_samples(image.coordinate_to_index(avg_r), 20, 50))
					it->second.pop_back();
			}
		}
	}
}

/**
 * @brief Classifies the base and service lines of the tennis court.
 * @details This is achieved by checking the number of intersections of each line. The base line intersections with 2 single sidelines and 2 doubles sidelines,
 * while the service line intersects with 2 single sidelines and the centre service line.
 * @note Smaller segments of the lines are returned, as these will be used in the classification and determination of start-end points of vertical lines.
 * @param[in,out] intersections - Map of lines to associated intersections. The horizontal (service and base) lines will be removed to improve performance
 * (albeit not significantly) when classifiying vertical lines.
 * @return Vector of line segments, containing start-end points and a classification.
 */
std::vector<LineSegment> LineClassifier::classify_horz_lines(std::unordered_map<Line, std::vector<Coordinate::Cartesian>, container_hash, container_equal>& intersections)
{
	std::vector<LineSegment> classified_lines;
	std::vector<Line> horizontal_lines;
	for (auto& it : intersections)
	{
		if (!it.first.is_vertical())
		{
			if (it.second.size() == NUMBER_OF_INTERSECTIONS_FOR_BASE_LINE)
			{
				LineSegment inner_seg(LineClasses::INNER_BASE_LINE, it.second.begin()[1], it.second.end()[-2]);
				classified_lines.push_back(inner_seg);

				// Smaller segments of base line used in classification of vertical lines
				LineSegment full_seg(LineClasses::BASE_LINE, it.second.front(), it.second.back());
				classified_lines.push_back(full_seg);

				LineSegment half_seg(LineClasses::INNER_BASE_HALF_LINE, it.second.begin()[1], it.second.end()[-3]);
				classified_lines.push_back(half_seg);

				horizontal_lines.push_back(it.first);
			}
			else if (it.second.size() == NUMBER_OF_INTERSECTIONS_FOR_SERVICE_LINE)
			{
				LineSegment seg(LineClasses::SERVICE_LINE, it.second.front(), it.second.back());
				classified_lines.push_back(seg);

				// Smaller segment of service line used in classification of vertical lines
				LineSegment seg2(LineClasses::SERVICE_LINE_HALF, it.second.begin()[1], it.second.end()[-3]);
				classified_lines.push_back(seg2);

				horizontal_lines.push_back(it.first);
			}
		}
	}

	// Delete all horizontal lines frop the map
	for (auto i : horizontal_lines)
		intersections.erase(i);

	return classified_lines;
}

/**
 * @brief Classifies the centre service line, singles sidelines, and doubles sidelines.
 * @details This is achieved by inspecting each vertical line's intersections.
 * 	- Centre Service Line - intersects the service line
 *  - Singles Side Line - intersects the inner base line
 * 	- Doubles Side Line - intersects the outter base line.
 *
 * The start-end points is then calculated, using the base/service line as the origin when appropriate, and the where
 * the line will reach the top of the image.
 * @param[in] vertical_intersections - Map of vertical lines associated with their intersections.
 * @param[in] horz_lines - Vector of classified horizontal lines, used for classification of horizontal lines.
 * @return All classified lines segments.
 */
std::vector<LineSegment> LineClassifier::classify_vert_lines(const std::unordered_map<Line, std::vector<Coordinate::Cartesian>, container_hash, container_equal>& vertical_intersections, const std::vector<LineSegment>& horz_lines)
{
	std::vector<LineSegment> classified_lines = horz_lines;

	LineSegment service_line = get_target_line(horz_lines, LineClasses::SERVICE_LINE);
	LineSegment service_line_half = get_target_line(horz_lines, LineClasses::SERVICE_LINE_HALF);
	LineSegment base_line_half = get_target_line(horz_lines, LineClasses::INNER_BASE_HALF_LINE);
	LineSegment base_line_outter = get_target_line(horz_lines, LineClasses::BASE_LINE);
	LineSegment base_line = get_target_line(horz_lines, LineClasses::INNER_BASE_LINE);

	for (auto& it : vertical_intersections)
	{
		for (Coordinate::Cartesian intersection : it.second)
		{
			// 2 Singles Sideline
			if (intersection == service_line.origin)
			{
				double m = static_cast<double>(service_line.origin.y - base_line.origin.y) / static_cast<double>(service_line.origin.x - base_line.origin.x);
				double c = static_cast<double>(base_line.origin.y) - static_cast<double>(m * base_line.origin.x);
				double x = (0 - c) / m;
				classified_lines.push_back({ LineClasses::SINGLES_SIDELINE, base_line.origin, Coordinate::Cartesian(std::abs(x), 0) });
			}
			else if (intersection == service_line.destination)
			{
				double m = static_cast<double>(service_line.destination.y - base_line.destination.y) / static_cast<double>(service_line.destination.x - base_line.destination.x);
				double c = static_cast<double>(base_line.destination.y) - static_cast<double>(m * base_line.destination.x);
				double x = (0 - c) / m;
				classified_lines.push_back({ LineClasses::SINGLES_SIDELINE, base_line.destination, Coordinate::Cartesian(std::abs(x), 0) });
			} //Doubles Side Line
			else if (intersection == base_line_outter.origin)
			{
				double m = static_cast<double>(service_line.origin.y - base_line.origin.y) / static_cast<double>(service_line.origin.x - base_line.origin.x);
				double c = static_cast<double>(base_line.origin.y) - static_cast<double>(m * base_line_outter.origin.x);
				double x = (0 - c) / m;
				classified_lines.push_back({ LineClasses::DOUBLES_SIDELINE, base_line_outter.origin, Coordinate::Cartesian(std::abs(x), 0) });
			}
			else if (intersection == base_line_outter.destination)
			{
				double m = static_cast<double>(service_line.destination.y - base_line.destination.y) / static_cast<double>(service_line.destination.x - base_line.destination.x);
				double c = static_cast<double>(base_line_outter.destination.y) - static_cast<double>(m * base_line_outter.destination.x);
				double x = (0 - c) / m;
				classified_lines.push_back({ LineClasses::DOUBLES_SIDELINE, base_line_outter.destination, Coordinate::Cartesian(std::abs(x), 0) });
			} // Centre Service Line
			else if (intersection == service_line_half.origin)
			{
				double m = static_cast<double>(service_line_half.origin.y - base_line_half.destination.y) / static_cast<double>(service_line_half.origin.x - base_line_half.destination.x);
				double c = static_cast<double>(base_line_half.destination.y) - static_cast<double>(m * base_line_half.destination.x);
				double x = (0 - c) / m;
				classified_lines.push_back({ LineClasses::CENTRE_SERVICE_LINE, service_line_half.origin, Coordinate::Cartesian(std::abs(x), 0) });
			}
		}
	}
	return classified_lines;
}

/**
 * @brief Find a target classified line segments in the vector of horizontal lines.
 * @param[in] lines- Vector of horizontal lines.
 * @param[in] target_class - The target line classification to find.
 * @return The target line segment.
 */
LineSegment LineClassifier::get_target_line(const std::vector<LineSegment>& lines, const LineClasses target_class) const
{
	for (const LineSegment& line : lines)
		if (line.line_class == target_class)
			return line;
	return LineSegment({ 0, 0 }, { 0, 0 });
}

/**
 * @brief Displays classified lines using OpenCV.
 * @note The OpenCV function WaitKey is required after this function to properly display the image.
 * @param[in] lines - Classified lines to draw.
 * @param[in] image - Image to underlay lines on top of.
 */
void LineClassifier::show_classified_lines(const std::vector<LineSegment>& lines, const Image& image) const
{
	cv::Mat cv = image.convert_to_mat();
	cv::cvtColor(cv, cv, cv::COLOR_GRAY2BGR);
	constexpr int8_t text_line_offset = -10;
	constexpr int8_t text_new_line_offset = -10;

	for (const LineSegment& line : lines)
	{
		cv::line(cv, cv::Point(line.origin.x, line.origin.y), cv::Point(line.destination.x, line.destination.y), cv::Scalar(0, 255, 0), 5);
		Coordinate::Cartesian average_coord = (line.origin + line.destination) / 2;
		switch (line.line_class)
		{
		case LineClasses::INNER_BASE_LINE:
			cv::putText(cv, "Base Line", cv::Point(average_coord.x, average_coord.y + text_line_offset), 0, 1.0, cv::Scalar(255, 255, 255), 2);
			break;
		case LineClasses::SERVICE_LINE:
			cv::putText(cv, "Service Line", cv::Point(average_coord.x, average_coord.y + text_line_offset), 0, 1.0, cv::Scalar(255, 255, 255), 2);
			break;
		case LineClasses::CENTRE_SERVICE_LINE:
			cv::putText(cv, "Centre Service Line", cv::Point(average_coord.x, average_coord.y + text_line_offset), 0, 1.0, cv::Scalar(255, 255, 255), 2);
			break;
		case LineClasses::DOUBLES_SIDELINE:
			cv::putText(cv, "Dbls", cv::Point(average_coord.x + text_line_offset, average_coord.y), 0, 1.0, cv::Scalar(255, 255, 255), 2);
			break;
		case LineClasses::SINGLES_SIDELINE:
			cv::putText(cv, "Sgls", cv::Point(average_coord.x + text_line_offset, average_coord.y), 0, 1.0, cv::Scalar(255, 255, 255), 2);
			break;
		default:
			break;
		}
	}
	cv::imshow("Classified Lines", cv);
};