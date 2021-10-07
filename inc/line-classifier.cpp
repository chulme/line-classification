#include "line-classifier.h"
#include <cmath>
#include <algorithm>
#include <array>
#include <opencv2/opencv.hpp>
#include <numbers>
#include <fstream>
#include <structs.h>

std::vector<Line> LineClassifier::classify_lines(const Image& image, std::vector<Line> hough_lines, const bool debug)
{
	std::unordered_map<Line, std::vector<Coordinate::Cartesian>, container_hash, container_equal> intersections = get_intersections(hough_lines, image);
	remove_false_horz_line_intersections(intersections, image);

	std::vector<LineSegment> classified_lines = classify_horz_lines(intersections);
	classified_lines = classify_vert_lines(intersections, classified_lines);

	cv::Mat debug_image = image.convert_to_mat();
	cv::cvtColor(debug_image, debug_image, cv::COLOR_GRAY2BGR);

	if (debug)
	{
		for (auto& it : intersections)
			for (auto intersection : it.second)
				cv::drawMarker(debug_image, cv::Point(intersection.x, intersection.y), cv::Scalar(0, 255, 0), 0, 20, 8);

		cv::imshow("Classified", debug_image);
	}
	std::vector<Coordinate::Cartesian> intersection_coords(intersections.size());
	for (auto i : intersections)
		intersection_coords.insert(intersection_coords.begin(), i.second.begin(), i.second.end());
	show_classified_lines(classified_lines, image);
	cv::waitKey();

	return std::vector<Line>();
}

std::unordered_map<Line, std::vector<Coordinate::Cartesian>, container_hash, container_equal> LineClassifier::get_intersections(const std::vector<Line>& lines, const Image& image)
{
	std::unordered_map<Line, std::vector<Coordinate::Cartesian>, container_hash, container_equal> coords;

	std::vector<Line> vertical_lines;
	std::vector<Line> horizontal_lines;

	//classify horz and vertical
	for (const Line& line : lines)
	{
		if (line.is_vertical())
			vertical_lines.push_back(line);
		else
			horizontal_lines.push_back(line);
	}

	for (const Line& horz_line : horizontal_lines)
	{
		std::vector<Coordinate::Cartesian> intersections_of_line;
		for (const Line& vert_line : vertical_lines)
		{
			intersections_of_line.push_back(get_intersection(horz_line, vert_line));
		}
		coords.insert({ horz_line, intersections_of_line });
	}

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

Coordinate::Cartesian LineClassifier::get_intersection(const Line& lineA, const Line& lineB)
{
	Radians thetaA = deg_to_radians(lineA.polar.theta);
	Radians thetaB = deg_to_radians(lineB.polar.theta);

	double ct1 = std::cos(thetaA);
	double st1 = std::sin(thetaA);
	double ct2 = std::cos(thetaB);
	double st2 = std::sin(thetaB);
	double d = ct1 * st2 - st1 * ct2;
	return {
		static_cast<int64_t>(std::abs((st2 * lineA.polar.r - st1 * lineB.polar.r) / d)),
		static_cast<int64_t>(std::abs((-ct2 * lineA.polar.r + ct1 * lineB.polar.r) / d)) };
}

void LineClassifier::remove_false_horz_line_intersections(std::unordered_map<Line, std::vector<Coordinate::Cartesian>, container_hash, container_equal>& intersections, const Image& image)
{
	for (auto it = intersections.begin(); it != intersections.end(); it++)
	{
		if (!it->first.is_vertical())
		{
			if (it->second.size() >= 5)
			{
				Coordinate::Cartesian avg_l = { (it->second[0] + it->second[1]) / 2 };
				Coordinate::Cartesian avg_r = { (it->second[4] + it->second[3]) / 2 };
				if (!image.does_block_contain_samples(image.coordinate_to_index(avg_l), 20, 50))
				{
					it->second.erase(it->second.begin());
				}
				if (!image.does_block_contain_samples(image.coordinate_to_index(avg_r), 20, 50))
				{
					it->second.pop_back();
				}
			}
		}
	}
}

std::vector<LineSegment> LineClassifier::classify_horz_lines(const std::unordered_map<Line, std::vector<Coordinate::Cartesian>, container_hash, container_equal>& intersections)
{
	std::vector<LineSegment> classified_lines;
	for (auto& it : intersections)
	{
		// Horizontal Line Classification
		if (!it.first.is_vertical()) {
			if (it.second.size() == 5)
			{
				LineSegment seg(LineClasses::INNER_BASE_LINE, it.second.begin()[1], it.second.end()[-2]);
				classified_lines.push_back(seg);

				LineSegment seg2(LineClasses::BASE_LINE, it.second.front(), it.second.back());
				classified_lines.push_back(seg2);

				LineSegment seg3(LineClasses::INNER_BASE_HALF_LINE, it.second.begin()[1], it.second.end()[-3]);
				classified_lines.push_back(seg3);
			}
			else if (it.second.size() == 3)
			{
				LineSegment seg(LineClasses::SERVICE_LINE, it.second.front(), it.second.back());
				classified_lines.push_back(seg);

				LineSegment seg2(LineClasses::SERVICE_LINE_HALF, it.second.begin()[1], it.second.end()[-3]);
				classified_lines.push_back(seg2);
			}
		}
		else // Vertical Line Classification
		{
		}
	}

	return classified_lines;
}

std::vector<LineSegment> LineClassifier::classify_vert_lines(const std::unordered_map<Line, std::vector<Coordinate::Cartesian>, container_hash, container_equal>& intersections, const std::vector<LineSegment>& horz_lines) {
	std::vector<LineSegment> classified_lines = horz_lines;

	/*
	LineSegment inner_base_line;
	for (const auto& it : intersections) {
		if (!it.first.is_vertical()) {
			if (it.second.size() == 5)
			{
				LineSegment seg(LineClasses::BASE_LINE, it.second.front(), it.second.back());
				classified_lines.push_back(seg);
			}
		}
	}
	*/
	LineSegment service_line = get_target_line(horz_lines, LineClasses::SERVICE_LINE);
	LineSegment service_line_half = get_target_line(horz_lines, LineClasses::SERVICE_LINE_HALF);
	LineSegment base_line_half = get_target_line(horz_lines, LineClasses::INNER_BASE_HALF_LINE);
	LineSegment base_line_outter = get_target_line(horz_lines, LineClasses::BASE_LINE);
	LineSegment base_line = get_target_line(horz_lines, LineClasses::INNER_BASE_LINE);

	for (auto& it : intersections) {
		if (it.first.is_vertical()) {
			for (Coordinate::Cartesian intersection : it.second) {
				// 2 Singles Sideline
				if (intersection == service_line.origin) {
					double m = static_cast<double>(service_line.origin.y - base_line.origin.y) / static_cast<double>(service_line.origin.x - base_line.origin.x);
					double c = static_cast<double>(base_line.origin.y) - static_cast<double>(m * base_line.origin.x);
					double x = (0 - c) / m;
					classified_lines.push_back({ LineClasses::SINGLES_SIDELINE, base_line.origin,Coordinate::Cartesian(std::abs(x),0) });
				}
				else if (intersection == service_line.destination) {
					double m = static_cast<double>(service_line.destination.y - base_line.destination.y) / static_cast<double>(service_line.destination.x - base_line.destination.x);
					double c = static_cast<double>(base_line.destination.y) - static_cast<double>(m * base_line.destination.x);
					double x = (0 - c) / m;
					classified_lines.push_back({ LineClasses::SINGLES_SIDELINE, base_line.destination,Coordinate::Cartesian(std::abs(x),0) });
				} // Centre Service Line
				else if (intersection == service_line_half.origin) {
					double m = static_cast<double>(service_line_half.origin.y - base_line_half.destination.y) / static_cast<double>(service_line_half.origin.x - base_line_half.destination.x);
					double c = static_cast<double>(base_line_half.destination.y) - static_cast<double>(m * base_line_half.destination.x);
					double x = (0 - c) / m;
					classified_lines.push_back({ LineClasses::CENTRE_SERVICE_LINE, service_line_half.origin,Coordinate::Cartesian(std::abs(x),0) });
				} //Doubles Side Line
				else if (intersection == base_line_outter.origin) {
					double m = static_cast<double>(service_line.origin.y - base_line.origin.y) / static_cast<double>(service_line.origin.x - base_line.origin.x);
					double c = static_cast<double>(base_line.origin.y) - static_cast<double>(m * base_line_outter.origin.x);
					double x = (0 - c) / m;
					classified_lines.push_back({ LineClasses::DOUBLES_SIDELINE, base_line_outter.origin,Coordinate::Cartesian(std::abs(x),0) });
				}
				else if (intersection == base_line_outter.destination) {
					double m = static_cast<double>(service_line.destination.y - base_line.destination.y) / static_cast<double>(service_line.destination.x - base_line.destination.x);
					double c = static_cast<double>(base_line_outter.destination.y) - static_cast<double>(m * base_line_outter.destination.x);
					double x = (0 - c) / m;
					classified_lines.push_back({ LineClasses::DOUBLES_SIDELINE, base_line_outter.destination,Coordinate::Cartesian(std::abs(x),0) });
				}
			}
		}
	}
	return classified_lines;
}

LineSegment LineClassifier::get_target_line(const std::vector<LineSegment>& lines, const LineClasses target_class) const {
	for (const LineSegment& line : lines)
		if (line.line_class == target_class)
			return line;
	return LineSegment({ 0,0 }, { 0,0 });
}

void LineClassifier::show_hough_transform(const std::vector<std::vector<double>>& hough_transform) const
{
	cv::Mat cv_image(hough_transform.front().size(), hough_transform.size(), CV_8UC3, cv::Scalar(0, 0, 0));
	for (size_t i = 0; i < hough_transform.size(); i++)
		for (size_t j = 0; j < hough_transform.front().size(); j++)
			cv::drawMarker(cv_image, cv::Point(i, j), cv::Scalar(0, 0, hough_transform[i][j] * 3), 4, 1, 1);
	cv::imshow("Hough Transform", cv_image);
}

void LineClassifier::show_hough_lines(const std::vector<Line>& hough_lines, const Image& image) const
{
	cv::Mat cv_img = image.convert_to_mat();
	cv::cvtColor(cv_img, cv_img, cv::COLOR_GRAY2BGR);

	for (const Line& line : hough_lines)
	{
		LineSegment line_seg = line.to_line_segment();
		cv::line(cv_img, cv::Point(line_seg.origin.x, line_seg.origin.y), cv::Point(line_seg.destination.x, line_seg.destination.y), cv::Scalar(0, 0, 255), 5);
	}
	cv::imshow("Hough Lines", cv_img);
}

void LineClassifier::show_classified_lines(const std::vector<LineSegment>& lines, const Image& image) const
{
	cv::Mat cv = image.convert_to_mat();
	cv::cvtColor(cv, cv, cv::COLOR_GRAY2BGR);
	constexpr int8_t text_line_offset = -10;
	constexpr int8_t text_new_line_offset = -10;

	for (const LineSegment& line : lines)
	{
		cv::drawMarker(cv, cv::Point(line.origin.x, line.origin.y), cv::Scalar(0, 255, 0), 0, 20, 8);
		cv::drawMarker(cv, cv::Point(line.destination.x, line.destination.y), cv::Scalar(0, 255, 0), 0, 20, 8);

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