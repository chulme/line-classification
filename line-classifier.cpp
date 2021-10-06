#include "line-classifier.h"
#include <cmath>
#include <algorithm>
#include <array>
#include <opencv2/opencv.hpp>
#include <numbers>
#include <fstream>
#include <structs.h>

std::vector<Line> LineClassifier::detect_lines(const Image& image, const uint64_t hough_threshold, const uint64_t line_threshold, const bool debug) {
	std::vector<std::vector<double>> hough_transform = create_hough_transform(image);
	std::vector<Line> hough_lines = get_hough_lines(hough_transform, hough_threshold);

	if (debug) {
		show_hough_transform(hough_transform);
		show_hough_lines(hough_lines, image);
	}
	// To-Do: filter and return
	prune_lines(hough_lines);
	get_intersections(hough_lines, image);

	return std::vector<Line>();
}

std::vector<std::vector<double>> LineClassifier::create_hough_transform(const Image& img)
{
	std::vector<Coordinate::Cartesian> coordinates = find_valid_sample_indices(img);

	std::vector<std::vector<double>> r(coordinates.size(), std::vector<double>(angles.size()));
	for (size_t i = 0; i < coordinates.size(); i++)
		for (size_t j = 0; j < angles.size(); j++)
			r[i][j] = coordinates[i].x * std::cos(angles[j]) + coordinates[i].y * std::sin(angles[j]);

	double max_r = find_max_element(r);
	std::vector<std::vector<double>> hough_transform(max_r + 1, std::vector<double>(angles.size()));
	for (size_t i = 0; i < coordinates.size(); i++)
		for (size_t j = 0; j < angles.size(); j++)
			if (r[i][j] >= 0.0f)
				hough_transform[r[i][j]][j]++;

	return hough_transform;
}

std::vector<Coordinate::Cartesian> LineClassifier::find_valid_sample_indices(const Image& image)
{
	std::vector<Coordinate::Cartesian> valid_coordinates;
	std::vector<std::vector<int>> v(image.height, std::vector<int>(image.width));

	for (size_t i = 0; i < image.samples.size(); i++)
		if (image.samples[i] != 0.0)
			valid_coordinates.push_back(image.index_to_coordinate(i));

	return valid_coordinates;
}

double LineClassifier::find_max_element(const std::vector<std::vector<double>>& two_dim_vec) const
{
	double largest_value = 0.0;
	for (const std::vector<double>& inner_vec : two_dim_vec)
		for (const double element : inner_vec)
			if (largest_value < element)
				largest_value = element;

	return largest_value;
}

std::vector<Line> LineClassifier::get_hough_lines(const std::vector<std::vector<double>>& hough_transform, const double threshold) const
{
	std::vector<Line> hough_lines;
	for (size_t i = 0; i < hough_transform.size(); i++)
		for (size_t j = 0; j < hough_transform.front().size(); j++)
			if (hough_transform[i][j] > threshold)
				hough_lines.push_back(Line(Coordinate::Polar(static_cast<double>(i), Degrees(static_cast<double>(j)))));

	return hough_lines;
}

void LineClassifier::prune_lines(std::vector<Line>& lines)
{
	for (auto it1 = lines.begin(); it1 != lines.end(); ++it1)
	{
		for (auto it2 = std::next(it1); it2 != lines.end(); )
		{
			if (is_similar(*it1, *it2))
			{
				*it1 = Line(Coordinate::Polar((it1->polar.r + it2->polar.r) / 2.0, (it1->polar.theta + it2->polar.theta) / 2.0));
				it2 = lines.erase(it2);
			}
			else { ++it2; }
		}
	}
}

bool LineClassifier::is_similar(const Line& line_a, const Line& line_b) {
	bool similarAngle = (angle_difference_d(line_a.polar.theta, line_b.polar.theta) < 30);
	bool similarR = (std::abs(line_a.polar.r - line_b.polar.r) < 15);
	return (similarAngle && similarR);
}

std::vector<Coordinate::Cartesian> LineClassifier::get_intersections(const std::vector<Line>& lines, const Image& image) {
	cv::Mat cv_img = image.convert_to_mat();
	cv::cvtColor(cv_img, cv_img, cv::COLOR_GRAY2BGR);
	Line l(Coordinate::Polar());

	//classify horz and vertical
	for (const Line& line : lines)
	{
		std::array<Coordinate::Cartesian, 2> coord_pair = line.to_line_segment();
		if (line.is_vertical()) {
			cv::line(cv_img, cv::Point(coord_pair[0].x, coord_pair[0].y), cv::Point(coord_pair[1].x, coord_pair[1].y), cv::Scalar(0, 0, 255), 5);
		}
		else {
			cv::line(cv_img, cv::Point(coord_pair[0].x, coord_pair[0].y), cv::Point(coord_pair[1].x, coord_pair[1].y), cv::Scalar(255, 0, 0), 5);
		}
	}

	std::vector<Coordinate::Cartesian> coords;
	for (size_t i = 0; i < lines.size(); i++) {
		for (size_t j = 0; j < lines.size(); j++) {
			if (i != j) {
				coords.push_back(get_intersection(lines[i], lines[j]));
			}
		}
	}

	for (auto coord : coords) {
		cv::drawMarker(cv_img, cv::Point(coord.x, coord.y), cv::Scalar(0, 255, 0), 0, 20, 8);
	}

	cv::imshow("classified lines", cv_img);
	cv::waitKey();
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
	if (d != 0) {
		return {
			static_cast<int64_t>(std::abs((st2 * lineA.polar.r - st1 * lineB.polar.r) / d)),
			static_cast<int64_t>(std::abs((-ct2 * lineA.polar.r + ct1 * lineB.polar.r) / d))
		};
	}
	else { //parallel
		return { 0,0 };
	}
}

void LineClassifier::show_hough_transform(const std::vector<std::vector<double>>& hough_transform) const {
	cv::Mat cv_image(hough_transform.front().size(), hough_transform.size(), CV_8UC3, cv::Scalar(0, 0, 0));
	for (size_t i = 0; i < hough_transform.size(); i++)
		for (size_t j = 0; j < hough_transform.front().size(); j++)
			cv::drawMarker(cv_image, cv::Point(i, j), cv::Scalar(0, 0, hough_transform[i][j] * 3), 4, 1, 1);
	cv::imshow("Hough Transform", cv_image);
}

void LineClassifier::show_hough_lines(const std::vector<Line>& hough_lines, const Image& image) const {
	cv::Mat cv_img = image.convert_to_mat();
	cv::cvtColor(cv_img, cv_img, cv::COLOR_GRAY2BGR);

	for (const Line& line : hough_lines)
	{
		std::array<Coordinate::Cartesian, 2> coord_pair = line.to_line_segment();
		cv::line(cv_img, cv::Point(coord_pair[0].x, coord_pair[0].y), cv::Point(coord_pair[1].x, coord_pair[1].y), cv::Scalar(0, 0, 255));
	}
	cv::imshow("Hough Lines (pre-pruning)", cv_img);
}