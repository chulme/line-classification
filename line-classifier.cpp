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
		Radians theta = deg_to_radians(180.0 - line.polar.theta);

		if (std::sin(theta) == 0) {
			cv::Point pt1(line.polar.r, line.polar.r);
			cv::Point pt2(0, image.width);
			cv::line(cv_img, pt1, pt2, cv::Scalar(0, 0, 255));
		}
		else {
			cv::Point pt1(0, line.polar.r / std::sin(theta));
			cv::Point pt2(image.width, (line.polar.r - image.width * std::cos(theta)) / std::sin(theta));
			cv::line(cv_img, pt1, pt2, cv::Scalar(0, 0, 255));
		}
	}
	cv::imshow("Hough Lines (pre-pruning)", cv_img);
}