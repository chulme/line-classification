#include <hough.h>

/**
 * @brief Creates hough transform of a given image.
 * @param[in] img - Image to transform
 * @param[in] debug - Optional argument to enable visualisation of the transform.
 * @return Hough transform represented as 2D Vector.
 */
std::vector<std::vector<double>> Hough::create_hough_transform(const Image &img, const bool debug)
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
	if (debug)
		show_hough_transform(hough_transform);

	return hough_transform;
}

/**
 * @brief Extracts hough lines from an image, using a hough transform.
 * @param[in] img - Image to extract hough lines from
 * @param[in] hough_transform - The hough transformed image
 * @param[in] threshold - Optional argument that thresholds hough lines to be returned.
 * @param[in] debug - Optional argument to enable visualisation of the transform.
 * @return Hough lines of an image, which is a representation of harsh lines in the image.
 */
std::vector<Line> Hough::get_hough_lines(const Image &img, const std::vector<std::vector<double>> &hough_transform,
										 const double threshold, const bool debug) const
{
	std::vector<Line> hough_lines;
	for (size_t i = 0; i < hough_transform.size(); i++)
		for (size_t j = 0; j < hough_transform.front().size(); j++)
			if (hough_transform[i][j] > threshold)
				hough_lines.push_back(Line(Coordinate::Polar(static_cast<double>(i), Degrees(static_cast<double>(j)))));

	prune_lines(hough_lines);

	if (debug)
		show_hough_lines(hough_lines, img);

	return hough_lines;
}

/**
 * @brief Finds all non 0 (non black) samples indices of an image.
 * @param[in] img - Image to transform.
 * @return Cartesian coordinates of valid samples.
 */
std::vector<Coordinate::Cartesian> Hough::find_valid_sample_indices(const Image &image)
{
	std::vector<Coordinate::Cartesian> valid_coordinates;
	std::vector<std::vector<int>> v(image.height, std::vector<int>(image.width));

	for (size_t i = 0; i < image.samples.size(); i++)
		if (image.samples[i] != 0.0)
			valid_coordinates.push_back(image.index_to_coordinate(i));

	return valid_coordinates;
}

/**
 * @brief Finds maximum element in 2D vector.
 * @param[in] two_dim_vec - 2D Vector to find max element of.
 * @return Maximum element of the 2D vector.
 */
double Hough::find_max_element(const std::vector<std::vector<double>> &two_dim_vec) const
{
	double largest_value = 0.0;
	for (const std::vector<double> &inner_vec : two_dim_vec)
		for (const double element : inner_vec)
			if (largest_value < element)
				largest_value = element;

	return largest_value;
}

/**
 * @brief Removes lines until only a single line exists per cluster of lines
 * @details This is achieved by looping over all lines, and if determined to be similar, an average is taken and the line removed.
 * @note This is not an ideal solution, as an outlier in a given cluster will impact the averaging. This could be improved by employing
 * outlier rejection before averaging.
 * @param[in, out] lines - The lines to prune
 */
void Hough::prune_lines(std::vector<Line> &lines) const
{
	//average out lines
	for (auto it1 = lines.begin(); it1 != lines.end(); ++it1)
	{
		for (auto it2 = std::next(it1); it2 != lines.end();)
		{
			if (is_similar(*it1, *it2))
			{
				*it1 = Line(Coordinate::Polar((it1->polar.r + it2->polar.r) / 2.0, (it1->polar.theta + it2->polar.theta) / 2.0));
				it2 = lines.erase(it2);
			}
			else
			{
				it2++;
			}
		}
	}
}

/**
 * @brief Determines if 2 lines are similar.
 * @note This is not an ideal solution, and could likely be replaced with a more sophisticated calculation.
 * However, it does the job in the given image but will likely cause issues when faced with an image at a different rotation relative
 * to the court.
 * @param[in] line_a - First line to compare
 * @param[in] line_a - Second line to compare
 * @return Flag indicating if they are similar (true if similar, false if not).
 */
bool Hough::is_similar(const Line &line_a, const Line &line_b) const
{
	bool similarAngle = (angle_difference_d(line_a.polar.theta, line_b.polar.theta) < 30);
	bool similarR = (std::abs(line_a.polar.r - line_b.polar.r) < 15);
	return (similarAngle && similarR);
}

/**
 * @brief Displays hough transform using Open CV.
 * @note OpenCV's WaitKey function is required after in order to display.
 * @param[in] hough_transform - The hough transform to display
 */
void Hough::show_hough_transform(const std::vector<std::vector<double>> &hough_transform) const
{
	cv::Mat cv_image(hough_transform.front().size(), hough_transform.size(), CV_8UC3, cv::Scalar(0, 0, 0));
	for (size_t i = 0; i < hough_transform.size(); i++)
		for (size_t j = 0; j < hough_transform.front().size(); j++)
			cv::drawMarker(cv_image, cv::Point(i, j), cv::Scalar(0, 0, hough_transform[i][j] * 3), 4, 1, 1);
	cv::imshow("Hough Transform", cv_image);
}

/**
 * @brief Displays hough lines using Open CV.
 * @note OpenCV's WaitKey function is required after in order to display.
 * @param[in] hough_lines - The hough lines to display
 * @param[in] image - The image where lines will be drawn on top of.
 */
void Hough::show_hough_lines(const std::vector<Line> &hough_lines, const Image &image) const
{
	cv::Mat cv_img = image.convert_to_mat();
	cv::cvtColor(cv_img, cv_img, cv::COLOR_GRAY2BGR);

	for (const Line &line : hough_lines)
	{
		ClassifiedLineSegment line_seg = line.to_line_segment();
		cv::line(cv_img, cv::Point(line_seg.origin.x, line_seg.origin.y), cv::Point(line_seg.destination.x, line_seg.destination.y), cv::Scalar(0, 0, 255), 5);
	}
	cv::imshow("Hough Lines", cv_img);
}