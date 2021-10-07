#include <hough.h>

std::vector<std::vector<double>> Hough::create_hough_transform(const Image& img)
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

std::vector<Line> Hough::get_hough_lines(const std::vector<std::vector<double>>& hough_transform, const double threshold) const
{
	std::vector<Line> hough_lines;
	for (size_t i = 0; i < hough_transform.size(); i++)
		for (size_t j = 0; j < hough_transform.front().size(); j++)
			if (hough_transform[i][j] > threshold)
				hough_lines.push_back(Line(Coordinate::Polar(static_cast<double>(i), Degrees(static_cast<double>(j)))));
	prune_lines(hough_lines);
	return hough_lines;
}

std::vector<Coordinate::Cartesian> Hough::find_valid_sample_indices(const Image& image)
{
	std::vector<Coordinate::Cartesian> valid_coordinates;
	std::vector<std::vector<int>> v(image.height, std::vector<int>(image.width));

	for (size_t i = 0; i < image.samples.size(); i++)
		if (image.samples[i] != 0.0)
			valid_coordinates.push_back(image.index_to_coordinate(i));

	return valid_coordinates;
}

double Hough::find_max_element(const std::vector<std::vector<double>>& two_dim_vec) const
{
	double largest_value = 0.0;
	for (const std::vector<double>& inner_vec : two_dim_vec)
		for (const double element : inner_vec)
			if (largest_value < element)
				largest_value = element;

	return largest_value;
}

void Hough::prune_lines(std::vector<Line>& lines) const
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
				++it2;
			}
		}
	}
}

bool Hough::is_similar(const Line& line_a, const Line& line_b) const
{
	bool similarAngle = (angle_difference_d(line_a.polar.theta, line_b.polar.theta) < 30);
	bool similarR = (std::abs(line_a.polar.r - line_b.polar.r) < 15);
	return (similarAngle && similarR);
}