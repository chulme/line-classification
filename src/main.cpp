#include <structs.h>
#include <hough.h>
#include <line-classifier.h>
#include <opencv2/opencv.hpp>
#include <fstream>

// Provided Image Details
constexpr int32_t image_width = 1392, image_height = 550;
constexpr std::string_view image_path = "res/image.raw";

void binarize(Image& img, const uint32_t threshold)
{
	for (uint8_t& sample : img.samples)
		sample = (sample > threshold) ? 255 : 0;
}

void write_lines_to_csv(const std::vector<ClassifiedLineSegment>& lines)
{
	std::ofstream myfile;
	myfile.open("results.csv");
	myfile << "Line," << "X" << "," << "Y" << "," << "X" << "," << "Y" << ",\n";

	for (const ClassifiedLineSegment& line : lines)
	{
		switch (line.line_class)
		{
		case LineClasses::INNER_BASE_LINE:
			myfile << "Base Line," << line.origin.x << "," << line.origin.y << "," << line.destination.x << "," << line.destination.y << ",\n";
			break;
		case LineClasses::SERVICE_LINE:
			myfile << "Service Line," << line.origin.x << "," << line.origin.y << "," << line.destination.x << "," << line.destination.y << ",\n";
			break;
		case LineClasses::CENTRE_SERVICE_LINE:
			myfile << "Centre Service Line," << line.origin.x << "," << line.origin.y << "," << line.destination.x << "," << line.destination.y << ",\n";

			break;
		case LineClasses::DOUBLES_SIDELINE:
			myfile << "Doubles Side Line," << line.origin.x << "," << line.origin.y << "," << line.destination.x << "," << line.destination.y << ",\n";
			break;
		case LineClasses::SINGLES_SIDELINE:
			myfile << "Singles Side Line," << line.origin.x << "," << line.origin.y << "," << line.destination.x << "," << line.destination.y << ",\n";
			break;
		default:
			break;
		}
	}
	myfile.close();
}

int main()
{
	Image img("res/image.raw", image_width, image_height);

	binarize(img, 150);

	Hough hough_transformer;
	auto hough_transform = hough_transformer.create_hough_transform(img, true);
	auto hough_lines = hough_transformer.get_hough_lines(img, hough_transform, 200, true);

	LineClassifier classifier;
	std::vector<ClassifiedLineSegment> lines = classifier.classify_lines(img, hough_lines,true);

	write_lines_to_csv(lines);
	cv::waitKey();
}