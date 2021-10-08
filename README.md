# line-classification
Project to extract and classify lines from an image of a sports court.

## Algorithm
### Hough Transform
The Hough Transform is used to extract harsh lines from an image, where transforming an image in the traditional x-y domain, to the hough domain with theta-p axises, as shown below.
...hough domain
The following is the image's transform in the hough domain, after it has been binarised.
...hough transform

Note the 7 'bright' spots on the image, these indicate that there exists 7 lines in the original image!
## Hough Lines
Lines are extracted from the hough transform, by finding hough domain samples greater than the threshold, and returning the associated theta-r values (the axis in which the hough domain is framed). From this, many lines are drawn per actual line. As increasing the threshold results in some lines no longer being detected correctly, pruning of these lines occurs, by comparing each line against each other, and if similar lines are found, an average is taken and 1 is removed, which results in 1 line per cluster. It is worth noting that This is not an ideal solution, as an outlier in a given cluster will impact the averaging. This could be improved by employing outlier rejection before averaging.

### Classification
Classification at a high-level, is achived by the following steps:
1) Determine intersections between horizontal and vertical hough lines.
2) Classify horizontal lines and determine the actual start-end points of these lines.
3) Classify vertical lines, using the classified horizontal lines and their associated start-end points.

The horizontal lines are classified first, as both the service and base line have differing number of intersections. Whereas the vertical lines each have 1-2 intersections which is not sufficient for classification.



#### Horizontal Line Classification
To classify the service and base lines, the actual number of intersections needs to be calculated, as hough lines will intersect with every perpendicular line. This encountered an unexpected challenge, whereby because of an outlier in the hough transform, the pruning average process causes the final line to be skewed off the original image's line. As a result, taking the centre coordinate between 2 sections and inspecting the pixel data is not sufficient, and instead a ROI must be scanned in order to compensate for inaccuracy.
...Visualise ROI scan

The steps for horizontal classification is visualised below:

#### Vertical Line Classification
The steps for vertical line classification is visualised below:
