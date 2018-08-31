#include "ardrone/ardrone.h"

using namespace cv;
using namespace std;

Mat image;

bool backprojMode = false;
bool selectObject = false;
int trackObject = 0;
bool showHist = true;
Point origin;
Rect selection;
int vmin = 10, vmax = 256, smin = 30;  //default values, can be changed with the sliders 

static void onMouse(int event, int x, int y, int, void*)
{
//	cout << "MOUSEEEE \n";  To check the flow of the code 
	if (selectObject)
	{
		selection.x = MIN(x, origin.x);
		selection.y = MIN(y, origin.y);
		selection.width = std::abs(x - origin.x);
		selection.height = std::abs(y - origin.y);
	//	cout << "selection \n"; To check the flow of the code
		selection &= Rect(0, 0, image.cols, image.rows);
	}

	switch (event)
	{
	case EVENT_LBUTTONDOWN:		// EVENT_LBUTTONDOWN = 1, 
		origin = Point(x, y);
		selection = Rect(x, y, 0, 0);
		selectObject = true;
	//	cout << "EVENT_LBUTTONDOWN \n";  To check the flow of the code
		break;
	case EVENT_LBUTTONUP:	//EVENT_LBUTTONUP = 4
		selectObject = false;
		if (selection.width > 0 && selection.height > 0)
			trackObject = -1;
	// cout << "EVENT_LBUTTONUP \n"; To check the flow of the code
		break;
	}
}

// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------
int main(int argc, char *argv[])
{
	// AR.Drone class
	ARDrone ardrone;

	/*CAMSHIFT */
	Rect trackWindow;
	int hsize = 16;
	float hranges[] = { 0,180 };
	const float* phranges = hranges;
		
	// Initialize
	if (!ardrone.open()) {
		std::cout << "Failed to initialize." << std::endl;
		return -1;
	}

   	// Instructions
	std::cout << "***************************************" << std::endl;
	std::cout << "*       CV Drone sample program       *" << std::endl;
	std::cout << "*           - How to Play -           *" << std::endl;
	std::cout << "***************************************" << std::endl;
	std::cout << "*                                     *" << std::endl;
	std::cout << "* - Controls -                        *" << std::endl;
	std::cout << "*    'S' -- Takeoff/Landing           *" << std::endl;
	std::cout << "*    'Up'    -- Move forward          *" << std::endl;
	std::cout << "*    'Down'  -- Move backward         *" << std::endl;
	std::cout << "*    'Left'  -- Turn left             *" << std::endl;
	std::cout << "*    'Right' -- Turn right            *" << std::endl;
	std::cout << "*    'Q'     -- Move upward           *" << std::endl;
	std::cout << "*    'A'     -- Move downward         *" << std::endl;
	std::cout << "*                                     *" << std::endl;
	std::cout << "* - Others -                          *" << std::endl;
	std::cout << "*    'T'     -- Track marker          *" << std::endl;
	std::cout << "*    'Esc'   -- Exit                  *" << std::endl;
	std::cout << "*                                     *" << std::endl;
	std::cout << "***************************************" << std::endl;

	// Thresholds
	int minH = 0, maxH = 255;
	int minS = 30, maxS = 255; /*CAMM*/
	int minV = 10, maxV = 255;

	// XML save data
	std::string filename("thresholds.xml");
	cv::FileStorage fs(filename, cv::FileStorage::READ);

	// If there is a save file then read it
	if (fs.isOpened())
	{
		minS = fs["S_MIN"];
		maxV = fs["V_MAX"];
		minV = fs["V_MIN"];
		fs.release();
	}

	// Create a window
	cv::createTrackbar("S min", "backproj", &minS, 255); // Create Trackbar to eliminate back ground noise
	cv::createTrackbar("V max", "backproj", &maxV, 255); // Create Trackbar to eliminate back ground noise
	cv::createTrackbar("V min", "backproj", &minV, 255); //Create Trackbar to eliminate back ground noise
	cv::resizeWindow("backproj", 0, 0);
	namedWindow("CamShift_object_tracking", 0);
	setMouseCallback("CamShift_object_tracking", onMouse, 0);		//set mouse handler for this window

														/*CAMM*/
	Mat frame, hsv, hue, mask, hist, histimg = Mat::zeros(200, 320, CV_8UC3), backproj;
	bool paused = false;
	Rect track_rect;
	// Main loop
	while (1) {
		// Key input

		int key = cv::waitKey(100);
		if (key == 0x1b)
			break;

		// Take off / Landing 
		if (key == 's') {
			if (ardrone.onGround())
				ardrone.takeoff();
			else
				ardrone.landing();


			// Move
			double vx = 0.0, vy = 0.0, vz = 0.0, vr = 0.0;
			if (key == 'i' || key == CV_VK_UP)    vx = 1.0;
			if (key == 'k' || key == CV_VK_DOWN)  vx = -1.0;
			if (key == 'u' || key == CV_VK_LEFT)  vr = 1.0;
			if (key == 'o' || key == CV_VK_RIGHT) vr = -1.0;
			if (key == 'j') vy = 1.0;
			if (key == 'l') vy = -1.0;
			if (key == 'q') vz = 1.0;
			if (key == 'a') vz = -1.0;

			// Change camera
			static int mode = 0;
			if (key == 'c') ardrone.setCamera(++mode % 4);

			// Switch tracking ON/OFF
			static int track = 0;
			if (key == 't')
				track = !track;

			if (key == 'p') paused = !paused;

			// Get an image
			image = ardrone.getImage();
			if (image.empty())
			{
				cout << "Image is empty" << endl;
				continue;
			}

			hsv = Mat::zeros(Size(image.cols, image.rows), image.type());
			cvtColor(image, hsv, COLOR_BGR2HSV);

			if (trackObject != 0)
			{
				int _vmin = minV, _vmax = maxV;

				/*get 'mask' - values to be discarded*/
				cv::inRange(hsv, Scalar(0, smin, MIN(_vmin, _vmax)), Scalar(180, 256, MAX(_vmin, _vmax)), mask);
				int ch[] = { 0, 0 };
				hue.create(hsv.size(), hsv.depth()); /*hue is of the same dimentions as hsv*/
				mixChannels(&hsv, 1, &hue, 1, ch, 1);  /*copy channels from hsv to hue*/

				if (trackObject < 0)
				{
					// Object has been selected by user, set up CAMShift search properties once
					Mat roi(hue, selection), maskroi(mask, selection);
					calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);
					normalize(hist, hist, 0, 255, NORM_MINMAX);

					trackWindow = selection;
					trackObject = 1; // Don't set up again, unless user selects new ROI

					histimg = Scalar::all(0);
					int binW = histimg.cols / hsize;
					Mat buf(1, hsize, CV_8UC3);
					for (int i = 0; i < hsize; i++)
						buf.at<Vec3b>(i) = Vec3b(saturate_cast<uchar>(i*180. / hsize), 255, 255);
					cvtColor(buf, buf, COLOR_HSV2BGR);

					for (int i = 0; i < hsize; i++)
					{
						int val = saturate_cast<int>(hist.at<float>(i)*histimg.rows / 255);
						rectangle(histimg, Point(i*binW, histimg.rows),
							Point((i + 1)*binW, histimg.rows - val),
							Scalar(buf.at<Vec3b>(i)), -1, 8);
					}
				}
			
				// Perform CAMShift
				calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
				backproj &= mask;
				RotatedRect trackBox = CamShift(backproj, trackWindow,
					TermCriteria(TermCriteria::EPS | TermCriteria::COUNT, 10, 1));
				if (trackWindow.area() <= 1)
				{
					int cols = backproj.cols, rows = backproj.rows, r = (MIN(cols, rows) + 5) / 6;
					trackWindow = Rect(trackWindow.x - r, trackWindow.y - r,
						trackWindow.x + r, trackWindow.y + r) &
						Rect(0, 0, cols, rows);
					track = 0; // Not tracking

				}

				if (backprojMode)
					cvtColor(backproj, image, COLOR_GRAY2BGR);
				track_rect = trackBox.boundingRect();
				rectangle(image, track_rect, Scalar(0, 255, 0));

			}


			if (selectObject && selection.width > 0 && selection.height > 0)
			{
				Mat roi(image, selection);
				bitwise_not(roi, roi);
			}


			/******** TRACKING  ********/

			if (!image.empty())
				cv::imshow("CamShift_object_tracking", image);
			// Detect contours
			std::vector<vector<Point>> contours;
			findContours(backproj.clone(), contours, RETR_CCOMP, CHAIN_APPROX_SIMPLE);

			// Find largest contour
			int contour_index = -1;
			double max_area = 0.0;
			for (size_t i = 0; i < contours.size(); i++) {
				double area = fabs(contourArea(contours[i]));
				if (area > max_area) {
					contour_index = i;
					max_area = area;
				}
			}


			// Object detected
			if (contour_index >= 0) {
				// Moments - used for, down, left, right
				/*this function can be improve to 3D function for forward and backward also,
				but I only use as 2D function for left, right, up and dowm                  */
				Moments moments = cv::moments(contours[contour_index], true);
				double marker_y = (int)(moments.m01 / moments.m00);
				double marker_x = (int)(moments.m10 / moments.m00);

				//get width and height of the rectangle
				int rows, cols;
				rows = track_rect.width;
				cols = track_rect.height;


				// Tracking
				if (track)
				{
					//if the object is larger than 1/2 of the iamge, then go backward with other movements.
					if (rows > (image.rows / 2) || cols > (image.cols / 2))
					{
						const double kp = 0.005;
						vx = 0.0;
						vy = 0.0;
						vz = kp * (backproj.rows / 2 - marker_y);
						vr = kp * (backproj.cols / 2 - marker_x);
						cout << "BACKWARD" << endl;
					}
					else if (rows < (image.rows / 6) || cols < (image.cols / 6))// if the object too small, go forward with higher speed 
					{
						const double kp = 0.005;
						if (rows > (image.rows / 12) || cols > (image.cols / 12))
						{
							vx = 0.5;
							vy = 0.0;
							vz = 0.0;
							vr = 0.0;
							cout << "FORWARD" << endl;
						}

						else if (rows < (image.rows / 12) || cols < (image.cols / 12))	//if lost object, track off
						{
							vx = 0.0;
							vy = 0.0;
							vz = 0.0;
							vr = 0.0;
							track = !track;
							cout << "NOT TRACKING" << endl;
						}
					}
					else 				// tracking movement other than condition above
					{
						const double kp = 0.005;

						vx = 0.1;
						vy = 0.0;
						vz = kp * (backproj.rows / 2 - marker_y);
						vr = kp * (backproj.cols / 2 - marker_x);
						cout << "NORMAL" << endl;
					}
				}

			}


			// Show result
			if (!backproj.empty())
				cv::imshow("backproj", backproj);


			cv::putText(image, (track) ? "track on" : "track off", cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, (track) ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
			cv::imshow("CamShift_object_tracking", image);

			ardrone.move3D(vx, vy, vz, vr);
			cout << "COORDINATES" << " " << vx << " " << vy << " " << vz << " " << vr << endl;
		}
	}

	// Save thresholds
	fs.open(filename, cv::FileStorage::WRITE);
	if (fs.isOpened()) {
		cv::write(fs, "H_MAX", maxH);
		cv::write(fs, "S_MIN", minS);
		cv::write(fs, "V_MIN", minV);
		fs.release();
	}

	// See you
	ardrone.close();

	return 0;
}