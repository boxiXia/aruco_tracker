/**
Copyright 2017 Rafael Mu�oz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
	  conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
	  of conditions and the following disclaimer in the documentation and/or other materials
	  provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Mu�oz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Mu�oz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Mu�oz Salinas.
*/

//#include "aruco/src/aruco.h"
//#include "aruco/src/cvdrawingutils.h"
#include "aruco/cvdrawingutils.h"

#include "aruco/aruco.h"
//#include "cvdrawingutils.h"
#include <fstream>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>
#include <string>
#include <stdexcept>
#include <clipp.h>

#if CV_MAJOR_VERSION >= 4
#define CV_CAP_PROP_FRAME_COUNT cv::CAP_PROP_FRAME_COUNT
#define CV_CAP_PROP_POS_FRAMES cv::CAP_PROP_POS_FRAMES
#endif
using namespace std;
using namespace cv;
using namespace aruco;

// Create the detector
MarkerDetector MDetector;
std::map<uint32_t, MarkerPoseTracker> MTracker;  // use a map so that for each id, we use a different pose tracker

VideoCapture TheVideoCapturer;
vector<Marker> TheMarkers;
Mat TheInputImage, TheInputImageGrey, TheInputImageCopy;
CameraParameters TheCameraParameters;
void cvTackBarEvents(int pos, void*);
string dictionaryString;
int iDetectMode = 0, iMinMarkerSize = 0, iCorrectionRate = 0, iShowAllCandidates = 0, iEnclosed = 0, iThreshold, iCornerMode, iDictionaryIndex, iTrack = 1;

int waitTime = 0;
bool showMennu = false, bPrintHelp = false, isVideo = false;


struct TimerAvrg
{
	std::vector<double> times;
	size_t curr = 0, n;
	std::chrono::high_resolution_clock::time_point begin, end;
	TimerAvrg(int _n = 30)
	{
		n = _n;
		times.reserve(n);
	}
	inline void start() { begin = std::chrono::high_resolution_clock::now(); }
	inline void stop()
	{
		end = std::chrono::high_resolution_clock::now();
		double duration = double(std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()) * 1e-6;
		if (times.size() < n)
			times.push_back(duration);
		else
		{
			times[curr] = duration;
			curr++;
			if (curr >= times.size())
				curr = 0;
		}
	}
	double getAvrg()
	{
		double sum = 0;
		for (auto t : times)
			sum += t;
		return sum / double(times.size());
	}
};

TimerAvrg Fps;

cv::Mat resize(const cv::Mat& in, cv::Size s)
{
	if (s.width == -1 || s.height == -1)
		return in;
	cv::Mat im2;
	cv::resize(in, im2, s);
	return im2;
}

cv::Mat resize(const cv::Mat& in, int width)
{
	if (in.size().width <= width)
		return in;
	float yf = float(width) / float(in.size().width);
	cv::Mat im2;
	cv::resize(in, im2, cv::Size(width, static_cast<int>(in.size().height * yf)));
	return im2;
}
cv::Mat resizeImage(cv::Mat& in, float resizeFactor)
{
	if (fabs(1 - resizeFactor) < 1e-3)
		return in;
	float nc = float(in.cols) * resizeFactor;
	float nr = float(in.rows) * resizeFactor;
	cv::Mat imres;
	cv::resize(in, imres, cv::Size(nc, nr));
	cout << "Imagesize=" << imres.size() << endl;
	return imres;
}
/************************************
*
*
*
*
************************************/
void setParamsFromGlobalVariables(aruco::MarkerDetector& md)
{

	md.setDetectionMode((DetectionMode)iDetectMode, float(iMinMarkerSize) / 1000.);
	md.getParameters().setCornerRefinementMethod((aruco::CornerRefinementMethod)iCornerMode);

	md.getParameters().detectEnclosedMarkers(iEnclosed);
	md.getParameters().ThresHold = iThreshold;
	md.getParameters().trackingMinDetections = (iTrack ? 3 : 0);
	if (aruco::Dictionary::getTypeFromString(md.getParameters().dictionary) != Dictionary::CUSTOM)
		md.setDictionary((aruco::Dictionary::DICT_TYPES)iDictionaryIndex, float(iCorrectionRate) / 10.); // sets the dictionary to be employed (ARUCO,APRILTAGS,ARTOOLKIT,etc)
}

void createMenu()
{
	cv::createTrackbar("Dictionary", "menu", &iDictionaryIndex, 13, cvTackBarEvents);
	cv::createTrackbar("DetectMode", "menu", &iDetectMode, 2, cvTackBarEvents);
	cv::createTrackbar("CornerMode", "menu", &iCornerMode, 2, cvTackBarEvents);
	cv::createTrackbar("Track", "menu", &iTrack, 1, cvTackBarEvents);

	cv::createTrackbar("MinMarkerSize", "menu", &iMinMarkerSize, 1000, cvTackBarEvents);
	cv::createTrackbar("Threshold", "menu", &iThreshold, 40, cvTackBarEvents);
	cv::createTrackbar("ErrorRate", "menu", &iCorrectionRate, 10, cvTackBarEvents);
	cv::createTrackbar("Enclosed", "menu", &iEnclosed, 1, cvTackBarEvents);
	cv::createTrackbar("ShowAll", "menu", &iShowAllCandidates, 1, cvTackBarEvents);
	iThreshold = MDetector.getParameters().ThresHold;
	iCornerMode = MDetector.getParameters().cornerRefinementM;
}

void putText(cv::Mat& im, string text, cv::Point p, float size)
{
	float fact = float(im.cols) / float(640);
	if (fact < 1)
		fact = 1;

	cv::putText(im, text, p, FONT_HERSHEY_SIMPLEX, size, cv::Scalar(0, 0, 0), 3 * fact);
	cv::putText(im, text, p, FONT_HERSHEY_SIMPLEX, size, cv::Scalar(125, 255, 255), 1 * fact);
}
void printHelp(cv::Mat& im)
{
	float fs = float(im.cols) / float(1200);

	putText(im, "'m': show/hide menu", cv::Point(10, fs * 60), fs * 0.5f);
	putText(im, "'s': start/stop video capture", cv::Point(10, fs * 80), fs * 0.5f);
	putText(im, "'w': write image to file", cv::Point(10, fs * 100), fs * 0.5f);
	putText(im, "'t': do a speed test", cv::Point(10, fs * 120), fs * 0.5f);
	putText(im, "'f': saves current configuration to file 'arucoConfig.yml'", cv::Point(10, fs * 140), fs * 0.5f);
}

void printInfo(cv::Mat& im)
{
	float fs = float(im.cols) / float(1200);
	putText(im, "fps=" + to_string(1. / Fps.getAvrg()), cv::Point(10, fs * 20), fs * 0.5f);
	putText(im, "'h': show/hide help", cv::Point(10, fs * 40), fs * 0.5f);
	if (bPrintHelp)
		printHelp(im);
}

void printMenuInfo()
{
	cv::Mat image(200, 400, CV_8UC3);
	image = cv::Scalar::all(255);
	string str = "Dictionary=" + aruco::Dictionary::getTypeString((aruco::Dictionary::DICT_TYPES)iDictionaryIndex);

	cv::putText(image, str, cv::Size(10, 20), FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(0, 0, 0), 1);

	str = "Detection Mode=" + MarkerDetector::Params::toString(MDetector.getParameters().detectMode);
	cv::putText(image, str, cv::Size(10, 40), FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(0, 0, 0), 1);
	str = "Corner Mode=" + MarkerDetector::Params::toString(MDetector.getParameters().cornerRefinementM);
	cv::putText(image, str, cv::Size(10, 60), FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(0, 0, 0), 1);
	cv::imshow("menu", image);
}

/************************************
*
*
*
*
************************************/

int main(int argc, char** argv)
{
	try
	{
		int vIdx = 0;// camera index
		float TheMarkerSize = 0.037;// marker size
		float resize_factor = 1; //resize factor
		std::string camera_params_path = "camera_params.yml"; //camera paramerter path
		std::string  dictionaryString = "ARUCO_MIP_25h7";
		std::string arucoconfig_path = "arucoConfig.yml";

		bool help = false;
		{
			using namespace clipp;//https://github.com/muellan/clipp
			auto cli = ((
				option("-i").doc("set camera_index(e.g 0 or 1)") & value("camera_index", vIdx),
				option("-s").doc("set marker size [m]") & value("marker_size_in_meters", TheMarkerSize),
				option("-c").doc("read [camera_params.yml]") & value("camera_params.yml", camera_params_path),
				option("-d").doc("set dictionary") & value("dictionary", dictionaryString),
				option("-rf").doc("set resize factor") & value("resize_factor", resize_factor),
				option("-config").doc("Load detector configuration file") & value("arucoConfig.yml", arucoconfig_path)
				)
				| option("-h", "--help").doc("show help").set(help, true)
				);

			if (!parse(argc, argv, cli) || help) {
				std::cout << make_man_page(cli, argv[0]);
				cerr << "Dictionaries: ";
				for (auto dict : aruco::Dictionary::getDicTypes())
					cerr << dict << " ";
				return 0;
			}
			printf("camera index: %d\n", vIdx);
			printf("marker size[m]:%f\n", TheMarkerSize);
			printf("camera_params:%s\n", camera_params_path.c_str());
			printf("marker dictionary:%s\n", dictionaryString.c_str());
			printf("resize_factor %f\n", resize_factor);
			printf("detector configuration file %s\n", arucoconfig_path.c_str());

		}

		TheCameraParameters.readFromXMLFile(camera_params_path);


		///////////  OPEN VIDEO
		// read from camera or from  file
		//vIdx = std::stoi(cml("-i", "0")); //camera id
		cout << "Opening camera index " << vIdx << endl;

		std::vector<int> params = {
			// https://docs.opencv.org/4.5.3/d4/d15/group__videoio__flags__base.html
			cv::CAP_PROP_FRAME_WIDTH,1920,
			cv::CAP_PROP_FRAME_HEIGHT,1080,
			cv::CAP_PROP_FPS,120,
			cv::CAP_PROP_EXPOSURE,-8,
			cv::CAP_PROP_GAIN,250,
			cv::CAP_PROP_BRIGHTNESS,100,
		};
		TheVideoCapturer.open(vIdx, cv::CAP_DSHOW, params);
		waitTime = 1;
		isVideo = true;

		// check video is open
		if (!TheVideoCapturer.isOpened())
			throw std::runtime_error("Could not open video");

		////create windows
		cv::namedWindow("in", cv::WINDOW_NORMAL);
		cv::resizeWindow("in", 1920, 1080);
		cv::namedWindow("thres", cv::WINDOW_NORMAL);

		///// CONFIGURE DATA
		// read first image to get the dimensions
		TheVideoCapturer >> TheInputImage;
		if (TheCameraParameters.isValid())
			TheCameraParameters.resize(TheInputImage.size());


		iDictionaryIndex = (uint64_t)aruco::Dictionary::getTypeFromString(dictionaryString);
		MDetector.setDictionary(dictionaryString, float(iCorrectionRate) / 10.); // sets the dictionary to be employed (ARUCO,APRILTAGS,ARTOOLKIT,etc)

		//MDetector.loadParamsFromFile(arucoconfig_path);

		iThreshold = MDetector.getParameters().ThresHold;
		iCornerMode = MDetector.getParameters().cornerRefinementM;

		


		setParamsFromGlobalVariables(MDetector);

		// go!
		char key = 0;
		int index = 0, indexSave = 0;
		// capture until press ESC or until the end of the video

		do
		{
			TheVideoCapturer.retrieve(TheInputImage);
			//std::cout << "Frame:" << TheVideoCapturer.get(CV_CAP_PROP_POS_FRAMES) << std::endl;
			TheInputImage = resizeImage(TheInputImage, resize_factor);
			// copy image
			Fps.start();
			TheMarkers = MDetector.detect(TheInputImage, TheCameraParameters, TheMarkerSize);

			for (auto& marker : TheMarkers)  // for each marker
				MTracker[marker.id].estimatePose(marker, TheCameraParameters, TheMarkerSize);  // call its tracker and estimate the pose


			Fps.stop();
			//// chekc the speed by calculating the mean speed of all iterations
			//cout << "\rTime detection=" << Fps.getAvrg()*1000 << " milliseconds nmarkers=" << TheMarkers.size() <<" images resolution="<<TheInputImage.size() <<std::endl;

			// print marker info and draw the markers in image
			TheInputImage.copyTo(TheInputImageCopy);

			if (iShowAllCandidates)
			{
				auto candidates = MDetector.getCandidates();
				for (auto cand : candidates) {
					Marker(cand, -1).draw(TheInputImageCopy, Scalar(255, 0, 255));
				}
			}

			for (unsigned int i = 0; i < TheMarkers.size(); i++)
			{
				//cout << TheMarkers[i] << endl;
				TheMarkers[i].draw(TheInputImageCopy, Scalar(0, 0, 255), 2, true, false);
			}

			// draw a 3d cube in each marker if there is 3d info
			if (TheCameraParameters.isValid() && TheMarkerSize > 0)
				for (unsigned int i = 0; i < TheMarkers.size(); i++)
				{
					if (TheMarkers[i].isPoseValid()) {
						CvDrawingUtils::draw3dCube(TheInputImageCopy, TheMarkers[i], TheCameraParameters);
						CvDrawingUtils::draw3dAxis(TheInputImageCopy, TheMarkers[i], TheCameraParameters);
						//std::cout << TheMarkers[i].Tvec << std::endl;
					}
				}

			// DONE! Easy, right?
			// show input with augmented information and the thresholded image
			printInfo(TheInputImageCopy);
			if (showMennu)
				printMenuInfo();

			cv::imshow("thres", resize(MDetector.getThresholdedImage(), 1024));

			cv::imshow("in", TheInputImageCopy);

			key = cv::waitKey(waitTime); // wait for key to be pressed
			if (key == 's')
				waitTime = waitTime == 0 ? 1 : 0;
			if (key == 'w')
			{ //writes current input image
				string number = std::to_string(indexSave++);
				while (number.size() != 3)
					number = "0" + number;
				string imname = "arucoimage" + number + ".png";
				cv::imwrite(imname, TheInputImageCopy);
				cout << "saved " << imname << endl;
				imname = "orgimage" + number + ".png";
				cv::imwrite(imname, TheInputImage);
				cout << "saved " << imname << endl;
				imname = "thresimage" + number + ".png";
				cv::imwrite(imname, MDetector.getThresholdedImage());
			}
			if (key == 'm')
			{
				if (showMennu)
					cv::destroyWindow("menu");
				else
				{
					cv::namedWindow("menu", cv::WINDOW_NORMAL);
					cv::resizeWindow("menu", 640, 480);
					createMenu();
					printMenuInfo();
				}
				showMennu = !showMennu;
			}
			if (key == 'h')
				bPrintHelp = !bPrintHelp;

			if (key == 't')
			{ //run a deeper speed test

				for (int t = 0; t < 30; t++)
				{
					// Detection of markers in the image passed
					Fps.start();
					TheMarkers = MDetector.detect(TheInputImage, TheCameraParameters, TheMarkerSize);
					Fps.stop();
					// chekc the speed by calculating the mean speed of all iterations
				}
				printInfo(TheInputImageCopy);
			}
			if (key == 'f')
			{
				cerr << "Configuration saved to arucoConfig.yml" << endl;
				MDetector.saveParamsToFile("arucoConfig.yml");
			}
			index++; // number of images captured

			if (isVideo)
				if (TheVideoCapturer.grab() == false)
					key = 27;
		} while (key != 27);
	}
	catch (std::exception& ex)

	{
		cout << "Exception :" << ex.what() << endl;
	}
}

void cvTackBarEvents(int pos, void*)
{
	(void)(pos);

	setParamsFromGlobalVariables(MDetector);

	// recompute
	Fps.start();
	TheMarkers = MDetector.detect(TheInputImage);
	Fps.stop();
	// chekc the speed by calculating the mean speed of all iterations
	TheInputImage.copyTo(TheInputImageCopy);
	if (iShowAllCandidates)
	{
		auto candidates = MDetector.getCandidates();
		for (auto cand : candidates)
			Marker(cand, -1).draw(TheInputImageCopy, Scalar(255, 0, 255), 1);
	}

	for (unsigned int i = 0; i < TheMarkers.size(); i++)
	{
		//cout << TheMarkers[i] << endl;
		TheMarkers[i].draw(TheInputImageCopy, Scalar(0, 0, 255), 2);
	}

	// draw a 3d cube in each marker if there is 3d info
	if (TheCameraParameters.isValid())
		for (unsigned int i = 0; i < TheMarkers.size(); i++)
			CvDrawingUtils::draw3dCube(TheInputImageCopy, TheMarkers[i], TheCameraParameters);
	cv::putText(TheInputImageCopy, "fps=" + to_string(1. / Fps.getAvrg()), cv::Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5f, cv::Scalar(125, 255, 255), 2);

	cv::imshow("in", TheInputImageCopy);
	cv::imshow("thres", resize(MDetector.getThresholdedImage(), 1024));
	if (showMennu)
		printMenuInfo();
}
