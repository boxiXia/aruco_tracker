/**
Copyright 2017 Rafael Muñoz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
*/

#include "aruco.h"
#include "timers.h"
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <sstream>
#include <string>
#include <stdexcept>
#include <Eigen/Geometry>
#include "sglviewer.h"
#include <clipp.h>
#include <msgpack.hpp>
#include <asio.hpp>

#include <deque>
#include <mutex>

std::string TheMarkerMapConfigFile;
//cv::VideoCapture TheVideoCapturer;
cv::Mat TheInputImage, TheInputImageCopy;
aruco::CameraParameters TheCameraParameters;
aruco::MarkerMap TheMarkerMapConfig;
aruco::MarkerDetector TheMarkerDetector;
aruco::MarkerMapPoseTracker TheMSPoseTracker;
//aruco::sgl_OpenCV_Viewer viewer;

class Camera {
public:
    cv::VideoCapture cap;
    Camera(
        int index = 1,
        std::vector<int> params = {
            // https://docs.opencv.org/4.5.3/d4/d15/group__videoio__flags__base.html
            cv::CAP_PROP_FRAME_WIDTH, 2560,
            cv::CAP_PROP_FRAME_HEIGHT, 1440,
            cv::CAP_PROP_FPS, 200,
            cv::CAP_PROP_EXPOSURE, -7,
            cv::CAP_PROP_GAIN, 200,
            cv::CAP_PROP_BRIGHTNESS, 100,
        }) {
        cap.open(index, cv::CAP_DSHOW, params);
        cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

        if (!cap.isOpened())
            throw std::runtime_error("Could not open video");

        cv::Mat _bgr_frame;
        cap >> _bgr_frame;
        dq.emplace_front(_bgr_frame);

        update_thread = std::thread(&Camera::update, this);
    }
    ~Camera() { close(); }
    void update() {
        aruco::Timer avrgTimer;
        while (should_run) {
            //avrgTimer.start();
            cv::Mat _bgr_frame;
            cap >> _bgr_frame;
            //cap.grab();
            //cap.retrieve(_bgr_frame);
            //avrgTimer.end();
            //std::cout << "Average time " << avrgTimer.getAverage() << " ms" << std::endl;

            dq.emplace_front(_bgr_frame);
            {
                std::lock_guard<std::mutex> lock(m);
                while (dq.size() > 1) {
                    dq.pop_back();
                }
            }

        }
    }
    bool grab() {
        return !dq.empty();
    }
    bool retrieve(cv::Mat& outframe) {
        std::lock_guard<std::mutex> lock(m);
        //dq.front().copyTo(outframe);
        outframe = dq.front();
        return true;
    }

    void close() {
        should_run = false;
        update_thread.join();
        cap.release();
    }
private:
    std::mutex m;
    std::deque<cv::Mat> dq;
    std::thread update_thread;
    //cv::Mat bgr_frame;
    bool should_run = true;
};

class DisplayHelper {
public:

    aruco::MarkerMap marker_map;
    cv::Mat input_image; // should update every time
    cv::Mat rt_matrix; // should uipdate every time
    std::vector<aruco::Marker> detected_markers; // should update every time
    bool should_plot = false;
    char key = 0;
    int wait_time = 1;
    bool should_run = true;
    float marker_size;
    DisplayHelper(float marker_size, aruco::MarkerMap marker_map) {
        this->marker_size = marker_size;
        this->marker_map = marker_map;
        update_thread = std::thread(&DisplayHelper::update, this);
    }
    ~DisplayHelper() {
        should_run = false;
        update_thread.join();
    }

private:
    std::thread update_thread;

    void update() {

        aruco::sgl_OpenCV_Viewer viewer;

        viewer.setParams(0.5, 1920, 1080, "map_viewer", marker_size);
        std::cout << "Press 's' to start/stop video" << std::endl;

        while (should_run && (key != 27)) {
            if (should_plot) {

                // print the markers detected that belongs to the markerset
                for (auto idx : marker_map.getIndices(detected_markers))
                    detected_markers[idx].draw(input_image, cv::Scalar(0, 0, 255), 1);

                key = viewer.show(marker_map, rt_matrix, input_image, wait_time);
                if (key == 's') wait_time = wait_time ? 0 : 1;
                should_plot = false;
            }
        }
    }
};





int main(int argc, char** argv)
{
    try
    {
        int waitTime = 1;
        int vIdx = 0;// camera index
        float TheMarkerSize = 0.03;// marker size
        // defualt values
        std::string camera_params_path = "camera_params.yml"; //camera paramerter path
        std::string markermapconfig_path = "markerset.yml";
        std::string arucoconfig_path = "arucoConfig.yml";

        std::string destination_ip = "127.0.0.1";
        unsigned short port = 32000;

        bool verbose = false;
        bool help = false;
        {
            using namespace clipp;//https://github.com/muellan/clipp
            auto cli = ((
                option("-i").doc("set camera_index(e.g 0 or 1)") & value("camera_index", vIdx),
                option("-m").doc("load marker map config file") & value("markerset.yml", markermapconfig_path),
                option("-c").doc("load camera_params.yml") & value("camera_params.yml", camera_params_path),
                option("-s").doc("set marker size [m]") & value("marker_size_in_meters", TheMarkerSize),
                option("-v").doc("set verbose to true").set(verbose, true),
                option("-config").doc("Load detector configuration file") & value("arucoConfig.yml", arucoconfig_path)
                )
                | option("-h", "--help").doc("show help").set(help, true)
                );

            auto result = parse(argc, argv, cli);
            if (!result || help) {
                std::cout << make_man_page(cli, argv[0]);
                std::cerr << "Dictionaries: ";
                for (auto dict : aruco::Dictionary::getDicTypes())
                    std::cerr << dict << " ";
                return 0;
            }
            printf("camera index: %d\n", vIdx);
            printf("marker map config file:%s\n", markermapconfig_path.c_str());
            printf("camera_params:%s\n", camera_params_path.c_str());
            printf("detector configuration file %s\n", arucoconfig_path.c_str());
            printf("marker size [m] %f\n", TheMarkerSize);
        }

        asio::io_service io_service;
        asio::ip::udp::socket socket(io_service);
        // Create the remote endpoint using the destination ip address and
        // the target port number.  This is not a broadcast
        auto remote = asio::ip::udp::endpoint(asio::ip::address::from_string(destination_ip), port);

        // Open the socket, socket's destructor will
        // automatically close it.
        socket.open(asio::ip::udp::v4());


        

        Camera TheVideoCapturer(vIdx);
        //// read first image to get the dimensions
        TheVideoCapturer.retrieve(TheInputImage);

        // read camera parameters
        TheCameraParameters.readFromXMLFile(camera_params_path);
        TheCameraParameters.resize(TheInputImage.size());
        // prepare the detector
        TheMarkerDetector.loadParamsFromFile(arucoconfig_path);

        TheMarkerMapConfig.readFromFile(markermapconfig_path);
        TheMarkerDetector.setDictionary( TheMarkerMapConfig.getDictionary());

        

        // prepare the pose tracker if possible
        // if the camera parameers are avaiable, and the markerset can be expressed in meters, then go

        if (TheMarkerMapConfig.isExpressedInPixels() && TheMarkerSize > 0)
            TheMarkerMapConfig = TheMarkerMapConfig.convertToMeters(TheMarkerSize);

        std::cout << "TheCameraParameters.isValid()=" << TheCameraParameters.isValid() << " "<< TheMarkerMapConfig.isExpressedInMeters() << std::endl;

        if (TheCameraParameters.isValid() && TheMarkerMapConfig.isExpressedInMeters()){
            TheMSPoseTracker.setParams(TheCameraParameters, TheMarkerMapConfig);
            TheMarkerSize=cv::norm(TheMarkerMapConfig[0][0]- TheMarkerMapConfig[0][1]);
        }

        //// Create gui
        DisplayHelper viewer(TheMarkerSize, TheMarkerMapConfig);

        cv::Mat rt_matrix;

        aruco::Timer avrgTimer;
        do
        {
            //avrgTimer.start();

            TheVideoCapturer.retrieve(TheInputImage);
            
            // Detection of the markers
            std::vector<aruco::Marker> detected_markers = TheMarkerDetector.detect(TheInputImage);
            // estimate 3d camera pose if possible
            if (TheMSPoseTracker.isValid())
                if (TheMSPoseTracker.estimatePose(detected_markers)) {
                    rt_matrix = TheMSPoseTracker.getRTMatrix();
                    if (verbose) std::cout << cv::format(rt_matrix, cv::Formatter::FMT_NUMPY) << std::endl;
                    //cout<<TheMSPoseTracker.getRvec()<<" "<< TheMSPoseTracker.getTvec()<<endl;

                }
            // update the viewer
            if (viewer.should_plot == false) {
                viewer.detected_markers = detected_markers;
                viewer.input_image = TheInputImage;
                viewer.rt_matrix = rt_matrix.clone();
                viewer.should_plot = true;
            }

            if (rt_matrix.rows) {
                std::vector<float>rt_vec(rt_matrix.begin<float>(), rt_matrix.end<float>());
                msgpack::sbuffer sb;
                msgpack::pack(sb, rt_vec);
                socket.send_to(asio::buffer(sb.data(), sb.size()), remote);
            }

            //avrgTimer.end();
            //std::cout << "Average computing time " << avrgTimer.getAverage() << " ms" << std::endl;

        } while (viewer.key != 27);
    }
    catch (std::exception& ex)
    {
        std::cout << "Exception :" << ex.what() << std::endl;
    }
}



void  getQuaternionAndTranslationfromMatrix44(const cv::Mat &M_in ,double &qx,double &qy,double &qz,double &qw,double &tx,double &ty,double &tz){
    //get the 3d part of matrix and get quaternion
    assert(M_in.total()==16);
    cv::Mat M;
    M_in.convertTo(M,CV_64F);
    cv::Mat r33=cv::Mat ( M,cv::Rect ( 0,0,3,3 ) );
    //use now eigen
    Eigen::Matrix3f e_r33;
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            e_r33(i,j)=M.at<double>(i,j);

    //now, move to a angle axis
    Eigen::Quaternionf q(e_r33);
    qx=q.x();
    qy=q.y();
    qz=q.z();
    qw=q.w();


    tx=M.at<double>(0,3);
    ty=M.at<double>(1,3);
    tz=M.at<double>(2,3);
}


