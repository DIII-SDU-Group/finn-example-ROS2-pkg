#ifndef FINN_INTERFACER_H
#define FINN_INTERFACER_H

/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <queue>
#include <fstream>


#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/image.hpp>
//#include <example_interfaces/msg/u_int8_multi_array.h>
// #include "vision_msgs/msg/detection2_d.hpp"
#include "vision_msgs/msg/bounding_box2_d.hpp"
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rate.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

extern "C" {
#include "xstreamtofinn.h"
#include "xfetch_finn.h"
}

/*****************************************************************************/
// Defines
/*****************************************************************************/

#define	IMAGE_X		256
#define IMAGE_Y		256
#define CHANNELS	3
#define SIZE 		IMAGE_X*IMAGE_Y*CHANNELS
#define BATCH_SIZE	8*IMAGE_Y*CHANNELS
#define N_BATCHES	32

#define STREAMTOFINN_NAME   "streamToFinn"
#define FETCHFINN_NAME      "fetch_finn"

using namespace std::chrono_literals;

/*****************************************************************************/
// Class
/*****************************************************************************/

class FinnInterfacerNode : public rclcpp::Node
{
public:
    FinnInterfacerNode(const std::string & node_name="finn_interfacer", const std::string & node_namespace="");
    ~FinnInterfacerNode();

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;

    XStreamtofinn stream_to_finn;
    XFetch_finn fetch_finn;

    void initIPs();

    void imageRecvCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    int streamImageToFinn(const cv::Mat img);
    int callStreamToFinnIP(uint8_t *ptr_img_data_in);

    void timer_callback();

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<vision_msgs::msg::BoundingBox2D>::SharedPtr bbox_publisher_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr bbox_img_publisher_;

    int callFetchFinnIP(uint8_t result[4]);
    uint8_t result[4];

    std::queue<cv::Mat> img_q;
};

// /*****************************************************************************/
// // Main
// /*****************************************************************************/

int main(int argc, char *argv[]);

#endif