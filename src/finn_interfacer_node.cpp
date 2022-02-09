/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "finn_interfacer_node.h"

/*****************************************************************************/
// Implementation
/*****************************************************************************/

FinnInterfacerNode::FinnInterfacerNode(const std::string & node_name, const std::string & node_namespace) 
    : Node(node_name, node_namespace)
{
    initIPs();
	    
    RCLCPP_DEBUG(this->get_logger(), "Successfully initialized IPs, waiting for fetch_finn to finish");

    //while(!XFetch_finn_IsIdle(&fetch_finn));
    if(XFetch_finn_IsIdle(&fetch_finn)){

        XFetch_finn_Start(&fetch_finn);

    }
     
    std::chrono::nanoseconds sleepperiod = 20000000;
    rclcpp::rate::GenericRate rate(sleepperiod);

    while(XFetch_finn_IsIdle(&fetch_finn)){
        XFetch_finn_Start(&fetch_finn);
        rate.sleep();  
    }
	    
    RCLCPP_DEBUG(this->get_logger(), "Successfully started fetch_finn");

    rclcpp::QoS video_qos(10);
    video_qos.keep_last(10);
    video_qos.reliable();
    video_qos.durability_volatile();

    subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image", video_qos, std::bind(&FinnInterfacerNode::imageRecvCallback, this, std::placeholders::_1));

    char bbox_topic[100];
    sprintf(bbox_topic, "%s/bbox", this->get_namespace());

    char bbox_img_topic[100];
    sprintf(bbox_img_topic, "%s/bbox_img", this->get_namespace());

    bbox_publisher_ = this->create_publisher<vision_msgs::msg::BoundingBox2D>(bbox_topic, 10);
    bbox_img_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(bbox_img_topic, video_qos);
    timer_ = this->create_wall_timer(5ms, std::bind(&FinnInterfacerNode::timer_callback, this));
}

FinnInterfacerNode::~FinnInterfacerNode()
{
    XStreamtofinn_Release(&stream_to_finn);
    XFetch_finn_Release(&fetch_finn);
}

void FinnInterfacerNode::initIPs()
{    
    int success = XStreamtofinn_Initialize(&stream_to_finn, STREAMTOFINN_NAME);

	if (success == XST_DEVICE_NOT_FOUND)
	{
		RCLCPP_FATAL(this->get_logger(), "Device stream_to_finn not found");
		
        rclcpp::shutdown();
	}

	if (success == XST_OPEN_DEVICE_FAILED)
	{
		RCLCPP_FATAL(this->get_logger(), "Open device stream_to_finn failed");
		
        rclcpp::shutdown();
	}

    if (success != XST_SUCCESS)
    {
        RCLCPP_FATAL(this->get_logger(), "Component stream_to_finn initialization failed ");
    
        rclcpp::shutdown();
    }        
    else{
        RCLCPP_INFO(this->get_logger(), "Component stream_to_finn initialization successful");
    }

    
    success = XFetch_finn_Initialize(&fetch_finn, FETCHFINN_NAME);

	if (success == XST_DEVICE_NOT_FOUND)
	{
		RCLCPP_FATAL(this->get_logger(), "Device fetch_finn not found");
		
        rclcpp::shutdown();
	}

	if (success == XST_OPEN_DEVICE_FAILED)
	{
		RCLCPP_FATAL(this->get_logger(), "Open device fetch_finn failed");
		
        rclcpp::shutdown();
	}

    if (success != XST_SUCCESS)
    {
        RCLCPP_FATAL(this->get_logger(), "Component fetch_finn initialization failed ");
    
        rclcpp::shutdown();
    }        
    else{
        RCLCPP_INFO(this->get_logger(), "Component fetch_finn initialization successful");
    }
}

void FinnInterfacerNode::imageRecvCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    RCLCPP_DEBUG(this->get_logger(), "Image received");

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);

    cv::Mat img;
    cv::cvtColor(cv_ptr->image, img, CV_BGR2RGB);
    cv::resize(img, img, cv::Size(IMAGE_X, IMAGE_Y), cv::INTER_LINEAR);

    img_q.push(img);

    int status = streamImageToFinn(img);

    if (!status)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed streaming image to finn");

        return;
    }

    RCLCPP_DEBUG(this->get_logger(), "Successfully streamed image to finn");
}


int FinnInterfacerNode::streamImageToFinn(const cv::Mat img)
{
    RCLCPP_DEBUG(this->get_logger(), "Calling stream_to_finn");

    if (img.total() != SIZE)
    {
        RCLCPP_ERROR(this->get_logger(), "Expected image of size %d, got size %d, aborting", SIZE, img.total());

        return 0;
    }

    std::vector<uint8_t> img_vec_in;

    img_vec_in.assign(img.data, img.data + img.total());

    int success = callStreamToFinnIP(&img_vec_in[0]);

    if (!success)
    {
        RCLCPP_ERROR(this->get_logger(), "Unsuccessful call to IP stream_to_finn");

        return 0;
    }
    
    RCLCPP_DEBUG(this->get_logger(), "Successful call to stream_to_finn IP");

    return 1;
}

int FinnInterfacerNode::callStreamToFinnIP(uint8_t *ptr_img_data_in)
{
    int length;

    for (int i = 0; i < N_BATCHES; i++)
    {
        RCLCPP_DEBUG(this->get_logger(), "Polling for stream_to_finn IP ready");

        while(!XStreamtofinn_IsReady(&stream_to_finn));

        length = XStreamtofinn_Write_image_in_Bytes(&stream_to_finn, 0, (char *)&(ptr_img_data_in[i*BATCH_SIZE]), BATCH_SIZE);

        if(length == BATCH_SIZE)
        {
            RCLCPP_DEBUG(this->get_logger(), "Wrote batch to stream_to_finn IP");
        } else
        {
            RCLCPP_ERROR(this->get_logger(), "Could not write batch to stream_to_finn IP");
            
            return 0;
        }

        RCLCPP_DEBUG(this->get_logger(), "Polling for stream_to_finn IP idle");

        while(!XStreamtofinn_IsIdle(&stream_to_finn));

        RCLCPP_DEBUG(this->get_logger(), "Starting stream_to_finn IP");

		XStreamtofinn_Start(&stream_to_finn);

        RCLCPP_DEBUG(this->get_logger(), "Started stream_to_finn IP");
    }

    return 1;
}

int FinnInterfacerNode::callFetchFinnIP(uint8_t result[4]){
        
    if(!XFetch_finn_IsDone(&fetch_finn)){
        return 0;
    }

    XFetch_finn_Read_res_out_Bytes(&fetch_finn, 0, (char*)(&result[0]), 4);

    RCLCPP_DEBUG(this->get_logger(), "Finished fetching result with fetch_finn IP");

    while(!XFetch_finn_IsIdle(&fetch_finn));

    XFetch_finn_Start(&fetch_finn);

    RCLCPP_DEBUG(this->get_logger(), "Started fetch_finn IP");

    return 1;
}

void FinnInterfacerNode::timer_callback()
{
    uint8_t result[4];
    int success = callFetchFinnIP(result);

    if (!success)
        return;

    if (!img_q.empty())
    {
        RCLCPP_DEBUG(this->get_logger(), "Image in queue");

        cv::Mat bbox_img = img_q.front();
        img_q.pop();

        cv::Rect rect(result[3], result[2], result[1]-result[3], result[0]-result[2]);

        cv::rectangle(bbox_img, rect, cv::Scalar(0, 0, 255));

        cv_bridge::CvImage img_out;
        img_out.header.set__frame_id("camera");
        img_out.header.set__stamp(rclcpp::Clock().now());
        img_out.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
        img_out.image = bbox_img;

        sensor_msgs::msg::Image::SharedPtr msg_out = img_out.toImageMsg();
        bbox_img_publisher_->publish(*msg_out);

        RCLCPP_DEBUG(this->get_logger(), "Published bbox image");
    } else
    {
        RCLCPP_ERROR(this->get_logger(), "Got Finn result, but no image is in queue");
    }
    
    vision_msgs::msg::BoundingBox2D msg;

    msg.center.x = (result[3] + result[1])/2;
    msg.center.y = (result[2] + result[0])/2;
    msg.center.theta = 0;
    msg.size_x = result[3] - result[1];
    msg.size_y = result[2] - result[0];

    bbox_publisher_->publish(msg);
    
    RCLCPP_DEBUG(this->get_logger(), "Published bbox");
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor executor;

    auto node = std::make_shared<FinnInterfacerNode>();

    executor.add_node(node);

    executor.spin();

    rclcpp::shutdown();

    return 0;
}
