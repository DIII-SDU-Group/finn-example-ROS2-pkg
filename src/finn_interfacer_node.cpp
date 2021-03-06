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
	    
    RCLCPP_INFO(this->get_logger(), "Successfully initialized IPs");

    RCLCPP_INFO(this->get_logger(), "Flushing fetch_finn FIFO");

    //while(!XFetch_finn_IsIdle(&fetch_finn));
    if(XFetch_finn_IsIdle(&fetch_finn)){

        RCLCPP_INFO(this->get_logger(), "Starting fetch_finn");
        XFetch_finn_Start(&fetch_finn);
        RCLCPP_INFO(this->get_logger(), "Started fetch_finn");
        
    }

    RCLCPP_INFO(this->get_logger(), "fetch_finn ready");

     
    std::chrono::nanoseconds sleepperiod(20000000);
    rclcpp::GenericRate<std::chrono::high_resolution_clock> rate(sleepperiod);

    while(XFetch_finn_IsIdle(&fetch_finn)){
        XFetch_finn_Start(&fetch_finn);
        rate.sleep();  
    }
	    
    RCLCPP_INFO(this->get_logger(), "Successfully started fetch_finn");

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
    
    if (!start_time_set)
    {
        start_time = std::chrono::system_clock::now();
        timing = true;
        // start_time = std::chrono::system_clock::now();
        start_time_set = true;
    }

    static int its_recv = 0;

    RCLCPP_DEBUG(this->get_logger(), "Image received");

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);

    cv::Mat img = cv_ptr->image;

    int width = img.size().width;
    int x_mid = width/2;
    int x_low = x_mid - (256/2);

    int height = img.size().height;
    int y_mid = height/2;
    int y_low = y_mid - (256/2);

    // Crop image

    // cv::cvtColor(img, img, CV_BGR2RGB);

    cv::resize(img, img, cv::Size(IMAGE_X, IMAGE_Y), cv::INTER_LINEAR);

    if (timing)
    {
        resize_ts = std::chrono::system_clock::now();
    }

    //std::ostringstream stringStream;
    //stringStream << "/home/mp4d/images_testing/recv/" << std::to_string(its_recv++) << ".png";
    //cv::imwrite(stringStream.str(), img);

    img_q.push(img);

    int status = streamImageToFinn(img);

    if (timing)
    {
        stf_ts = std::chrono::system_clock::now();
    }

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

    if (img.total() != SIZE/CHANNELS)
    {
        RCLCPP_ERROR(this->get_logger(), "Expected image of size %d, got size %d, aborting", SIZE, img.total());

        return 0;
    }

    std::vector<uint8_t> img_vec_in;

    img_vec_in.assign(img.data, img.data + img.total()*CHANNELS);

    
    // uint8_t img_arr[SIZE];
    // int cnt = 0;
    // for (int i = 0; i < IMAGE_X; i++)
    // {
    //     for (int j = IMAGE_Y-1; j > -1; j--)
    //     {
    //         for (int k = CHANNELS-1; k > -1; k--)
    //         {
    //             img_arr[cnt++] = img.at<uint8_t>(i,j,k);
    //         }
    //     }
    // }
    

    // static int its = 0;

    // if (its == 0){
    //     int success = callStreamToFinnIP(&test_image0[0]);
    // }
    // if (its == 1){
    //     int success = callStreamToFinnIP(&test_image1[0]);
    // }
    // if (its == 2){
    //     int success = callStreamToFinnIP(&test_image2[0]);
    // }
    // if (its == 3){
    //     int success = callStreamToFinnIP(&test_image3[0]);
    // }
    // if (its == 4){
    //     int success = callStreamToFinnIP(&test_image4[0]);
    // }
    // if (its == 5){
    //     int success = callStreamToFinnIP(&test_image5[0]);
    // }
    // if (its == 6){
    //     int success = callStreamToFinnIP(&test_image6[0]);
    // }
    // if (its == 7){
    //     int success = callStreamToFinnIP(&test_image7[0]);
    // }
    // if (its == 8){
    //     int success = callStreamToFinnIP(&test_image8[0]);
    // }
    // if (its == 9){
    //     int success = callStreamToFinnIP(&test_image9[0]);
    // }
    // if (its == 10){
    //     int success = callStreamToFinnIP(&test_image10[0]);
    // }
    // if (its == 11){
    //     int success = callStreamToFinnIP(&test_image11[0]);
    // }
    // if (its == 12){
    //     int success = callStreamToFinnIP(&test_image12[0]);
    // }
    // if (its == 13){
    //     int success = callStreamToFinnIP(&test_image13[0]);
    // }
    // if (its == 14){
    //     int success = callStreamToFinnIP(&test_image14[0]);
    // }
    // if (its == 15){
    //     int success = callStreamToFinnIP(&test_image15[0]);
    // }
    // if (its == 16){
    //     RCLCPP_DEBUG(this->get_logger(), "All images dones");
    //     return 0;
    // }

    // its++;





    // int success = callStreamToFinnIP(&test_image0[0]);
    // int success = callStreamToFinnIP(&img_arr[0]);
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

    if (timing)
    {
        finn_done_ts = std::chrono::system_clock::now();
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
    RCLCPP_DEBUG(this->get_logger(), "Timer callback");

    static int its = 0;
    uint8_t result[4];
    RCLCPP_DEBUG(this->get_logger(), "Calling fetch_finn");
    int success = callFetchFinnIP(result);

    // uint8_t tmp_val = result[3];
    // result[3] = 255 - result[1];
    // result[1] = 255 - tmp_val;

    if (!success)
    {
        RCLCPP_DEBUG(this->get_logger(), "Fetch finn not done yet");
        return;
    }

    RCLCPP_DEBUG(this->get_logger(), "Fetch finn done");

    if (!img_q.empty())
    {
        RCLCPP_DEBUG(this->get_logger(), "Image in queue");

        cv::Mat bbox_img = img_q.front();
        img_q.pop();

        cv::Rect rect(result[3], result[2], result[1]-result[3], result[0]-result[2]);

        cv::cvtColor(bbox_img, bbox_img, CV_BGR2RGB);

        cv::rectangle(bbox_img, rect, cv::Scalar(0, 0, 255));

        cv::circle(bbox_img, cv::Point(result[3]+((result[1]-result[3])/2), result[2]+((result[0]-result[2])/2)),5, cv::Scalar(0,0,255),-1, 8,0);

        cv::line(bbox_img, cv::Point(256/2,0), cv::Point(256/2,255), cv::Scalar(0,0,255));

        static cv::VideoWriter video("/home/mp4d/test_video.avi", cv::VideoWriter::fourcc('M','J','P','G'), 23, cv::Size(256,256));
        video.write(bbox_img);

        cv_bridge::CvImage img_out;
        img_out.header.set__frame_id("camera");
        img_out.header.set__stamp(rclcpp::Clock().now());
        img_out.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
        img_out.image = bbox_img;

        sensor_msgs::msg::Image::SharedPtr msg_out = img_out.toImageMsg();
        bbox_img_publisher_->publish(*msg_out);

        // std::ostringstream stringStream;
        // stringStream << "/home/mp4d/images_testing/bbox/" << std::to_string(its) << ".png";
        // cv::imwrite  (stringStream.str(), bbox_img);

        RCLCPP_DEBUG(this->get_logger(), "Published bbox image");
    } else
    {
        RCLCPP_FATAL(this->get_logger(), "Got Finn result, but no image is in queue");
    }

    // std::ostringstream stringStream;
    // stringStream << "/home/mp4d/images_testing/bbox/" << std::to_string(its++) << ".txt";
    // std::ofstream bbox_file;
    // bbox_file.open(stringStream.str());
    // bbox_file << std::to_string(result[3]) << "\t" << std::to_string(result[2]) << "\t" << std::to_string(result[1]) << "\t" << std::to_string(result[0]); // xmin ymin xmax ymax in .txt
    // bbox_file.close();

    its++;

    vision_msgs::msg::BoundingBox2D msg;

    msg.center.x = (result[3] + result[1])/2;
    msg.center.y = (result[2] + result[0])/2;
    msg.center.theta = 0;
    msg.size_x = result[1] - result[3];
    msg.size_y = result[0] - result[2];

    bbox_publisher_->publish(msg);
    
    RCLCPP_DEBUG(this->get_logger(), "Published bbox");
    std::chrono::time_point<std::chrono::system_clock> publish_done_ts = std::chrono::system_clock::now();

    if (timing)
    {

        int resize_dur = std::chrono::duration_cast<std::chrono::nanoseconds>(resize_ts - start_time).count();
        int stf_dur = std::chrono::duration_cast<std::chrono::nanoseconds>(stf_ts - resize_ts).count();
        int finn_dur = std::chrono::duration_cast<std::chrono::nanoseconds>(finn_done_ts - stf_ts).count();
        int publish_dur = std::chrono::duration_cast<std::chrono::nanoseconds>(publish_done_ts - finn_done_ts).count();
        int total_dur = std::chrono::duration_cast<std::chrono::nanoseconds>(publish_done_ts - start_time).count();

    
        RCLCPP_INFO(this->get_logger(), "Resize time: %d ns", resize_dur);
        RCLCPP_INFO(this->get_logger(), "Stream to finn time: %d ns", stf_dur);
        RCLCPP_INFO(this->get_logger(), "Finn processing time: %d ns", finn_dur);
        RCLCPP_INFO(this->get_logger(), "Publishing time: %d ns", publish_dur);
        RCLCPP_INFO(this->get_logger(), "Total time: %d ns", total_dur);

        timing = false;
    }
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
