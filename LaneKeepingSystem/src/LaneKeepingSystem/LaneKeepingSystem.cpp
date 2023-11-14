#include "LaneKeepingSystem/LaneKeepingSystem.hpp"
#define USB_CAM_NODE 1

namespace Xycar {
template <typename PREC>
LaneKeepingSystem<PREC>::LaneKeepingSystem()
{
    std::string configPath;
    mNodeHandler.getParam("config_path", configPath);
    YAML::Node config = YAML::LoadFile(configPath);

    mPID = new PIDController<PREC>(config["PID"]["P_GAIN"].as<PREC>(), config["PID"]["I_GAIN"].as<PREC>(), config["PID"]["D_GAIN"].as<PREC>());
    mMovingAverage = new MovingAverageFilter<PREC>(config["MOVING_AVERAGE_FILTER"]["SAMPLE_SIZE"].as<uint32_t>());
    mLaneDetector = new LaneDetector<PREC>(config);
    /*
        create your lane detector.
    */
    setParams(config);

    mPublisher = mNodeHandler.advertise<xycar_msgs::xycar_motor>(mPublishingTopicName, mQueueSize);
    mSubscriber = mNodeHandler.subscribe(mSubscribedTopicName, mQueueSize, &LaneKeepingSystem::imageCallback, this);
}

template <typename PREC>
void LaneKeepingSystem<PREC>::setParams(const YAML::Node& config)
{
    mPublishingTopicName = config["TOPIC"]["PUB_NAME"].as<std::string>();
    mSubscribedTopicName = config["TOPIC"]["SUB_NAME"].as<std::string>();
    mQueueSize = config["TOPIC"]["QUEUE_SIZE"].as<uint32_t>();
    mXycarSpeed = config["XYCAR"]["START_SPEED"].as<PREC>();
    mXycarMaxSpeed = config["XYCAR"]["MAX_SPEED"].as<PREC>();
    mXycarMinSpeed = config["XYCAR"]["MIN_SPEED"].as<PREC>();
    mXycarSpeedControlThreshold = config["XYCAR"]["SPEED_CONTROL_THRESHOLD"].as<PREC>();
    mAccelerationStep = config["XYCAR"]["ACCELERATION_STEP"].as<PREC>();
    mDecelerationStep = config["XYCAR"]["DECELERATION_STEP"].as<PREC>();
    yOffset=config["XYCAR"]["Y_OFFSET"].as<PREC>();
    mDebugging = config["DEBUG"].as<bool>();
}

template <typename PREC>
LaneKeepingSystem<PREC>::~LaneKeepingSystem()
{
    delete mPID;
    delete mMovingAverage;
    // delete your LaneDetector if you add your LaneDetector.
}

template <typename PREC>
void LaneKeepingSystem<PREC>::run()
{
#if  USB_CAM_NODE
    ros::Rate rate(kFrameRate);
    while (ros::ok())
    {     
        ros::spinOnce();   
        //while  (mFrame.size() != (int)(mLaneDetector->GetWidth()*mLaneDetector->GetHeight()*3)) continue;
        if (mFrame.empty()){
            //std::cerr<<"Frame empty!"<<std::endl;
            continue;
        }
        double center = laneDetection(imageProcess(mFrame));
        double error = (center - mLaneDetector->GetWidth()/2);        
        auto steeringAngle = mPID->getControlOutput(error);
        drive(steeringAngle);
    }
#else
    cv::VideoCapture cap;
    cap.open(0); //임시
    cap.set(cv::CAP_PROP_POS_FRAMES, 0);
    cap.set(cv::CAP_PROP_FPS, 30);
    cv::Mat frame;

    if (!cap.isOpened()){
        std::cerr<<"Camera open failed!"<<std::endl;
        return;
    }
    ros::Rate rate(kFrameRate);
    while (ros::ok())
    {
        cap >> frame;
        if (frame.empty()){
            std::cerr<<"Frame empty!"<<std::endl;
            continue;
        }        
        double center = laneDetection(frame,imageProcess(frame));
        double error = (center - mLaneDetector->GetWidth()/2);        
        auto steeringAngle = mPID->getControlOutput(error);
        drive(steeringAngle);
    }
#endif
}

template <typename PREC>
std::vector<cv::Vec4f> LaneKeepingSystem<PREC>::imageProcess(cv::Mat& frame)
{
	cv::Mat img_gray, img_histo, img_blur, img_edge, roi, thresframe;
    std::vector<cv::Vec4f> lines;

    cv::Mat output;
    cv::Mat mask = cv::Mat::zeros(frame.size(), CV_8UC1);

    /*std::vector<cv::Point> polygon;
    polygon.push_back(cv::Point(0, frame.rows));
    polygon.push_back(cv::Point(50, (int)yOffset-50));
    polygon.push_back(cv::Point(580, (int)yOffset-50));
    polygon.push_back(cv::Point(frame.cols, frame.rows));

    cv::fillConvexPoly(mask, &polygon[0],4, cv::Scalar(255));
    */
    std::vector<cv::Point> square;
    square.push_back(cv::Point(0, frame.rows));
    square.push_back(cv::Point(0, (int)yOffset-50)); 
    square.push_back(cv::Point(frame.cols, (int)yOffset-50)); 
    square.push_back(cv::Point(frame.cols,frame.rows)); 

    cv::fillConvexPoly(mask, &square[0], 4, cv::Scalar(255));

    cv::cvtColor(frame, img_gray, cv::COLOR_BGR2GRAY);	
    //bitwise_and(img_gray, mask, output, mask= mask);        
    cv::GaussianBlur(img_gray, img_blur, cv::Size(), 2.0);
    cv::Canny(img_blur, img_edge, 100, 150);
    cv::bitwise_and(img_edge, mask, output, mask= mask);    
    cv::threshold(output,thresframe,190,255,cv::THRESH_BINARY);
    cv::HoughLinesP(thresframe, lines, 1, CV_PI / 180, 50, 70, 5);
    cv::imshow("mask",mask);
    cv::imshow("Output",output);
    return lines; 
}

template <typename PREC>
double LaneKeepingSystem<PREC>::laneDetection(std::vector<cv::Vec4f> all_lines)
{    
    std::vector<cv::Point> left_points, right_points;
    cv::Point lposl, lposr, rposl, rposr;
    std::vector<int> left_x_at_Y_offset;
    std::vector<int> right_x_at_Y_offset;
    cv::Mat result_frame=mFrame.clone();
    cv::Point lpos, rpos;
    double slope;
    static cv::Point prev_lpos(0,yOffset);
    static cv::Point prev_rpos(mLaneDetector->GetWidth(), yOffset);
    static int leftC = 0, rightC = 0;
    //drawCircle(result_frame, (lposl+lposr)/2, mLaneDetector->kBlue);
    for (cv::Vec4f line_ : all_lines) {
        cv::Point pt1(line_[0], line_[1]);
        cv::Point pt2(line_[2], line_[3]);
        slope = (double)(pt2.y - pt1.y) / (pt2.x - pt1.x + 0.0001);

        if (abs(slope) < 1) { 
            int x_at_Y_offset;
            if (pt1.x != pt2.x) {
                x_at_Y_offset = (yOffset - pt1.y) / slope + pt1.x;
            } else {
                x_at_Y_offset = pt1.x; 
            }

            if (slope < 0 && x_at_Y_offset <250) {
                left_x_at_Y_offset.push_back(x_at_Y_offset);
            } else if (slope > 0 && x_at_Y_offset >380) {
                right_x_at_Y_offset.push_back(x_at_Y_offset);
            }
        }
        drawLine(result_frame, pt1, pt2,mLaneDetector->kGreen);
    }

    if (!left_x_at_Y_offset.empty() && left_x_at_Y_offset.size() <3) {
        int lposl_x = *min_element(left_x_at_Y_offset.begin(), left_x_at_Y_offset.end());
        int lposr_x = *max_element(left_x_at_Y_offset.begin(), left_x_at_Y_offset.end());

        lpos = cv::Point((lposl_x+lposr_x)/2,yOffset);
        prev_lpos = lpos;
        leftC = 0;

    }else{
        lpos = prev_lpos;
        leftC +=1;
    }

    if (!right_x_at_Y_offset.empty() && right_x_at_Y_offset.size() <3) {
        int rposl_x = *min_element(right_x_at_Y_offset.begin(), right_x_at_Y_offset.end());
        int rposr_x = *max_element(right_x_at_Y_offset.begin(), right_x_at_Y_offset.end());

        rpos = cv::Point((rposl_x+rposr_x)/2, yOffset);
        prev_rpos = rpos;
        rightC =0;
    }else{
        rightC +=1;
        rpos = prev_rpos;
    }

    //when points are too close
    if (abs(rpos.x - lpos.x) <300){
        if(rightC != 0){
            rpos.x = 640;
            // rpos.x = lpos.x +300;
            prev_rpos = rpos;
        }else{
            lpos.x = 0;
            // lpos.x = rpos.x -300;
            prev_lpos = lpos;
        }
    }

    if(lpos.x >180){
        rpos.x += 50;
        prev_rpos.x += 1;
        rightC = 0;
    }else if(rpos.x < 400){
        lpos.x -=50;
        prev_lpos.x -=1;
        leftC =0;
    }

    // when prev track not update for long time
    if(rightC > 300 && leftC > 150){
        rpos.x = 640;
        // rpos.x = lpos.x +500;
        prev_rpos = rpos;
    }
    if(leftC >300&& rightC >150){
        lpos.x = 0;
        // lpos.x = rpos.x -500;
        prev_lpos = lpos;
    }

    drawCircle(result_frame, lpos, mLaneDetector->kRed); 
    putText(result_frame, cv::format("(%d, %d, left)", lpos.x, (int)yOffset), 
            lpos + cv::Point(-50, -20), 
            cv::FONT_HERSHEY_SIMPLEX, 0.5,mLaneDetector->kRed, 1, cv::LINE_AA);


    drawCircle(result_frame, rpos, mLaneDetector->kRed); 
    putText(result_frame, cv::format("(%d, %d,right)", rpos.x, (int)yOffset), 
            rpos + cv::Point(-50, -20), 
            cv::FONT_HERSHEY_SIMPLEX, 0.5, mLaneDetector->kRed, 1, cv::LINE_AA);


    drawCircle(result_frame, (lpos + rpos)/2, cv::Scalar(255, 0, 0)); 
    putText(result_frame, cv::format("(%d, %d,lane_center)", (int)(lpos.x + rpos.x)/2, (int)yOffset), 
        (lpos + rpos)/2 + cv::Point(-50, -20), 
        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1, cv::LINE_AA);

    drawCircle(result_frame,cv::Point(480/2, yOffset),cv::Scalar(0,0,0));

    cv::imshow("result_frame", result_frame);
    cv::waitKey(1);
    return (double)(lpos.x + rpos.x)/2;
}
template <typename PREC>
void LaneKeepingSystem<PREC>::drawLine(cv::Mat& frame, cv::Point start_p,cv::Point end_p, cv::Scalar color)
{		
	cv::line(frame,start_p, end_p, color, 2, cv::LINE_AA);
}
template <typename PREC>
void LaneKeepingSystem<PREC>::drawCircle(cv::Mat& frame, cv::Point center, cv::Scalar color)
{
    cv::circle(frame,center,5,color,2, cv::LINE_AA);
}

template <typename PREC>
void LaneKeepingSystem<PREC>::imageCallback(const sensor_msgs::Image& message)
{
    cv::Mat src = cv::Mat(message.height, message.width, CV_8UC3, const_cast<uint8_t*>(&message.data[0]), message.step);
    cv::cvtColor(src, mFrame, cv::COLOR_RGB2BGR);
    // cv::imshow(mFrame)
}

template <typename PREC>
void LaneKeepingSystem<PREC>::speedControl(PREC steeringAngle)
{
    if (std::abs(steeringAngle) > mXycarSpeedControlThreshold)
    {
        mXycarSpeed -= mDecelerationStep;
        mXycarSpeed = std::max(mXycarSpeed, mXycarMinSpeed);
        return;
    }

    mXycarSpeed += mAccelerationStep;
    mXycarSpeed = std::min(mXycarSpeed, mXycarMaxSpeed);
}

template <typename PREC>
void LaneKeepingSystem<PREC>::drive(PREC steeringAngle)
{
    xycar_msgs::xycar_motor motorMessage;
    motorMessage.angle = std::round(steeringAngle);
    speedControl(steeringAngle);
    motorMessage.speed = std::round(mXycarSpeed);
    mPublisher.publish(motorMessage);
}

template class LaneKeepingSystem<float>;
template class LaneKeepingSystem<double>;
} // namespace Xycar
