#include "chrono"
#include "opencv2/opencv.hpp"
#include "yolov8-seg.hpp"
#include <ros/ros.h>

#include "image_transport/image_transport.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CompressedImage.h"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include "fsd_common_msgs/YoloCone.h"
#include "fsd_common_msgs/YoloConeDetections.h"
#include "fsd_common_msgs/img_pro_info.h"
#include "std_msgs/Float32.h"

const std::vector<std::string> CLASS_NAMES = {
    "person",         "bicycle",    "car",           "motorcycle",    "airplane",     "bus",           "train",
    "truck",          "boat",       "traffic light", "fire hydrant",  "stop sign",    "parking meter", "bench",
    "bird",           "cat",        "dog",           "horse",         "sheep",        "cow",           "elephant",
    "bear",           "zebra",      "giraffe",       "backpack",      "umbrella",     "handbag",       "tie",
    "suitcase",       "frisbee",    "skis",          "snowboard",     "sports ball",  "kite",          "baseball bat",
    "baseball glove", "skateboard", "surfboard",     "tennis racket", "bottle",       "wine glass",    "cup",
    "fork",           "knife",      "spoon",         "bowl",          "banana",       "apple",         "sandwich",
    "orange",         "broccoli",   "carrot",        "hot dog",       "pizza",        "donut",         "cake",
    "chair",          "couch",      "potted plant",  "bed",           "dining table", "toilet",        "tv",
    "laptop",         "mouse",      "remote",        "keyboard",      "cell phone",   "microwave",     "oven",
    "toaster",        "sink",       "refrigerator",  "book",          "clock",        "vase",          "scissors",
    "teddy bear",     "hair drier", "toothbrush"};

const std::vector<std::vector<unsigned int>> COLORS = {
    {0, 114, 189},   {217, 83, 25},   {237, 177, 32},  {126, 47, 142},  {119, 172, 48},  {77, 190, 238},
    {162, 20, 47},   {76, 76, 76},    {153, 153, 153}, {255, 0, 0},     {255, 128, 0},   {191, 191, 0},
    {0, 255, 0},     {0, 0, 255},     {170, 0, 255},   {85, 85, 0},     {85, 170, 0},    {85, 255, 0},
    {170, 85, 0},    {170, 170, 0},   {170, 255, 0},   {255, 85, 0},    {255, 170, 0},   {255, 255, 0},
    {0, 85, 128},    {0, 170, 128},   {0, 255, 128},   {85, 0, 128},    {85, 85, 128},   {85, 170, 128},
    {85, 255, 128},  {170, 0, 128},   {170, 85, 128},  {170, 170, 128}, {170, 255, 128}, {255, 0, 128},
    {255, 85, 128},  {255, 170, 128}, {255, 255, 128}, {0, 85, 255},    {0, 170, 255},   {0, 255, 255},
    {85, 0, 255},    {85, 85, 255},   {85, 170, 255},  {85, 255, 255},  {170, 0, 255},   {170, 85, 255},
    {170, 170, 255}, {170, 255, 255}, {255, 0, 255},   {255, 85, 255},  {255, 170, 255}, {85, 0, 0},
    {128, 0, 0},     {170, 0, 0},     {212, 0, 0},     {255, 0, 0},     {0, 43, 0},      {0, 85, 0},
    {0, 128, 0},     {0, 170, 0},     {0, 212, 0},     {0, 255, 0},     {0, 0, 43},      {0, 0, 85},
    {0, 0, 128},     {0, 0, 170},     {0, 0, 212},     {0, 0, 255},     {0, 0, 0},       {36, 36, 36},
    {73, 73, 73},    {109, 109, 109}, {146, 146, 146}, {182, 182, 182}, {219, 219, 219}, {0, 114, 189},
    {80, 183, 189},  {128, 128, 0}};

const std::vector<std::vector<unsigned int>> MASK_COLORS = {
    {255, 56, 56},  {255, 157, 151}, {255, 112, 31}, {255, 178, 29}, {207, 210, 49},  {72, 249, 10}, {146, 204, 23},
    {61, 219, 134}, {26, 147, 52},   {0, 212, 187},  {44, 153, 168}, {0, 194, 255},   {52, 69, 147}, {100, 115, 255},
    {0, 24, 236},   {132, 56, 255},  {82, 0, 133},   {203, 56, 255}, {255, 149, 200}, {255, 55, 199}};

class SubscribeAndPublish
{
public:
    
    std::string path;
    std::string engine_file_path;
    YOLOv8_seg* yolov8;

    SubscribeAndPublish()
    {
        result_pub_ = n_.advertise<fsd_common_msgs::YoloConeDetections>("/yolov8_seg/boundingboxes", 1);
        image_transport::ImageTransport it_(n_);
        pub_ = it_.advertise("/yolov8_seg/mask", 1);//<sensor_msgs::Image>
        sub_ = it_.subscribe("/usb_cam/image_raw", 1, &SubscribeAndPublish::callback, this);
    }

    void callback(const sensor_msgs::ImageConstPtr& msg)
    {
        //make a message to save
        fsd_common_msgs::YoloConeDetections predict_result_msgs;
        fsd_common_msgs::YoloCone result;

        cv::Mat             res, img, res_mask;
        cv::Size            size = cv::Size{640, 640};
        int      topk         = 100;
        int      seg_h        = 160;
        int      seg_w        = 160;
        int      seg_channels = 32;
        float    score_thres  = 0.25f;
        float    iou_thres    = 0.65f;
        std::vector<Object> objs;
        cv_bridge::CvImage cv_ptr_seg;

        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        img = cv_ptr->image;

        //clear last answer
        objs.clear();

        yolov8->copy_from_Mat(img, size);

        // //get the detect start time
        auto start = std::chrono::system_clock::now();

        yolov8->infer();

        yolov8->postprocess(objs, score_thres, iou_thres, topk, seg_channels, seg_h, seg_w);
        yolov8->draw_objects(img, res, objs, CLASS_NAMES, COLORS, MASK_COLORS);
        yolov8->draw_mask(img, res_mask, objs, CLASS_NAMES, COLORS, MASK_COLORS);

        sensor_msgs::ImagePtr msg_mask = cv_bridge::CvImage(std_msgs::Header(), "bgr8", res_mask).toImageMsg();
 
        pub_.publish(msg_mask);

        for (int i=0; i<objs.size(); i++) {
            // printf("---\n");
            // printf("x       = %f\n", objs[i].rect.x);
            // printf("y       = %f\n", objs[i].rect.y);
            // printf("width   = %f\n", objs[i].rect.width);
            // printf("height  = %f\n", objs[i].rect.height);
            // printf("label   = %d\n", objs[i].label);
            // printf("prob    = %f\n", objs[i].prob);

            result.x.data       = objs[i].rect.x;
            result.y.data       = objs[i].rect.y;
            result.width.data   = objs[i].rect.width;
            result.height.data  = objs[i].rect.height;
            result.label.data   = objs[i].label;
            result.prob.data    = objs[i].prob;
            
            predict_result_msgs.cone_detections.push_back(result);
        }

        result_pub_.publish(predict_result_msgs);

        //show the video with boundingboxs
        // cv::imshow("result_img", res_mask);
        // cv::waitKey(1);

        //get the detect end time
        auto end = std::chrono::system_clock::now();
        
        //get the detect time
        auto tc = (double)std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.;

        //print detect time
        printf("cost %.4lf ms\n", tc);
    }

private:
    ros::NodeHandle n_;
    ros::Publisher result_pub_;
    //image_transport::ImageTransport it_(n_);
    image_transport::Publisher pub_;
    //ros::Subscriber sub_;
    image_transport::Subscriber sub_;

};


int main(int argc, char** argv)
{
    // cuda:0
    cudaSetDevice(0);

    const std::string engine_file_path{argv[1]};
    const std::string path{argv[2]};

    int a = 10;

    assert(argc == 3);

    auto yolov8 = new YOLOv8_seg(engine_file_path);

    yolov8->make_pipe(true);

    // cv::namedWindow("result_img", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("result_img", cv::WINDOW_KEEPRATIO);

    //make a ROS node "yolov8_node"
    ros::init(argc, argv, "yolov8_node");

    SubscribeAndPublish SAPObject;

    SAPObject.path = path;
    SAPObject.engine_file_path = engine_file_path;
    SAPObject.yolov8 = yolov8;

    ros::spin();

    cv::destroyAllWindows();

    delete yolov8;

    return 0;
}
