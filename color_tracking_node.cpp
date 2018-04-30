#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ctime>
#include <fstream>
#include <iostream>

using namespace std;

//#define MEASURE_TIME 1
#define COLOR_TRACKING 1

#ifdef MEASURE_TIME
clock_t t_begin = 0;
#endif

int upperBuffer = 0;
int lowerBuffer = 0;
void cv_process_img(const cv::Mat& input_img, cv::Mat& output_img)
{
	cv::Mat gray_img;
	cv::cvtColor(input_img, gray_img, CV_RGB2GRAY);

	double t1 = 20;
	double t2 = 50;
	int apertureSize = 3;

	cv::Canny(gray_img, output_img, t1, t2, apertureSize);
}

void cv_publish_img(image_transport::Publisher &pub, cv::Mat& pub_img)
{
	//sensor_msgs::ImagePtr pub_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", pub_img).toImageMsg();
	sensor_msgs::ImagePtr pub_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", pub_img).toImageMsg();
	pub.publish(pub_msg);
}

double round(double number)
{
    return number < 0.0 ? ceil(number - 0.5) : floor(number + 0.5);
}

void gen_random(char *s, const int len) {
    static const char alphanum[] =
        "0123456789"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz";

    for (int i = 0; i < len; ++i) {
        s[i] = alphanum[rand() % (sizeof(alphanum) - 1)];
    }

    s[len] = 0;
}

void cv_color_tracking(const cv::Mat& bgr_image)
{

    cv::Mat orig_image = bgr_image.clone();

    cv::medianBlur(bgr_image, bgr_image, 3);

// Convert input image to HSV
    cv::Mat hsv_image;
    cv::cvtColor(bgr_image, hsv_image, cv::COLOR_BGR2HSV);

    // Threshold the HSV image, keep only the red pixels
    cv::Mat lower_red_hue_range;
    cv::Mat upper_red_hue_range;
    cv::inRange(hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_hue_range);
    cv::inRange(hsv_image, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), upper_red_hue_range);

// the following range might be too small
//    cv::inRange(hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(10, 155, 155), lower_red_hue_range);
//    cv::inRange(hsv_image, cv::Scalar(40, 100, 100), cv::Scalar(50, 155, 155), upper_red_hue_range);

//0 100 100 | 10 255 255 | 160 100 100 | 179 255 255

    // Combine the above two images
    cv::Mat red_hue_image;
    cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);

    cv::GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);

    // Use the Hough transform to detect circles in the combined threshold image
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(red_hue_image, circles, CV_HOUGH_GRADIENT, 1, red_hue_image.rows/8, 100, 20, 0, 0);

     // std round is not working

     // creating file to maintain directional output
     ofstream myfile;
     myfile.open ("/home/nvidia/circle.txt");


    // Loop over all detected circles and outline them on the original image
    //if(circles.size() == 0) std::exit(-1);
    for(size_t current_circle = 0; current_circle < circles.size(); ++current_circle) {
        cv::Point center(round(circles[current_circle][0]), round(circles[current_circle][1]));
        int radius = round(circles[current_circle][2]);

        // draws a rectangle outline or a filled rectangle
        // whose two opposite corners are pt1 and pt2

        cv::Point pt1(center.x-radius,center.y-radius);
        cv::Point pt2(center.x+radius,center.y+radius);

        cv::rectangle(orig_image, pt1, pt2, cv::Scalar(0, 255, 0), 5);

        cv::Point pt3(center.x-radius / 2, center.y+radius / 2);
        cv::Point pt4(center.x+radius / 2, center.y-radius / 2);
        cv::Point pt5(center.x+radius / 2, center.y+radius / 2);
        cv::Point pt6(center.x-radius / 2, center.y-radius / 2);


        cv::line(orig_image, pt3, pt4, cv::Scalar(0, 255, 0), 5);
        cv::line(orig_image, pt5, pt6, cv::Scalar(0, 255, 0), 5);

        // Detect position of car relative to circle
        int width = orig_image.cols;

        int third = width / 3.0;

	char* myID = new char;
	gen_random(myID, 5);
	if(radius >= 50){
		upperBuffer++;
	}

	if(radius < 40){
		lowerBuffer++;
	}

	if(lowerBuffer > 10){
		upperBuffer = 0;
		lowerBuffer = 0;
	}

	cout << "RADIUS: " << radius << endl;
        if(center.x < third){
           // car is right of circle
           cout << "Right" << endl;
	//if(radius >= 50 || twentyPlusCount > 4){
	   if(upperBuffer >= 3){
	   	myfile << "ID: " << myID << " Command: Right" << "\n";
		upperBuffer = 0;
		lowerBuffer = 0;
	   }
        }else if(center.x < third*2){
           // car is in front of circle
           cout << "Front" << endl;
	   if(upperBuffer >= 3){
                myfile << "ID: " << myID << " Command: Front" << "\n";
		upperBuffer = 0;
		lowerBuffer = 0;
           }
        }else{
           // car is left of circle
          cout << "Left" << endl;
	  if(upperBuffer >= 3){
                myfile << "ID: " << myID << " Command: Left" << "\n";
		upperBuffer = 0;
		lowerBuffer = 0;
           }
       }

    }

    myfile.close();

    // Show images
    //cv::imshow("Detected red circles on the input image", orig_image);

    cv::waitKey(1);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg, image_transport::Publisher &pub)
{


	cv_bridge::CvImageConstPtr cv_ori_img_ptr;
	try{
		cv::Mat cv_ori_img = cv_bridge::toCvShare(msg, "bgr8")->image;
		cv::Mat cv_output_img;

		cv_process_img(cv_ori_img, cv_output_img);

#ifdef COLOR_TRACKING
		cv_color_tracking(cv_ori_img);
#endif

		cv_publish_img(pub, cv_output_img);

#ifdef MEASURE_TIME
		clock_t t_end = clock();
		double delta_time= double(t_end - t_begin) / CLOCKS_PER_SEC;
		cout << "Delta_t = " << 1/delta_time << "\n";
		//t_begin = t_end;
#endif

		//cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
		//cv::imshow("view", cv_output_img);
		cv::waitKey(30);

#ifdef MEASURE_TIME
		t_begin = clock();
#endif
	}catch(cv_bridge::Exception& e){
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}


}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");

	ros::NodeHandle nh;

	//cv::namedWindow("view");
	//cv::startWindowThread();
	image_transport::ImageTransport it(nh);

	ros::NodeHandle nh_pub;
	image_transport::ImageTransport itpub(nh_pub);
	image_transport::Publisher pub = itpub.advertise("sample/cannyimg", 1);

#ifdef MEASURE_TIME
	t_begin = clock();
#endif

	image_transport::Subscriber sub = it.subscribe("rgb/image_rect_color", 1, boost::bind(imageCallback, _1, pub));
	ros::spin();
	// cv::destroyWindow("view");

}
