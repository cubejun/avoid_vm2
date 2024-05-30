#include "avoid_vm/vm.hpp"
VM::VM() : Node("mysub")
{
  writer1.open("lidarsave.mp4", cv::VideoWriter::fourcc('D', 'I', 'V', 'X'), 30, cv::Size(500, 500));
  lidar_info_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS(), std::bind(&VM::scanCb, this, _1));

  pub_ = this->create_publisher<std_msgs::msg::Int32>("err", rclcpp::SensorDataQoS());
  timer_ = this->create_wall_timer(100ms, std::bind(&VM::publish_msg, this));
}

void VM::scanCb(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  int count = scan->scan_time / scan->time_increment;
  int x, y, rmindex, lmindex;
  float rmin, lmin, rx, ry, lx, ly, rd, ld;
  // printf("[SLLIDAR INFO]: I heard a laser scan %s[%d]:\n", scan->header.frame_id.c_str(), count);
  // printf("[SLLIDAR INFO]: angle_range : [%f, %f]\n", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
  cv::Mat img(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));
  
  lmin = scan->ranges[0];
  rmin = scan->ranges[539];
  for (int i = 0; i < count; i++) {
    float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
    // printf("[SLLIDAR INFO]: angle-distance : [%f, %f]\n", degree, scan->ranges[i]);
    x = 250 + scan->ranges[i]*100 * sin(degree*M_PI/180);
    y = 250 + scan->ranges[i]*100 * cos(degree*M_PI/180);

    
    // if(-180< degree && degree<=-90){
    //   if(lmin>=scan->ranges[i]){
    //     lmin=scan->ranges[i];
    //     lmindex = i;
    //     ld = degree;
    //   }
    // }
    // if(90 <= degree && degree < 180){
    //   if(rmin>=scan->ranges[i]){
    //     rmin=scan->ranges[i];
    //     rmindex = i;
    //     rd = degree;
    //   }
    // }
    cv::circle(img, cv::Point(x, y), 2, cv::Scalar(0, 0, 255), 2);
    // cv::line(img, cv::Point(250, 0), cv::Point(250, 250), cv::Scalar(0, 0, 0), 1);
    // cv::line(img, cv::Point(0, 250), cv::Point(500, 250), cv::Scalar(0, 0, 0), 1);
  }

	cv::Mat bin, gray;
  cv::Mat ROI = img(cv::Rect(0, 0, 500, 255));
  cv::cvtColor(ROI, gray, cv::COLOR_BGR2GRAY);
	cv::threshold(gray, bin, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);

	cv::Mat labels, stats, centroids;
	int cnt = cv::connectedComponentsWithStats(bin, labels, stats, centroids); // 레이블 영역의 통계, 무게 중심 좌표 정보

	// 각 객체 영역에 바운딩 박스 표시하기
	for (int i = 1;i < cnt;i++) {
		int* p = stats.ptr<int>(i);
		
		if (p[4] > 200) {
      rectangle(img, cv::Rect(p[0], p[1], p[2], p[3]), cv::Scalar(0, 255, 255)); // x,y,가로,세로 크기
      if(p[0]>250&&p[1]+ p[3]<250&&p[0]>0&&p[1]>0){
        rx = p[0];
        ry = p[1] + p[3];
      }
      if(p[0]+ p[2]<250&&p[1]+ p[3]<250&&p[0]>0&&p[1]>0){
        lx = p[0] + p[2];
        ly = p[1] + p[3];
      }
    }
		
	}
  // rx = 250 + rmin*50 * sin(rd*M_PI/180);
  // ry = 250 + rmin*50 * cos(rd*M_PI/180);
  // lx = 250 + lmin*50 * sin(ld*M_PI/180);
  // ly = 250 + lmin*50 * cos(ld*M_PI/180);
  err = RAD2DEG(atan((rx+lx-500)/(500-ry-ly)));
  // cv::circle(img, cv::Point(rx, ry), 5, cv::Scalar(255, 0, 0), 5);
  // cv::circle(img, cv::Point(lx, ly), 5, cv::Scalar(0, 255, 0), 5);
  cv::line(img, cv::Point(250, 250), cv::Point(rx, ry), cv::Scalar(255, 0, 0), 3);
  cv::line(img, cv::Point(250, 250), cv::Point(lx, ly), cv::Scalar(255, 0, 0), 3);
  cv::line(img, cv::Point(250, 250), cv::Point(rx-(250-lx), ry-(250-ly)), cv::Scalar(255, 255, 0), 3);
  printf("ld : %f, rd : %f, err : %d\n",ld, rd, err);
  writer1 << img;
  cv::imshow("img", img);
  cv::waitKey(1);
}
void VM::publish_msg()
{
  intmsg.data = err;
	RCLCPP_INFO(this->get_logger(), "Publish: %d", intmsg.data);
	pub_->publish(intmsg);
}