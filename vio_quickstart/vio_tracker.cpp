#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/imu.pb.h>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <chrono>
#include <vector>

static const std::string CAMERA_TOPIC = "/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image";
static const std::string IMU_TOPIC = "/world/iris_runway/model/iris_with_standoffs/link/imu_link/sensor/imu_sensor/imu";
static constexpr double HFOV_RAD = 1.0471975512; // approximate Gazebo camera horizontal FOV (60 deg)
static constexpr double GRAVITY = 9.80665;

std::mutex mtx;
std::ofstream cameraCsv, imuCsv, vioCsv;
cv::Mat prevGray;
std::vector<cv::Point2f> prevPts;
bool havePrev = false;
double lastCamT = -1.0, lastImuT = -1.0, yaw = 0.0, x = 0.0, y = 0.0;
double gyroX = 0.0, gyroY = 0.0, gyroZ = 0.0;
unsigned long long cameraCount = 0, imuCount = 0;
double cameraHeight = 3.0;

static double stamp(const gz::msgs::Header &h) {
  return static_cast<double>(h.stamp().sec()) + 1e-9 * static_cast<double>(h.stamp().nsec());
}

void OnImu(const gz::msgs::IMU &msg) {
  const double t = stamp(msg.header());
  std::lock_guard<std::mutex> lock(mtx);
  gyroX = msg.angular_velocity().x();
  gyroY = msg.angular_velocity().y();
  gyroZ = msg.angular_velocity().z();
  if (imuCsv.is_open()) {
    imuCsv << t << ',' << gyroX << ',' << gyroY << ',' << gyroZ << ','
           << msg.linear_acceleration().x() << ',' << msg.linear_acceleration().y() << ',' << msg.linear_acceleration().z() << '\n';
  }
  ++imuCount;
  lastImuT = t;
  if (imuCount % 1000 == 0) std::cout << "[IMU] samples=" << imuCount << " t=" << t << "s\n";
}

void OnImage(const gz::msgs::Image &msg) {
  const double t = stamp(msg.header());
  const int w = static_cast<int>(msg.width()), h = static_cast<int>(msg.height());
  const auto &data = msg.data();
  if (w <= 0 || h <= 0 || data.size() < static_cast<size_t>(w*h*3)) {
    std::cerr << "Unexpected camera buffer (" << w << "x" << h << ", bytes=" << data.size() << ")\n";
    return;
  }
  cv::Mat rgb(h, w, CV_8UC3, const_cast<char*>(data.data()), msg.step());
  cv::Mat gray;
  // Gazebo camera stream in this world is RGB8 (step=width*3).
  cv::cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);

  std::lock_guard<std::mutex> lock(mtx);
  double flowX = 0.0, flowY = 0.0;
  size_t tracked = 0;
  if (havePrev && !prevPts.empty()) {
    std::vector<cv::Point2f> nextPts;
    std::vector<unsigned char> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(prevGray, gray, prevPts, nextPts, status, err);
    std::vector<double> dx, dy;
    for (size_t i=0; i<status.size(); ++i) {
      if (status[i] && std::isfinite(nextPts[i].x) && std::isfinite(nextPts[i].y)) {
        dx.push_back(static_cast<double>(nextPts[i].x - prevPts[i].x));
        dy.push_back(static_cast<double>(nextPts[i].y - prevPts[i].y));
      }
    }
    tracked = dx.size();
    if (tracked >= 5) {
      auto median = [](std::vector<double> v) {
        const size_t m = v.size()/2;
        std::nth_element(v.begin(), v.begin()+m, v.end());
        return v[m];
      };
      flowX = median(dx);
      flowY = median(dy);
    }
    const double dt = (lastCamT >= 0.0) ? std::max(0.0, t-lastCamT) : 0.0;
    // A lightweight relative VIO baseline: integrate gyro yaw and convert image
    // translation to approximate ground displacement using a fixed camera height.
    // This is not a replacement for a calibrated full 6-DoF estimator (e.g. OpenVINS).
    yaw += gyroZ * dt;
    const double fx = static_cast<double>(w) / (2.0 * std::tan(HFOV_RAD/2.0));
    const double dxBody = -flowX * cameraHeight / fx;
    const double dyBody =  flowY * cameraHeight / fx;
    x += std::cos(yaw)*dxBody - std::sin(yaw)*dyBody;
    y += std::sin(yaw)*dxBody + std::cos(yaw)*dyBody;
  }
  cameraCsv << t << ',' << w << ',' << h << ',' << msg.step() << ',' << data.size() << '\n';
  vioCsv << t << ',' << x << ',' << y << ',' << yaw << ',' << flowX << ',' << flowY << ',' << tracked << '\n';
  cameraCsv.flush(); vioCsv.flush();
  prevGray = gray.clone();
  prevPts.clear();
  cv::goodFeaturesToTrack(prevGray, prevPts, 250, 0.01, 7.0);
  havePrev = true;
  lastCamT = t;
  ++cameraCount;
  if (cameraCount % 10 == 0) std::cout << "[CAMERA/VIO] frames=" << cameraCount << " features=" << tracked << " xy=(" << x << "," << y << ")m\n";
}

int main(int argc, char **argv) {
  if (argc > 1) cameraHeight = std::max(0.2, std::atof(argv[1]));
  cameraCsv.open("camera_log.csv");
  imuCsv.open("imu_log.csv");
  vioCsv.open("vio_estimate.csv");
  if (!cameraCsv || !imuCsv || !vioCsv) { std::cerr << "Cannot open CSV output files in current directory\n"; return 1; }
  cameraCsv << "timestamp_sec,width,height,step_bytes,data_bytes\n";
  imuCsv << "timestamp_sec,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z\n";
  vioCsv << "timestamp_sec,x_m,y_m,yaw_rad,median_flow_dx_px,median_flow_dy_px,tracked_features\n";

  gz::transport::Node node;
  const bool camOk = node.Subscribe(CAMERA_TOPIC, &OnImage);
  const bool imuOk = node.Subscribe(IMU_TOPIC, &OnImu);
  if (!camOk || !imuOk) { std::cerr << "Failed to subscribe to Gazebo topics. Check world/topic names.\n"; return 2; }
  std::cout << "Subscribed to Iris camera and IMU.\n"
            << "Camera height used for approximate scale: " << cameraHeight << " m\n"
            << "Writing camera_log.csv, imu_log.csv, and vio_estimate.csv\n"
            << "This is a lightweight optical-flow + gyro baseline, not full calibrated OpenVINS.\n"
            << "Keep Gazebo running; press Ctrl+C to stop.\n";
  while (true) std::this_thread::sleep_for(std::chrono::seconds(1));
  return 0;
}
