#ifndef ZED_CPU_ROS_TEARING_FILTER_HPP
#define ZED_CPU_ROS_TEARING_FILTER_HPP

#include <vector>
#include <math.h>

// OpenCV
#include <opencv2/opencv.hpp>


class TearingFilter {
private:
    int hough_threshold_;
    float atol_;

public:
    TearingFilter(int hough_threshold=300, float atol=0.01) :
            hough_threshold_(hough_threshold),
            atol_(atol)
    { }

    void setHoughThreshold(int hough_threshold) {
        this->hough_threshold_ = hough_threshold;
    }

    void setAtol(float atol) {
        this->atol_ = atol;
    }

    bool detectTearing(const cv::Mat& img) {
        cv::Mat dst;
        cv::Canny(img, dst, 50, 150, 3);
        std::vector<cv::Vec2f> lines;
        cv::HoughLines(dst, lines, 1, CV_PI/180, hough_threshold_, 0, 0 );

        bool found = false;
        for (auto& line : lines) {
            float rho = line[0], theta = line[1];
            cv::Point pt1, pt2;
            double a = cos(theta), b = sin(theta);
            double x0 = a*rho, y0 = b*rho;
            pt1.x = cvRound(x0 + 1000*(-b));
            pt1.y = cvRound(y0 + 1000*(a));
            pt2.x = cvRound(x0 - 1000*(-b));
            pt2.y = cvRound(y0 - 1000*(a));
            if (90.0-atol_ <= theta*180/CV_PI <= 90.0+atol_ && pt1.y+pt2.y > 0) {
                found = true;
            }
        }
        return found;
    }
};

#endif //ZED_CPU_ROS_TEAR_FILTER_HPP_H
