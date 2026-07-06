#ifndef D435I_VIDEO_CAPTURE_HPP
#define D435I_VIDEO_CAPTURE_HPP

#include <sstream>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <string>
#include <iostream>
#include <cmath>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

class RS_D435i
{
public:
    RS_D435i(int color_width, int color_height, int depth_width, int depth_heigh, int fps);
    ~RS_D435i() {};

    void init();
    void getImage(cv::Mat &color_dst, rs2::depth_frame &depth_dst, cv::Mat &depth_color_dst);

    cv::Point3f get3DPoint(const rs2::depth_frame &depth, cv::Point2f target);
    std::vector<cv::Point3f> get3DPoints(const rs2::depth_frame &depth, const std::vector<cv::Point2f> &target);
    float getDistance(const rs2::depth_frame &depth, cv::Point2f p0, cv::Point2f p1);
    const rs2_intrinsics &getIntrinColor() const;

private:

    int color_height;
    int color_width;
    int depth_height;
    int depth_width;
    int fps;

    float depth_clipping_distance;

    rs2::colorizer color_map;
    rs2::pipeline pipe;
    rs2::config pipe_config;
    rs2::pipeline_profile profile;
    rs2::video_stream_profile depth_stream;
    rs2::video_stream_profile color_stream;

    rs2_intrinsics intrinDepth;
    rs2_intrinsics intrinColor;
    rs2_extrinsics extrinDepth2Color;

    cv::Mat depth_image;
    cv::Mat depth_image_4_show;
    cv::Mat color_image;
    cv::Mat result;
    cv::Mat depth_in_color_z;

    float get_distance_with_neighboring_average(const rs2::depth_frame &depth, int x, int y, int radius = 3);
    float euclidean_dist(const cv::Point3f p0, const cv::Point3f p1);
};

#endif // D435I_VIDEO_CAPTURE_HPP