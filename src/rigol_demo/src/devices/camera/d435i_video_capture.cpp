#include "d435i_video_capture.hpp"

using namespace std;
using namespace cv;

RS_D435i::RS_D435i(int color_width, int color_height, int depth_width, int depth_height, int fps)
{
    this->color_width = color_width;
    this->color_height = color_height;
    this->depth_width = depth_width;
    this->depth_height = depth_height;
    this->fps = fps;
}

void RS_D435i::init()
{
    pipe_config.enable_stream(RS2_STREAM_DEPTH, depth_width, depth_height, RS2_FORMAT_Z16, fps);
    pipe_config.enable_stream(RS2_STREAM_COLOR, color_width, color_height, RS2_FORMAT_BGR8, fps);

    // start()函数返回数据管道的profile
    profile = pipe.start(pipe_config);

    // 定义一个变量去转换深度到距离
    depth_clipping_distance = 1.f;

    // 声明数据流
    depth_stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    color_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();

    // 获取内参
    intrinDepth = depth_stream.get_intrinsics();
    intrinColor = color_stream.get_intrinsics();

    // 直接获取从深度摄像头坐标系到彩色摄像头坐标系的欧式变换矩阵
    extrinDepth2Color = depth_stream.get_extrinsics_to(color_stream);
}

void RS_D435i::getImage(cv::Mat &color_dst, rs2::depth_frame &depth_dst, cv::Mat &depth_color_dst)
{
    rs2::align align_to_color(RS2_STREAM_COLOR); // 对齐到彩色流
    // 堵塞程序直到新的一帧捕获
    rs2::frameset frameset = pipe.wait_for_frames();
    rs2::frameset aligned_frames = align_to_color.process(frameset);

    // 取深度图和彩色图
    rs2::frame color_frame = frameset.get_color_frame();
    rs2::depth_frame aligned_depth = aligned_frames.get_depth_frame().as<rs2::depth_frame>();

    //  获取宽高
    const int color_w = color_frame.as<rs2::video_frame>().get_width();
    const int color_h = color_frame.as<rs2::video_frame>().get_height();

    rs2::decimation_filter dec_filter;                     // 减小分辨率，去高频噪声
    dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2); // 1..8

    rs2::spatial_filter spat_filter;                              // 空间滤波：保边平滑
    spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.5f); // 0..1
    spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20);

    rs2::temporal_filter temp_filter; // 时间滤波：随时间平滑（针对随机噪声）
    temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.4f);
    temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20);

    rs2::hole_filling_filter hole_filter;             // 填充空洞（0值）
    hole_filter.set_option(RS2_OPTION_HOLES_FILL, 2); // 1 or 2 (aggressive)

    // depth_frame = spat_filter.process(depth_frame);
    // depth_frame = temp_filter.process(depth_frame);
    aligned_depth = hole_filter.process(aligned_depth);

    // 创建OPENCV类型 并传入数据
    color_image = Mat(Size(color_w, color_h),
                      CV_8UC3, (void *)color_frame.get_data(), Mat::AUTO_STEP);
    rs2::frame depth_colorized = aligned_depth.apply_filter(color_map);

    const int depth_w = depth_colorized.as<rs2::video_frame>().get_width();
    const int depth_h = depth_colorized.as<rs2::video_frame>().get_height();
    depth_image = Mat(Size(depth_w, depth_h),
                      CV_8UC3, (void *)depth_colorized.get_data(), Mat::AUTO_STEP);

    color_dst = color_image.clone();
    depth_dst = aligned_depth;
    depth_color_dst = depth_image.clone();
}

cv::Point3f RS_D435i::get3DPoint(const rs2::depth_frame &depth, cv::Point2f target)
{
    cv::Point3f result_cord(0.0f, 0.0f, 0.0f);
    int x0 = target.x;
    int y0 = target.y;
    float d0 = depth.get_distance(x0, y0);
    if (d0 == 0.0f)
    {
        d0 = get_distance_with_neighboring_average(depth, x0, y0, 2);
    }
    if (d0 > 0.0f)
    {
        float p0[3];
        float pix0[2] = {(float)x0, (float)y0};
        rs2_deproject_pixel_to_point(p0, &intrinColor, pix0, d0);
        result_cord.x = p0[0];
        result_cord.y = p0[1];
        result_cord.z = p0[2];
    }
    return result_cord;
}

std::vector<cv::Point3f> RS_D435i::get3DPoints(const rs2::depth_frame &depth, const std::vector<cv::Point2f> &target)
{
    std::vector<cv::Point3f> result_pts;
    for (size_t i = 0; i < target.size(); ++i)
    {
        result_pts.push_back(get3DPoint(depth, target[i]));
    }
    return result_pts;
}

float RS_D435i::getDistance(const rs2::depth_frame &depth, cv::Point2f p0, cv::Point2f p1)
{
    cv::Point3f px = get3DPoint(depth, p0);
    cv::Point3f py = get3DPoint(depth, p1);
    if (px != cv::Point3f(0.0f, 0.0f, 0.0f) && py != cv::Point3f(0.0f, 0.0f, 0.0f))
    {
        return euclidean_dist(px, py);
    }
    else
    {
        return 0.0f;
    }
}

const rs2_intrinsics &RS_D435i::getIntrinColor() const
{
    return intrinColor;
}

float RS_D435i::get_distance_with_neighboring_average(const rs2::depth_frame &depth, int x, int y, int radius)
{
    int w = depth.get_width();
    int h = depth.get_height();
    float sum = 0.0f;
    int cnt = 0;
    for (int yy = std::max(0, y - radius); yy <= std::min(h - 1, y + radius); yy++)
    {
        for (int xx = std::max(0, x - radius); xx <= std::min(w - 1, x + radius); xx++)
        {
            float d = depth.get_distance(xx, yy);
            if (d > 0 && !std::isnan(d))
            {
                sum += d;
                cnt++;
            }
        }
    }
    if (cnt == 0)
    {
        return 0.0f;
    }
    return sum / cnt;
}

float RS_D435i::euclidean_dist(const cv::Point3f p0, const cv::Point3f p1)
{
    cv::Point3f tmp = p0 - p1;
    return std::sqrt(tmp.x * tmp.x + tmp.y * tmp.y + tmp.z * tmp.z);
}
