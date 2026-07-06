#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include "opencv2/opencv.hpp"
#include "devices/camera/d435i_video_capture.hpp"


struct MouseState
{
    cv::Point point{0, 0};
    bool valid = false;
    float depth_m = 0.0f;
};

struct ProgramOptions
{
    int color_width = 1280;
    int color_height = 720;
    int depth_width = 1280;
    int depth_height = 720;
    int fps = 30;
    std::string rgb_window = "D435i RGB";
    std::string depth_window = "D435i Depth (Pseudo Color)";
};

static void mouseCallback(int event, int x, int y, int /*flags*/, void *userdata)
{
    if (event == cv::EVENT_MOUSEMOVE || event == cv::EVENT_LBUTTONDOWN)
    {
        auto *state = static_cast<MouseState *>(userdata);
        state->point = cv::Point(x, y);
        state->valid = true;
    }
}

static float queryDepthAtPoint(RS_D435i &camera, const rs2::depth_frame &depth_frame, const cv::Point &point)
{
    if (!depth_frame)
    {
        return 0.0f;
    }
    const int x = std::clamp(point.x, 0, depth_frame.get_width() - 1);
    const int y = std::clamp(point.y, 0, depth_frame.get_height() - 1);
    float depth_m = depth_frame.get_distance(x, y);
    if (depth_m <= 0.0f)
    {
        const cv::Point2f uv(static_cast<float>(x), static_cast<float>(y));
        const cv::Point3f p3 = camera.get3DPoint(depth_frame, uv);
        depth_m = p3.z;
    }
    return depth_m;
}

static void printUsage(const char *program_name)
{
    std::cout << "Usage: " << program_name << " [options]\n"
              << "Options:\n"
              << "  --color_width=N        RGB image width (default: 1280)\n"
              << "  --color_height=N       RGB image height (default: 720)\n"
              << "  --depth_width=N        Depth image width (default: 1280)\n"
              << "  --depth_height=N       Depth image height (default: 720)\n"
              << "  --fps=N                Camera fps (default: 30)\n"
              << "  --rgb_window=NAME      RGB window name (default: D435i RGB)\n"
              << "  --depth_window=NAME    Depth window name (default: D435i Depth (Pseudo Color))\n"
              << "  --help                 Show this help message\n";
}

static bool parseIntOption(const std::string &arg, const std::string &name, int &value)
{
    const std::string prefix = "--" + name + "=";
    if (arg.rfind(prefix, 0) != 0)
    {
        return false;
    }

    try
    {
        value = std::stoi(arg.substr(prefix.size()));
        return true;
    }
    catch (const std::exception &)
    {
        std::cerr << "Invalid value for " << name << ": " << arg << std::endl;
        std::exit(1);
    }
}

static bool parseStringOption(const std::string &arg, const std::string &name, std::string &value)
{
    const std::string prefix = "--" + name + "=";
    if (arg.rfind(prefix, 0) != 0)
    {
        return false;
    }

    value = arg.substr(prefix.size());
    return true;
}

static ProgramOptions parseArguments(int argc, char **argv)
{
    ProgramOptions options;

    for (int i = 1; i < argc; ++i)
    {
        const std::string arg(argv[i]);
        if (arg == "--help")
        {
            printUsage(argv[0]);
            std::exit(0);
        }

        if (parseIntOption(arg, "color_width", options.color_width) ||
            parseIntOption(arg, "color_height", options.color_height) ||
            parseIntOption(arg, "depth_width", options.depth_width) ||
            parseIntOption(arg, "depth_height", options.depth_height) ||
            parseIntOption(arg, "fps", options.fps) ||
            parseStringOption(arg, "rgb_window", options.rgb_window) ||
            parseStringOption(arg, "depth_window", options.depth_window))
        {
            continue;
        }

        std::cerr << "Unknown argument: " << arg << std::endl;
        printUsage(argv[0]);
        std::exit(1);
    }

    return options;
}

int main(int argc, char **argv)
{
    const ProgramOptions options = parseArguments(argc, argv);

    RS_D435i camera(options.color_width, options.color_height, options.depth_width, options.depth_height, options.fps);
    camera.init();

    std::cout << "D435i camera initialized with color resolution " << options.color_width << 'x' << options.color_height
              << ", depth resolution " << options.depth_width << 'x' << options.depth_height
              << ", fps=" << options.fps << std::endl;

    cv::namedWindow(options.rgb_window, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(options.depth_window, cv::WINDOW_AUTOSIZE);

    MouseState mouse_state;
    cv::setMouseCallback(options.rgb_window, mouseCallback, &mouse_state);

    cv::Mat rgb_image;
    rs2::depth_frame depth_frame = rs2::depth_frame(nullptr);
    cv::Mat depth_color_image;

    while (true)
    {
        camera.getImage(rgb_image, depth_frame, depth_color_image);
        if (rgb_image.empty() || depth_color_image.empty())
        {
            std::cout << "Waiting for valid camera frames..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        if (mouse_state.valid)
        {
            mouse_state.depth_m = queryDepthAtPoint(camera, depth_frame, mouse_state.point);
            std::ostringstream text;
            text << std::fixed << std::setprecision(3) << mouse_state.depth_m << " m";
            cv::putText(rgb_image, text.str(), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
            cv::circle(rgb_image, mouse_state.point, 4, cv::Scalar(0, 255, 0), cv::FILLED);
        }
        else
        {
            cv::putText(rgb_image, "Move mouse over image to read depth", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
        }

        cv::imshow(options.rgb_window, rgb_image);
        cv::imshow(options.depth_window, depth_color_image);

        const int key = cv::waitKey(1);
        if (key == 27 || key == 'q' || key == 'Q')
        {
            break;
        }
        cv::waitKey(10);
    }

    cv::destroyAllWindows();
    return 0;
}
