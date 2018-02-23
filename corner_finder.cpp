#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "nlohmann/json.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;

po::variables_map parse_command_line_params(int argc, char **argv) {
  po::options_description desc("Stereo viewer options");
  desc.add_options()
    ("left_camera", po::value<int>()->default_value(0), "Left camera id")
    ("right_camera", po::value<int>()->default_value(1), "Right camera id")
    ("board_width", po::value<int>()->default_value(9), "Number of columns on the checker board")
    ("board_height", po::value<int>()->default_value(6), "Number of rows on the checker board")
    ("square_size", po::value<float>()->default_value(18), "Checker borad grid cell size in mm")
    ("total_frames", po::value<int>()->default_value(50), "Number of calibration frames")
    ("output_path", po::value<std::string>()->default_value("./corners.json"), "Output folder for the detector corners");

  po::variables_map vm;

  try {
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help")) {
      std::cout << desc << std::endl;
      exit(1);
    }
  } catch (po::error &e) {
    std::cerr << "Error: " << e.what() << std::endl;
    std::cerr << desc << std::endl;
    exit(1);
  }

  return vm;
}

std::pair<bool, cv::Mat> detect_corners(cv::Mat &frame, cv::Size board_size) {
  cv::Mat gray;
  cv::Mat corners;
  cvtColor(frame, gray, CV_BGR2GRAY);
  bool found = findChessboardCorners(gray, board_size, corners,
                                     CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

  if (found) {
    cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1),
                 cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
    drawChessboardCorners(frame, board_size, corners, found);
  }

  return std::make_pair(found, corners);
}

using frame_object_points_t = std::vector<cv::Point3f>;
using frame_image_points_t = std::vector<cv::Point2f>;

// TODO: use only 1 set of object points
struct frame_points_t {
  int frame_index;
  frame_object_points_t object_points;
  frame_image_points_t image_points;
};

nlohmann::json convert_view_to_json(const frame_points_t& frame_points) {
  nlohmann::json j;
  j["index"] = frame_points.frame_index;

  nlohmann::json j_object_points = nlohmann::json::array();
  for(const auto& p : frame_points.object_points) {
    j_object_points.push_back({p.x, p.y, p.z});
  }
  j["object_points"] = j_object_points;

  nlohmann::json j_image_points = nlohmann::json::array();
  for(const auto& p : frame_points.image_points) {
    j_image_points.push_back({p.x, p.y});
  }
  j["image_points"] = j_image_points;
  return j;
}

nlohmann::json convert_frame_to_json(const std::map<std::string, frame_points_t>& frame) {
  nlohmann::json j;
  j["left"] = convert_view_to_json(frame.at("left"));
  j["right"] = convert_view_to_json(frame.at("right"));
  return j;
}

nlohmann::json convert_frames_to_json(const std::vector<std::map<std::string, frame_points_t>>& frames) {
  nlohmann::json j = nlohmann::json::array();
  for(const auto& frame : frames) {
    j.push_back(convert_frame_to_json(frame));
  }
  return j;
}

int main(int argc, char **argv) {
  auto vm = parse_command_line_params(argc, argv);

  //initialize and allocate memory to load the video stream from camera
  std::unique_ptr<cv::VideoCapture> cams[2] = {
    std::make_unique<cv::VideoCapture>(vm["left_camera"].as<int>()),
    std::make_unique<cv::VideoCapture>(vm["right_camera"].as<int>())
  };

  if (!(cams[0]->isOpened() && cams[1]->isOpened())) return 1;

  const int board_width = vm["board_width"].as<int>();
  const int board_height = vm["board_height"].as<int>();
  cv::Size board_size(board_width, board_height);

  const float square_size = vm["square_size"].as<float>();
  frame_object_points_t grid_points;
  for(int i=0;i<board_height;++i) {
    for(int j=0;j<board_width;++j) {
      grid_points.push_back(cv::Point3f(j * square_size, i * square_size, 0.f));
    }
  }

  std::vector<std::map<std::string, frame_points_t>> calibration_data;
  int frame_index = 0;
  const int total_frames = vm["total_frames"].as<int>();

  while (frame_index < total_frames) {
    //grab and retrieve each frames of the video sequentially
    cv::Mat3b frame0, frame1;
    *cams[0] >> frame0;
    *cams[1] >> frame1;

    bool found0, found1;
    cv::Mat corners0, corners1;
    std::tie(found0, corners0) = detect_corners(frame0, board_size);
    std::tie(found1, corners1) = detect_corners(frame1, board_size);

    cv::Mat frame;
    cv::hconcat(std::vector<cv::Mat>{frame0, frame1}, frame);
    cv::imshow("Corner finder", frame);

    //wait for 40 milliseconds
    int c = cvWaitKey(40);

    //exit the loop if user press "Esc" key  (ASCII value of "Esc" is 27)
    if (27 == char(c)) break;

    // save the data when press "Space"
    if(found0 && found1 && 32 == char(c)) {
      calibration_data.push_back(
        std::map<std::string, frame_points_t> {
          std::make_pair("left", frame_points_t{frame_index, grid_points, corners0}),
          std::make_pair("right", frame_points_t{frame_index, grid_points, corners1})
        }
      );

      ++frame_index;
      std::cout << frame_index << "/" << total_frames << " recorded." << std::endl;
    }
  }

  nlohmann::json j;
  j["frames"] = convert_frames_to_json(calibration_data);
  std::cout << j << std::endl;

  const std::string output_filename = vm["output_path"].as<std::string>();
  std::ofstream fout(output_filename);
  if(fout) {
    fout << j << std::endl;
  } else {
    std::cerr << "Failed to write " << output_filename << std::endl;
    exit(1);
  }

  return 0;
}
