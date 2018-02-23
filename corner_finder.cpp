#include <boost/filesystem/path.hpp>
#include <boost/program_options.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace po = boost::program_options;
namespace fs = boost::filesystem;

po::variables_map parse_command_line_params(int argc, char **argv) {
  po::options_description desc("Stereo viewer options");
  desc.add_options()
    ("left_camera", po::value<int>()->default_value(0), "Left camera id")
    ("right_camera", po::value<int>()->default_value(1), "Right camera id")
    ("board_width", po::value<int>()->default_value(9), "Number of columns on the checker board")
    ("board_height", po::value<int>()->default_value(6), "Number of rows on the checker board");

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

  while (true) {
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
  }

  return 0;
}
