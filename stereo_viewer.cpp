#include <boost/filesystem/path.hpp>
#include <boost/program_options.hpp>
#include <opencv2/opencv.hpp>

namespace po = boost::program_options;
namespace fs = boost::filesystem;

po::variables_map parse_command_line_params(int argc, char** argv) {
  po::options_description desc("Stereo viewer options");
  desc.add_options()
    ("left_camera", po::value<int>()->default_value(0), "Left camera id")
    ("right_camera", po::value<int>()->default_value(1), "Right camera id");

  po::variables_map vm;

  try {
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if(vm.count("help")) {
      std::cout << desc << std::endl;
      exit(1);
    }
  } catch(po::error& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    std::cerr << desc << std::endl;
    exit(1);
  }

  return vm;
}

int main(int argc, char** argv)
{
  auto vm = parse_command_line_params(argc, argv);

  //initialize and allocate memory to load the video stream from camera
  std::unique_ptr<cv::VideoCapture> cams[2] = {
    std::make_unique<cv::VideoCapture>(vm["left_camera"].as<int>()),
    std::make_unique<cv::VideoCapture>(vm["right_camera"].as<int>())
  };

  if( !(cams[0]->isOpened() && cams[1]->isOpened()) ) return 1;

  while(true) {
    //grab and retrieve each frames of the video sequentially
    cv::Mat3b frame0, frame1;
    *cams[0] >> frame0;
    *cams[1] >> frame1;

    cv::Mat frame;
    cv::hconcat(std::vector<cv::Mat>{frame0, frame1}, frame);
    cv::imshow("Corner finder", frame);

    //wait for 40 milliseconds
    int c = cvWaitKey(40);

    //exit the loop if user press "Esc" key  (ASCII value of "Esc" is 27)
    if(27 == char(c)) break;
  }

  return 0;
}
