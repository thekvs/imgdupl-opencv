#include <iostream>
#include <fstream>
#include <string>

#include <opencv2/opencv.hpp>

int
main(int argc, char **argv)
{
    if (argc < 3) {
        std::cout << "Usage: " << argv[0] << " <filename> <x-size>" << std::endl;
        exit(EXIT_SUCCESS);
    }

    std::string filename = argv[1];
    int         x_size = atoi(argv[2]);

    cv::Mat orig_image = cv::imread(filename.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
    if (orig_image.empty()) {
        std::cerr << "Couldn't read an image" << std::endl;
        return EXIT_FAILURE;
    }

    double ratio = static_cast<double>(x_size) / orig_image.cols;

    cv::Mat image;

    cv::resize(orig_image, image, cv::Size(), ratio, ratio, cv::INTER_CUBIC);

    cv::namedWindow("Resized image");
    cv::imshow("Resized image", image);
    cv::waitKey();

    return EXIT_SUCCESS;
}
