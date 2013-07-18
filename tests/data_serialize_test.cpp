#include <iostream>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

typedef cv::Ptr<cv::FeatureDetector>     FeatureDetectorPtr;
typedef cv::Ptr<cv::DescriptorExtractor> DescriptorExtractorPtr;

typedef std::vector<cv::KeyPoint> KeyPoints;

void
usage(const char *program)
{
    std::cout << "Usage: " << program << " <image>" << std::endl;
    exit(EXIT_SUCCESS);
}

int
main(int argc, char **argv)
{
    cv::initModule_nonfree();

    if (argc < 2) {
        usage(argv[0]);
    }

    std::string filename = argv[1];

    FeatureDetectorPtr     detector = cv::Ptr<cv::FeatureDetector>(new cv::SurfFeatureDetector(400));
    DescriptorExtractorPtr extractor = cv::Ptr<cv::DescriptorExtractor>(new cv::SurfDescriptorExtractor());

    cv::Mat image = cv::imread(filename.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
    if (image.empty()) {
        exit(EXIT_FAILURE);
    }

    KeyPoints keypoints;
    cv::Mat   descriptors;

    detector->detect(image, keypoints);
    extractor->compute(image, keypoints, descriptors);

    cv::FileStorage fs(".yml", cv::FileStorage::MEMORY | cv::FileStorage::WRITE);

    fs << "keypoints" << keypoints;
    fs << "descriptors" << descriptors;

    std::string serialized = fs.releaseAndGetString();

    std::cout << serialized;

    return EXIT_SUCCESS;
}
