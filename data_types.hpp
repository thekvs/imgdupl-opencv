#ifndef __DATA_TYPES_HPP_INCLUDED__
#define __DATA_TYPES_HPP_INCLUDED__

#include <vector>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>

namespace imgdupl {

typedef cv::Ptr<cv::FeatureDetector>     FeatureDetectorPtr;
typedef cv::Ptr<cv::DescriptorExtractor> DescriptorExtractorPtr;
typedef cv::Ptr<cv::DescriptorMatcher>   DescriptorMatcherPtr;

typedef std::vector<cv::KeyPoint> KeyPoints;
typedef cv::Mat                   Descriptors;
typedef std::vector<KeyPoints>    TrainKeyPoints;
typedef std::vector<Descriptors>  TrainDescriptors;
typedef std::vector<std::string>  Images;
typedef std::vector<cv::DMatch>   Matches;

} // namespace

#endif
