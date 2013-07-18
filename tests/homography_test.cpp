// g++ -Wall -W -O3 `pkg-config --cflags opencv` homography_test.cpp -o /tmp/homography_test -Wl,-rpath=/opt/opencv/lib `pkg-config --libs opencv`
#include <ctime>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include "data_types.hpp"

using namespace imgdupl;

void
process_file(
    const std::string &filename,
    FeatureDetectorPtr &detector,
    DescriptorExtractorPtr &extractor,
    cv::Mat &image,
    KeyPoints &keypoints,
    cv::Mat &descriptors
)
{
    image = cv::imread(filename.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
    if (image.empty()) {
        exit(EXIT_FAILURE);
    }

    keypoints.clear();

    detector->detect(image, keypoints);
    extractor->compute(image, keypoints, descriptors);
}

bool
is_nice_homography(const cv::Mat &h, std::vector<double> &coeffs)
{
    const double det = cv::determinant(h);
    const double n1 = sqrt(h.at<double>(0, 0) * h.at<double>(0, 0) + h.at<double>(1, 0) * h.at<double>(1, 0));
    const double n2 = sqrt(h.at<double>(0, 1) * h.at<double>(0, 1) + h.at<double>(1, 1) * h.at<double>(1, 1));
    const double n3 = sqrt(h.at<double>(2, 0) * h.at<double>(2, 0) + h.at<double>(2, 1) * h.at<double>(2, 1));

    coeffs.push_back(det);
    coeffs.push_back(n1);
    coeffs.push_back(n2);
    coeffs.push_back(n3);

    std::cout << h << std::endl;

    cv::Mat r, q;
    RQDecomp3x3(h, r, q);

    std::cout << r << std::endl << q << std::endl;

    if (det < 0 || fabs(det) < 2e-05) {
        return false;
    }

    if (n1 > 4 || n1 < 0.1) {
        return false;
    }

    if (n2 > 4 || n2 < 0.1) {
        return false;
    }
    
    if (n3 > 0.002) {
        return false;
    }

    return true;
}

#if 1
bool
findHomography2(
    const cv::Mat &image,
    const KeyPoints& source,
    const KeyPoints& result,
    const Matches& input,
    Matches& inliers,
    cv::Mat& homography
)
{
    if (input.size() < 4)
        return false;

    const int pointsCount = input.size();
    const float reprojectionThreshold = 3;

    //Prepare src and dst points
    std::vector<cv::Point2f> srcPoints, dstPoints;
    for (int i = 0; i < pointsCount; i++)
    {
        srcPoints.push_back(source[input[i].trainIdx].pt);
        dstPoints.push_back(result[input[i].queryIdx].pt);
    }

    // Find homography using RANSAC algorithm
    std::vector<unsigned char> status;
    homography = cv::findHomography(srcPoints, dstPoints, CV_RANSAC, reprojectionThreshold, status);

    // Warp dstPoints to srcPoints domain using inverted homography transformation
    std::vector<cv::Point2f> srcReprojected;
    cv::perspectiveTransform(dstPoints, srcReprojected, homography.inv());

    cv::Size out_image_size(image.cols, image.rows);
    cv::Mat  out_image;

    out_image.create(out_image_size, CV_MAKETYPE(image.depth(), 3));
    out_image = cv::Scalar::all(0);

    if(image.type() == CV_8U) {
        cvtColor(image, out_image, CV_GRAY2BGR);
    } else {
        image.copyTo(out_image);
    }

    for (size_t i = 0; i < srcPoints.size(); i++) {
        cv::circle(out_image, srcPoints[i], 3, cv::Scalar(128,255,255));
    }

    for (size_t i = 0; i < srcReprojected.size(); i++) {
        cv::circle(out_image, srcReprojected[i], 4, cv::Scalar(255,128,255));
    }

    // Pass only matches with low reprojection error (less than reprojectionThreshold value in pixels)
    inliers.clear();
    for (int i = 0; i < pointsCount; i++)
    {
        cv::Point2f actual = srcPoints[i];
        cv::Point2f expect = srcReprojected[i];
        cv::Point2f v = actual - expect;
        float distanceSquared = v.dot(v);

        if (/*status[i] && */distanceSquared <= reprojectionThreshold * reprojectionThreshold)
        {
            inliers.push_back(input[i]);
        }
    }

    cv::imshow("Matches Reprojected", out_image);
    cv::waitKey();

    // Test for bad case
    if (inliers.size() < 4)
        return false;

    // Now use only good points to find refined homography:
    std::vector<cv::Point2f> refinedSrc, refinedDst;
    for (size_t i = 0; i < inliers.size(); i++) {
        refinedSrc.push_back(source[inliers[i].trainIdx].pt);
        refinedDst.push_back(result[inliers[i].queryIdx].pt);
    }

    // Use least squares method to find precise homography
    cv::Mat homography2 = cv::findHomography(refinedSrc, refinedDst, 0, reprojectionThreshold);

    // Reproject again:
    cv::perspectiveTransform(dstPoints, srcReprojected, homography2.inv());
    inliers.clear();

    for (size_t i = 0; i < srcReprojected.size(); i++) {
        cv::circle(out_image, srcReprojected[i], 5, cv::Scalar(255,128,155));
    }

    cv::imshow("Matches Reprojected", out_image);
    cv::waitKey();

    for (int i = 0; i < pointsCount; i++)
    {
        cv::Point2f actual = srcPoints[i];
        cv::Point2f expect = srcReprojected[i];
        cv::Point2f v = actual - expect;
        float distanceSquared = v.dot(v);

        if (distanceSquared <= reprojectionThreshold * reprojectionThreshold)
        {
            inliers.push_back(input[i]);
        }
    }

    homography = homography2;

    return inliers.size() >= 4;
}
#endif

int
main(int argc, char **argv)
{
    cv::initModule_nonfree();

    if (argc < 3) {
        std::cout << "Not enough args. given" << std::endl;
        exit(EXIT_SUCCESS);
    }

    std::string file1 = argv[1];
    std::string file2 = argv[2];

    clock_t start = clock();

#if 0
    FeatureDetectorPtr     detector = cv::Ptr<cv::FeatureDetector>(new cv::OrbFeatureDetector(100));
    DescriptorExtractorPtr extractor = cv::Ptr<cv::DescriptorExtractor>(new cv::OrbDescriptorExtractor());
    DescriptorMatcherPtr   matcher = cv::Ptr<cv::DescriptorMatcher>(new cv::BFMatcher(cv::NORM_HAMMING));
#endif

#if 1
    FeatureDetectorPtr     detector = cv::Ptr<cv::FeatureDetector>(new cv::SurfFeatureDetector(400));
    DescriptorExtractorPtr extractor = cv::Ptr<cv::DescriptorExtractor>(new cv::SurfDescriptorExtractor());
    DescriptorMatcherPtr   matcher = cv::Ptr<cv::DescriptorMatcher>(new cv::BFMatcher(cv::NORM_L2));
#endif

    KeyPoints keypoints1, keypoints2;
    cv::Mat   descriptors1, descriptors2;
    cv::Mat   image1, image2;

    process_file(file1, detector, extractor, image1, keypoints1, descriptors1);
    process_file(file2, detector, extractor, image2, keypoints2, descriptors2);

    Matches matches;
    matcher->match(descriptors1, descriptors2, matches);

    cv::Mat homography;
    cv::Mat image_matches;

    //-- Quick calculation of max and min distances between keypoints
    double max_dist = 0;
    double min_dist = 100;

    for (int i = 0; i < descriptors1.rows; i++ ) {
        double dist = matches[i].distance;
        if (dist < min_dist) {
            min_dist = dist;
        }

        if (dist > max_dist) {
            max_dist = dist;
        }
    }

    //-- "good" matches (i.e. whose distance is less than 3*min_dist )
    std::vector<cv::DMatch> good_matches;

    //int rows = std::min(descriptors1.rows, descriptors2.rows);
    int rows = descriptors1.rows;
    for (int i = 0; i < rows; i++ ) {
        if (matches[i].distance <= 3 * min_dist) {
            good_matches.push_back(matches[i]);
        }
    }

#if 1
    Matches inliers;

    bool found = findHomography2(image1, keypoints1, keypoints2, matches, inliers, homography);

    std::cout << "findHomography2: " << found << std::endl;

    drawMatches(image1, keypoints1, image2, keypoints2, inliers, image_matches/*, cv::Scalar(255,255,255)*/);
    cv::namedWindow("Matches");
    cv::imshow("Matches", image_matches);
    cv::waitKey();
#endif

#if 1
    //-- Localize the object
    std::vector<cv::Point2f> img1;
    std::vector<cv::Point2f> img2;

    for (size_t i = 0; i < good_matches.size(); i++) {
        //-- Get the keypoints from the good matches
        img1.push_back(keypoints1[good_matches[i].queryIdx].pt);
        img2.push_back(keypoints2[good_matches[i].trainIdx].pt);
    }

    bool    is_coincide;
    clock_t finish;

    std::vector<double> coeffs;

    std::vector<double>::iterator cur_coeff_it;
    std::vector<double>::iterator end_coeff_it;

    try {
        homography = cv::findHomography(img1, img2, CV_RANSAC);
    } catch (cv::Exception &exc) {
        std::cerr << "Error: " << exc.what() << std::endl;
        goto stat;
    }

    is_coincide = is_nice_homography(homography, coeffs);
    finish = clock();

    drawMatches(image1, keypoints1, image2, keypoints2, good_matches, image_matches/*, cv::Scalar(255,255,255)*/);
    cv::namedWindow("Matches");
    cv::imshow("Matches", image_matches);
    cv::waitKey();

    std::cout << std::endl << "Images coincide: " << std::boolalpha << is_coincide << std::endl;
    
    std::cout << "Homography coeffs:";
    cur_coeff_it = coeffs.begin();
    end_coeff_it = coeffs.end();
    for (; cur_coeff_it != end_coeff_it; ++cur_coeff_it) {
        std::cout << " " << *cur_coeff_it;
    }
    std::cout << std::endl;

    std::cout << "  Time used: " << (finish - start) / ((float)CLOCKS_PER_SEC) << " sec." << std::endl;

stat:

    std::cout << std::endl << "Statistics:" << std::endl << std::endl;
    std::cout << "  Number of keypoints1: " << keypoints1.size() << std::endl;
    std::cout << "  Size of descriptors1: " << descriptors1.rows << " x " << descriptors1.cols << std::endl;
    std::cout << "  Bytes used in keypoints1: " << keypoints1.size() * sizeof(KeyPoints::value_type) << std::endl;
    std::cout << "  Bytes used in descriptors1: " << descriptors1.elemSize() * descriptors1.total() << std::endl;
    std::cout << "  Number of keypoints2: " << keypoints2.size() << std::endl;
    std::cout << "  Size of descriptors2: " << descriptors2.rows << " x " << descriptors2.cols << std::endl;
    std::cout << "  Bytes used in keypoints2: " << keypoints2.size() * sizeof(KeyPoints::value_type) << std::endl;
    std::cout << "  Bytes used in descriptors2: " << descriptors2.elemSize() * descriptors2.total() << std::endl;
    std::cout << "  Type of element used in descriptors: " << descriptors1.type() << std::endl;
    std::cout << "  Number of matches: " << matches.size() << std::endl;
    std::cout << "  Number of good_matches: " << good_matches.size() << std::endl;
    std::cout << "  Max. distance: " << max_dist << std::endl;
    std::cout << "  Min. distance: " << min_dist << std::endl;

#endif

    return EXIT_SUCCESS;
}
