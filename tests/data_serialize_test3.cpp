#include <iostream>
#include <sstream>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/gzip.hpp>

#include "serialize.hpp"
#include "data_types.hpp"

using namespace imgdupl;

namespace bio = boost::iostreams;

void
usage(const char *program)
{
    std::cout << "Usage: " << program << " <image>" << std::endl;
    exit(EXIT_SUCCESS);
}

bool
operator==(const cv::KeyPoint &p1, const cv::KeyPoint &p2)
{
    return (
        p1.pt.x == p2.pt.x &&
        p1.pt.y == p2.pt.y &&
        p1.size == p2.size &&
        p1.angle == p2.angle &&
        p1.response == p2.response &&
        p1.octave == p2.octave &&
        p1.class_id == p2.class_id
    );
}

bool
is_equal(const KeyPoints &kp1, const KeyPoints &kp2)
{
    bool eq = (kp1.size() == kp2.size());

    if (eq) {
        for (size_t i = 0; i < kp1.size(); i++) {
            eq = (kp1[i] == kp2[i]);
            if (!eq) {
                return false;
            }
        }
        return true;
    } else {
        return false;
    }
}

bool
is_equal(const cv::Mat &m1, const cv::Mat &m2)
{
    bool eq = (m1.elemSize() == m2.elemSize() &&
               m1.type() &&
               m2.type() &&
               m1.cols == m2.cols &&
               m1.rows == m2.rows);

    if (eq) {
        const size_t data_size = m1.cols * m1.rows * m1.elemSize();
        eq = (memcmp(m1.ptr(), m2.ptr(), data_size) == 0);
        if (eq) {
            return true;
        } else {
            return false;
        }
    } else {
        return false;
    }
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

    std::stringstream compressed;

    {
        bio::filtering_streambuf<bio::output> compressor;

        compressor.push(bio::gzip_compressor(bio::gzip::best_speed));
        compressor.push(compressed);

        boost::archive::binary_oarchive oa(compressor);

        oa << keypoints;
        oa << descriptors;
    }

    std::cout << "Compressed size: " << compressed.str().size() << std::endl;

    KeyPoints keypoints2;
    cv::Mat   descriptors2;

    {
        bio::filtering_streambuf<bio::input> decompressor;

        decompressor.push(bio::gzip_decompressor());
        decompressor.push(compressed);

        boost::archive::binary_iarchive ia(decompressor);
        
        ia >> keypoints2;
        ia >> descriptors2;
    }

    std::cout << "Descriptors equal: " << is_equal(descriptors, descriptors2) << std::endl;
    std::cout << "Keypoints equal  : " << is_equal(keypoints, keypoints2) << std::endl;

    return EXIT_SUCCESS;
}
