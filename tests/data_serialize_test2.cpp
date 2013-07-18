#include <iostream>
#include <fstream>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>

#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/gzip.hpp>

#include "data_types.hpp"

using namespace imgdupl;

BOOST_SERIALIZATION_SPLIT_FREE(cv::Mat)
BOOST_SERIALIZATION_SPLIT_FREE(cv::KeyPoint)
namespace boost {
    namespace serialization {

        /** Serialization support for cv::Mat */
        template<class Archive>
        void save(Archive &ar, const cv::Mat &m, const unsigned int __attribute__((unused)) version)
        {
            size_t elem_size = m.elemSize();
            size_t elem_type = m.type();

            ar & m.cols;
            ar & m.rows;
            ar & elem_size;
            ar & elem_type;

            const size_t data_size = m.cols * m.rows * elem_size;
            ar & boost::serialization::make_array(m.ptr(), data_size);
        }

        /** Serialization support for cv::KeyPoint */
        template<class Archive>
        void save(Archive &ar, const cv::KeyPoint &p, const unsigned int __attribute__((unused)) version)
        {
            ar & p.pt.x;
            ar & p.pt.y;
            ar & p.size;
            ar & p.angle;
            ar & p.response;
            ar & p.octave;
            ar & p.class_id;
        }

        /** Serialization support for cv::Mat */
        template<class Archive>
        void load(Archive &ar, cv::Mat &m, const unsigned int __attribute__((unused)) version)
        {
            int    cols, rows;
            size_t elem_size, elem_type;

            ar & cols;
            ar & rows;
            ar & elem_size;
            ar & elem_type;

            m.create(rows, cols, elem_type);

            size_t data_size = m.cols * m.rows * elem_size;
            ar & boost::serialization::make_array(m.ptr(), data_size);
        }

        /** Serialization support for cv::KeyPoint */
        template<class Archive>
        void load(Archive &ar, const cv::KeyPoint &p, const unsigned int __attribute__((unused)) version)
        {
            ar & p.pt.x;
            ar & p.pt.y;
            ar & p.size;
            ar & p.angle;
            ar & p.response;
            ar & p.octave;
            ar & p.class_id;
        }
    }
}

void
usage(const char *program)
{
    std::cout << "Usage: " << program << " <image> <archive>" << std::endl;
    exit(EXIT_SUCCESS);
}

int
main(int argc, char **argv)
{
    cv::initModule_nonfree();

    if (argc < 3) {
        usage(argv[0]);
    }

    std::string filename = argv[1];
    std::string archive = argv[2];

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

    {
        namespace bio = boost::iostreams;

        std::ios_base::openmode mode = std::ios::binary | std::ios::trunc;
        std::ofstream           ofs(archive.c_str(), mode);

        bio::filtering_streambuf<bio::output> stream;

        stream.push(bio::gzip_compressor(bio::gzip::best_speed));
        stream.push(ofs);

        boost::archive::binary_oarchive oa(stream);

        oa << keypoints;
        oa << descriptors;
    }

    return EXIT_SUCCESS;
}
