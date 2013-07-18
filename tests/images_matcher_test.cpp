#include <iostream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

namespace bfs = boost::filesystem;

typedef cv::Ptr<cv::FeatureDetector>     FeatureDetectorPtr;
typedef cv::Ptr<cv::DescriptorExtractor> DescriptorExtractorPtr;
typedef cv::Ptr<cv::DescriptorMatcher>   DescriptorMatcherPtr;

typedef std::vector<cv::Mat>     TrainDescriptors;
typedef std::vector<std::string> Images;
typedef std::vector<cv::DMatch>  Matches;
typedef std::vector<Matches>     MatchesVect;

// key is an image index, value is a number of times feature was selected
typedef std::map<int, size_t>    MatchesCounter;

// Grouped matches
class GMatches {
public:

    int    index;
    size_t matches;

    GMatches():
        index(-1),
        matches(0)
    {
    }

    GMatches(int index_, size_t matches_):
        index(index_),
        matches(matches_)
    {
    }

    bool operator<(const GMatches &other) const {
        return matches < other.matches;
    }

    bool operator>(const GMatches &other) const {
        return matches > other.matches;
    }
};

void
print_best_n_matches(int n, Images &images, Matches &matches)
{
    BOOST_FOREACH(Matches::value_type &v, matches) {
        std::cout << v.queryIdx << '\t' << v.trainIdx << '\t' << v.imgIdx << '\t' << v.distance << std::endl;
    }

    MatchesCounter features;

    BOOST_FOREACH(Matches::value_type &v, matches) {
        int idx = v.imgIdx;
        MatchesCounter::iterator found = features.find(idx);
        if (found == features.end()) {
            features[idx] = 1;
        } else {
            features[idx]++;
        }
    }

    /*
    std::cout << "****" << std::endl;
    BOOST_FOREACH(MatchesCounter::value_type &v, features) {
        std::cout << images[v.first] << " -> " << v.second << std::endl;
    }
    std::cout << "****" << std::endl;
    */

    typedef std::vector<GMatches> GMatchesVect;

    GMatchesVect vect;

    BOOST_FOREACH(MatchesCounter::value_type &v, features) {
        vect.push_back(GMatches(v.first, v.second));
    }

    std::sort(vect.begin(), vect.end(), std::greater<GMatchesVect::value_type>());

    int counter = 0;

    BOOST_FOREACH(GMatchesVect::value_type &v, vect) {
        std::cout << v.index << "," << images[v.index] << " --> " << v.matches << std::endl;
        counter++;
        if (counter == n) {
            break;
        }
    }

    std::sort(matches.begin(), matches.end());

    std::cout << std::endl << "Best match based on minimum distance" << std::endl;
    std::cout << images[matches[0].imgIdx] << "  --> " << matches[0].distance << std::endl;
}

void
print_best_n_matches(int n, Images &images, MatchesVect &matches_vect)
{
    BOOST_FOREACH(MatchesVect::value_type &v, matches_vect) {
        print_best_n_matches(n, images, v);
        std::cout << "***" << std::endl;
    }
}

void
process_file(
    const std::string &filename,
    FeatureDetectorPtr &detector,
    DescriptorExtractorPtr &extractor,
    cv::Mat &descriptors
)
{
    std::vector<cv::KeyPoint> keypoints;

    cv::Mat image = cv::imread(filename.c_str(), CV_LOAD_IMAGE_GRAYSCALE);

    detector->detect(image, keypoints);
    extractor->compute(image, keypoints, descriptors);
}

void
process_directory(
    const std::string &directory,
    FeatureDetectorPtr &detector,
    DescriptorExtractorPtr &extractor,
    Images &files,
    TrainDescriptors &train_descriptors
)
{
    bfs::path root(directory);

    if (bfs::exists(root) && bfs::is_directory(root)) {
        std::string filename;
        bfs::recursive_directory_iterator cur_it(root), end_it;

        files.clear();
        train_descriptors.clear();

        size_t processed_files = 0;

        for (; cur_it != end_it; ++cur_it) {
            if (bfs::is_regular_file(cur_it->path())) {
                filename = cur_it->path().string();

                cv::Mat descriptors;
                process_file(filename, detector, extractor, descriptors);
                train_descriptors.push_back(descriptors);
                files.push_back(filename);

                processed_files++;
                if (processed_files % 1000 == 0) {
                    std::cout << "processed " << processed_files << " files" << std::endl;
                }
            }
        }
    }
}

bool
create_engines(
    FeatureDetectorPtr &feature_detector,
    DescriptorExtractorPtr &descriptor_extractor,
    DescriptorMatcherPtr &descriptor_matcher
)
{
    bool is_created = false;

    std::string detector_type = "ORB";
    std::string descriptor_type = "ORB";

    //feature_detector = cv::FeatureDetector::create(detector_type);
    feature_detector = cv::Ptr<cv::FeatureDetector>(new cv::ORB());
    if (feature_detector.empty()) {
        std::cerr << "Can't create feature detector of type \"" << detector_type << "\"" << std::endl;
        goto end;
    }

    descriptor_extractor = cv::DescriptorExtractor::create(descriptor_type);
    if (descriptor_extractor.empty()) {
        std::cerr << "Can't create descriptor extractor of type \"" << descriptor_type << "\"" << std::endl;
        goto end;   
    }

    //descriptor_matcher = DescriptorMatcher::create(matcher_type);
    descriptor_matcher = cv::Ptr<cv::DescriptorMatcher>(
        new cv::FlannBasedMatcher(new cv::flann::LshIndexParams(10, 10, 2))
    );
    if (descriptor_matcher.empty()) {
        std::cerr << "Can't create descriptor matcher" << std::endl;
        goto end;   
   }

    is_created = true;

end:
    return is_created;
}


int
main(int argc, char **argv)
{
    cv::initModule_nonfree();

    if (argc < 3) {
        std::cout << argv[0] << " <image> <directory>" << std::endl;
        return 0;
    }

    std::string query_file = argv[1];
    std::string directory = argv[2];

    //FeatureDetectorPtr     detector;
    //DescriptorExtractorPtr extractor;
    //DescriptorMatcherPtr   matcher;

    FeatureDetectorPtr     detector = cv::Ptr<cv::FeatureDetector>(new cv::OrbFeatureDetector(500));
    DescriptorExtractorPtr extractor = cv::Ptr<cv::DescriptorExtractor>(new cv::OrbDescriptorExtractor());
    DescriptorMatcherPtr   matcher = cv::Ptr<cv::DescriptorMatcher>(
        new cv::FlannBasedMatcher(new cv::flann::LshIndexParams(10, 10, 2)));

    //create_engines(detector, extractor, matcher);

    cv::Mat          query_descriptors;
    TrainDescriptors train_descriptors;
    Images           images;

    process_file(query_file, detector, extractor, query_descriptors);
    process_directory(directory, detector, extractor, images, train_descriptors);

    std::cout << "Index build" << std::endl;

    matcher->add(train_descriptors);
    matcher->train();

#if 1
    Matches matches;
    matcher->match(query_descriptors, matches);
    CV_Assert(query_descriptors.rows == (int)matches.size() || matches.empty());

    std::cout << matches.size() << std::endl;
    std::cout << query_descriptors.rows << "," << query_descriptors.cols << std::endl;

    print_best_n_matches(3, images, matches);
#endif

#if 0
    MatchesVect matches_vect;
    matcher->knnMatch(query_descriptors, matches_vect, 3);
    print_best_n_matches(3, images, matches_vect);
#endif

#if 0
    cv::Mat image1 = cv::imread(file1.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
    cv::Mat image2 = cv::imread(file2.c_str(), CV_LOAD_IMAGE_GRAYSCALE);

    if(image1.empty()) {
        std::cerr << "Couldn't load \"" << file1 << "\" image" << std::endl;
        return EXIT_FAILURE;
    }

    if(image2.empty()) {
        std::cerr << "Couldn't load \"" << file2 << "\" image" << std::endl;
        return EXIT_FAILURE;
    }

    cv::ORB orb;

    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    cv::Mat desc1, desc2;

    orb(image1, cv::Mat(), keypoints1, desc1);
    orb(image2, cv::Mat(), keypoints2, desc2);

    std::cout << "#keypoints1: " << keypoints1.size() << std::endl;
    std::cout << "#keypoints2: " << keypoints2.size() << std::endl;

    std::vector<cv::DMatch> matches;
    cv::FlannBasedMatcher flann_matcher(new cv::flann::LshIndexParams(20,10,2));

    double max_dist = 0;
    double min_dist = 100;

    flann_matcher.match(desc1, desc2, matches);

    //-- Quick calculation of max and min distances between keypoints
    for (int i = 0; i < desc1.rows; i++) {
        double dist = matches[i].distance;
        if (dist < min_dist) {
            min_dist = dist;
        }
        if (dist > max_dist) {
            max_dist = dist;
        }
    }

    size_t good_matches = 0;
    for (int i = 0; i < desc1.rows; i++) {
        if (matches[i].distance < 2 * min_dist) {
            good_matches++;
        }
    }

    std::cout << "FlannBasedMatcher:" << matches.size() << std::endl;
    std::cout << "-- Max dist: " << max_dist << std::endl;
    std::cout << "-- Min dist: " << min_dist << std::endl;
    std::cout << "-- Good pts: " << good_matches << std::endl;

    matches.clear();

    cv::BFMatcher bf_matcher(cv::NORM_HAMMING);

    bf_matcher.match(desc1, desc2, matches);

    std::cout << "#matches: " << matches.size() << std::endl;

    max_dist = 0;
    min_dist = 100;

    //-- Quick calculation of max and min distances between keypoints
    for (int i = 0; i < desc1.rows; i++) {
        double dist = matches[i].distance;
        if (dist < min_dist) {
            min_dist = dist;
        }
        if (dist > max_dist) {
            max_dist = dist;
        }
    }

    std::cout << "BFMatcher:" << matches.size() << std::endl;
    std::cout << "-- Max dist: " << max_dist << std::endl;
    std::cout << "-- Min dist: " << min_dist << std::endl;

    for (size_t i = 0; i < matches.size(); i++) {
        std::cout << "#" << i << ": " << "queryIdx: " << matches[i].queryIdx << 
            ", trainIdx: " << matches[i].trainIdx << ", imgIdx: " <<
            matches[i].imgIdx << ", distance: " << matches[i].distance << std::endl;
    }

    /*
    std::cout << "TrainDescriptors:" << std::endl;
    for (size_t i = 0; i < descriptors.size(); ++i) {
        std::cout << descriptors[i] << ", ";
    }
    std::cout << std::endl;
    */
#endif

    return 0;
}
