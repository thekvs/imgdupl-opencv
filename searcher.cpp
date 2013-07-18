#include "searcher.hpp"

namespace imgdupl {

namespace bio = boost::iostreams;

std::pair<bool, std::string>
Searcher::search(const cv::Mat &image)
{
    KeyPoints   query_keypoints;
    Descriptors query_descriptors;

    detector->detect(image, query_keypoints);
    extractor->compute(image, query_keypoints, query_descriptors);

    Matches matches;

    {
        boost::lock_guard<boost::mutex> lock(mutex); // Do we really need this lock?

        matcher->match(query_descriptors, matches);
    }

    int best_match = find_best_match(matches);

    double min_distance = 100;
    BOOST_FOREACH(Matches::value_type &v, matches) {
        if (v.imgIdx == best_match && v.distance < min_distance) {
            min_distance = v.distance;
        }
    }

    Matches good_matches;
    BOOST_FOREACH(Matches::value_type &v, matches) {
        if (v.imgIdx == best_match && v.distance < 3 * min_distance) {
            good_matches.push_back(v);
        }
    }

    std::vector<cv::Point2f> img1(good_matches.size());
    std::vector<cv::Point2f> img2(good_matches.size());

    BOOST_FOREACH(Matches::value_type &v, good_matches) {
        img1.push_back(query_keypoints[v.queryIdx].pt);
        img2.push_back(train_keypoints[best_match][v.trainIdx].pt);
    }

    cv::Mat homography;

    try {
        homography = cv::findHomography(img1, img2, CV_RANSAC);
    } catch (cv::Exception &exc) {
        return std::make_pair(false, std::string(""));        
    }

    bool        homography_status = is_nice_homography(homography);
    std::string path;

    if (homography_status) {
        path = images[best_match];
    }

    return std::make_pair(homography_status, path);
}

int
Searcher::find_best_match(const Matches &matches)
{
    int result;

    // key is an image index, value is a number of times feature was selected
    typedef std::map<int, size_t> MatchesCounter;

    MatchesCounter features;

    BOOST_FOREACH(const Matches::value_type &v, matches) {
        int idx = v.imgIdx;
        MatchesCounter::iterator found = features.find(idx);
        if (found == features.end()) {
            features[idx] = 1;
        } else {
            features[idx]++;
        }
    }

    typedef std::vector<GrouppedMatch> GrouppedMatches;

    GrouppedMatches groupped_matches;
    
    groupped_matches.reserve(features.size());

    BOOST_FOREACH(MatchesCounter::value_type &v, features) {
        groupped_matches.push_back(GrouppedMatch(v.first, v.second));
    }

    std::sort(groupped_matches.begin(), groupped_matches.end(), std::greater<GrouppedMatches::value_type>());
    result = groupped_matches[0].index;

    return result;
}

void
Searcher::clear()
{
    train_descriptors.clear();
    train_descriptors.clear();
    images.clear();
}

void
Searcher::create_engines()
{
    detector = cv::FeatureDetector::create(cfg.detector());
    extractor = cv::DescriptorExtractor::create(cfg.extractor());
    matcher = cv::DescriptorMatcher::create(cfg.matcher());
}

void
Searcher::load()
{
    sqlite3 *db_handler;
    int      rc;

    rc = sqlite3_open_v2(cfg.db_file().c_str(), &db_handler, SQLITE_OPEN_READONLY, NULL);
    THROW_EXC_IF_FAILED(rc == SQLITE_OK, "sqlite3_open_v2() failed");

    std::string   st = "SELECT path, data FROM images";
    sqlite3_stmt *stmt = NULL;

    rc = sqlite3_prepare_v2(db_handler, st.c_str(), st.size(), &stmt, NULL);
    THROW_EXC_IF_FAILED(rc == SQLITE_OK, "sqlite3_prepare_v2() failed: \"%s\"", sqlite3_errmsg(db_handler));

    clear();

    while (true) {
        rc = sqlite3_step(stmt);
        THROW_EXC_IF_FAILED(rc == SQLITE_ROW || rc == SQLITE_DONE, "sqlite3_step() failed: \"%s\"",
            sqlite3_errmsg(db_handler));
        if (rc == SQLITE_DONE) {
            break;
        }

        std::string path = std::string(
            reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0)),
            sqlite3_column_bytes(stmt, 0)
        );

        std::stringstream data(std::string(
            reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1)),
            sqlite3_column_bytes(stmt, 1)
        ));

        KeyPoints   keypoints;
        Descriptors descriptors;

        {
            bio::filtering_streambuf<bio::input> stream;

            stream.push(bio::gzip_decompressor());
            stream.push(data);

            boost::archive::binary_iarchive ia(stream);

            ia >> keypoints;
            ia >> descriptors;
        }

        train_keypoints.push_back(keypoints);
        train_descriptors.push_back(descriptors);
        images.push_back(path);
    }

    matcher->add(train_descriptors);
    matcher->train();

    rc = sqlite3_finalize(stmt);
    THROW_EXC_IF_FAILED(rc == SQLITE_OK, "sqlite3_finalize() failed: \"%s\"", sqlite3_errmsg(db_handler));

    sqlite3_close(db_handler);
    sqlite3_shutdown();
}

bool
Searcher::is_nice_homography(const cv::Mat &h)
{
    const double det = cv::determinant(h);

    if (det < 0 || fabs(det) < 2e-05) {
        return false;
    }

    const double n1 = sqrt(h.at<double>(0, 0) * h.at<double>(0, 0) + h.at<double>(1, 0) * h.at<double>(1, 0));

    if (n1 > 4 || n1 < 0.1) {
        return false;
    }

    const double n2 = sqrt(h.at<double>(0, 1) * h.at<double>(0, 1) + h.at<double>(1, 1) * h.at<double>(1, 1));

    if (n2 > 4 || n2 < 0.1) {
        return false;
    }

    const double n3 = sqrt(h.at<double>(2, 0) * h.at<double>(2, 0) + h.at<double>(2, 1) * h.at<double>(2, 1));
    
    if (n3 > 0.002) {
        return false;
    }

    return true;
}

} // namespace
