#ifndef __SEARCHER_HPP_INCLUDED__
#define __SEARCHER_HPP_INCLUDED__

#include <utility>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/filter/gzip.hpp>

#include <boost/foreach.hpp>

#include <boost/thread.hpp>

#include <sqlite3.h>

#include "exc.hpp"
#include "config.hpp"
#include "serialize.hpp"
#include "data_types.hpp"

namespace imgdupl {

class Searcher {
public:

    Config cfg;

    Searcher(const Config &_cfg): cfg(_cfg)
    {
        create_engines();
        load();
    }

    std::pair<bool, std::string> search(const cv::Mat &image);

private:

    FeatureDetectorPtr     detector;
    DescriptorExtractorPtr extractor;
    DescriptorMatcherPtr   matcher;

    TrainKeyPoints   train_keypoints;
    TrainDescriptors train_descriptors;

    Images images;

    mutable boost::mutex mutex;

    Searcher() {}

    void load();
    void create_engines();
    void clear();
    bool is_nice_homography(const cv::Mat &h);
    int find_best_match(const Matches &matches);
};

class GrouppedMatch {
public:

    int    index;   // image index
    size_t matches; // number of time feature from that image was selected

    GrouppedMatch():
        index(-1),
        matches(0)
    {
    }

    GrouppedMatch(int index_, size_t matches_):
        index(index_),
        matches(matches_)
    {
    }

    bool operator<(const GrouppedMatch &other) const {
        return matches < other.matches;
    }

    bool operator>(const GrouppedMatch &other) const {
        return matches > other.matches;
    }
};

}

#endif
