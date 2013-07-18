// g++ -Wall -W -O3 data_deflate_test.cpp -o /tmp/t -lboost_iostreams
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <iomanip>
#include <cerrno>

#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/gzip.hpp>

namespace bio = boost::iostreams;

off_t
get_file_size(std::string file)
{
    struct stat st;

    memset(&st, 0, sizeof(st));

    int rc = stat(file.c_str(), &st);
    if (rc == 0) {
        return st.st_size;
    } else {
        std::cerr << "Error: " << strerror(errno) << std::endl;
        exit(EXIT_FAILURE);
    }
}


int
main(int argc, char **argv)
{
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <file>" << std::endl;
        exit(EXIT_SUCCESS);
    }

    std::string data_file = argv[1];

    std::ifstream file(data_file.c_str(), std::ios::in|std::ios::binary);

    size_t  file_size = get_file_size(data_file);
    char   *raw_data = new char[file_size];

    file.read(raw_data, file_size);

    std::string data(raw_data, file_size);

    delete[] raw_data;

    std::stringstream      compressed;
    bio::filtering_ostream compressor;

    compressor.push(bio::gzip_compressor(bio::gzip::best_speed));
    compressor.push(compressed);

    compressor.write(data.c_str(), data.size());
    bio::close(compressor);

    std::stringstream                     decompressed;
    bio::filtering_streambuf<bio::output> output(decompressed);
    bio::filtering_streambuf<bio::input>  decompressor;

    decompressor.push(bio::gzip_decompressor());
    decompressor.push(compressed);

    bio::copy(decompressor, output);
    bio::close(output);

    std::cout << "Initial size     : " << data.size() << std::endl;
    std::cout << "Compressed size  : " << compressed.str().size() << std::endl;
    std::cout << "Decompressed size: " << decompressed.str().size() << std::endl;
    std::cout << "Is data equal    : " << std::boolalpha << (data == decompressed.str()) << std::endl;

    return EXIT_SUCCESS;
}
