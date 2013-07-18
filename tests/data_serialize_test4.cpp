#include <iostream>
#include <sstream>
#include <string>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include <boost/serialization/string.hpp>

#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/gzip.hpp>

class Foo {
public:

    std::string s;

    Foo(std::string _s): s(_s) {}
    Foo() {};

    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive &ar, const unsigned int)
    {
        ar & s;
    }
};

int
main()
{
    namespace bio = boost::iostreams;

    std::stringstream compressed;

    Foo f0("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
    Foo f1;

    {
        bio::filtering_streambuf<bio::output> compressor;

        compressor.push(bio::gzip_compressor(bio::gzip::best_speed));
        compressor.push(compressed);

        boost::archive::binary_oarchive oa(compressor);

        oa << f0;
    }

    std::cout << "Compressed size: " << compressed.str().size() << std::endl;

    {
        bio::filtering_streambuf<bio::input> decompressor;

        decompressor.push(bio::gzip_decompressor());
        decompressor.push(compressed);

        boost::archive::binary_iarchive ia(decompressor);
        
        ia >> f1;
    }

    std::cout << (f0.s == f1.s) << std::endl;

    return EXIT_SUCCESS;
}