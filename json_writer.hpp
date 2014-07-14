#ifndef __JSON_WRITER_HPP_INCLUDED__
#define __JSON_WRITER_HPP_INCLUDED__

#include <yajl/yajl_tree.h>
#include <yajl/yajl_gen.h>

#include "exc.hpp"

namespace imgdupl {

#define YAJL_STR(s)         ((const unsigned char*)s.c_str())
#define YAJL_STR_SIZE(s)    (s.size())

class JsonWriter {
public:

    yajl_gen g;

    JsonWriter(yajl_gen gen): g(gen) {}

    void write(const std::string &s)
    {
        status = yajl_gen_string(g, YAJL_STR(s), YAJL_STR_SIZE(s));
        THROW_EXC_IF_FAILED(status == yajl_gen_status_ok,
            "YAJL generator failed, error code: %u, data: %s",
            status, s.c_str());
    }

    template <typename T>
    void write(T i)
    {
        status = yajl_gen_integer(g, i);
        THROW_EXC_IF_FAILED(status == yajl_gen_status_ok,
            "YAJL generator failed, error code: %u, data: %lli",
            status, static_cast<long long int>(i));
    }

    template <typename T>
    void write(const std::string &k, T v)
    {
        write(k);
        write(v);
    }

protected:
    
    yajl_gen_status status;

private:

    JsonWriter() {};    
};

class JsonMapWriter: public JsonWriter {
public:

    JsonMapWriter(yajl_gen gen):
        JsonWriter(gen)
    {
        status = yajl_gen_map_open(g);
        THROW_EXC_IF_FAILED(status == yajl_gen_status_ok,
            "YAJL generator failed, error code: %u", status);
    }

    ~JsonMapWriter()
    {
        yajl_gen_map_close(g);
    }
};

class JsonArrayWriter: public JsonWriter {
public:

    JsonArrayWriter(yajl_gen gen):
        JsonWriter(gen)
    {
        status = yajl_gen_array_open(g);
        THROW_EXC_IF_FAILED(status == yajl_gen_status_ok,
            "YAJL generator failed, error code: %u", status);
    }

    ~JsonArrayWriter()
    {
        yajl_gen_array_close(g);
    }
};
 
} // namespace

#endif
