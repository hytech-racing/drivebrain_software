#ifndef __ARG_PARSE_H__
#define __ARG_PARSE_H__

#include <boost/program_options.hpp>

#include <string>
#include <utility>
#include <tuple>


std::tuple<std::string, std::string, bool> parse_arguments(int argc, char* argv[]);


#endif // __ARG_PARSE_H__