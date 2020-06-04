#pragma once

#include <fstream>
#include <string>

// without some sort of boost reference first, the next ones errror
#include <boost/regex.hpp>
#include <boost/exception/exception.hpp>
#include <boost/current_function.hpp>
#if !defined( BOOST_THROW_EXCEPTION )
#define BOOST_THROW_EXCEPTION(x) ::boost::exception_detail::throw_exception_(x,BOOST_CURRENT_FUNCTION,__FILE__,__LINE__)
#endif
// include headers that implement a archive in simple text format and xml

#include <boost/archive/tmpdir.hpp>

// txt
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

// xml 
#include <boost/archive/xml_iarchive.hpp> 
#include <boost/archive/xml_oarchive.hpp> 

// assorted sitl-specific utils to assist in boost serialze and deserialize
std::string string_to_hex(const std::string& input);
int hex_value(char hex_digit);
std::string hex_to_string(const std::string& input);


