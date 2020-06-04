#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "SITL/Serialize.h"
// with thanks to https://stackoverflow.com/questions/3381614/c-convert-string-to-hexadecimal-and-vice-versa
//#include <stdexcept>


namespace boost
{
void throw_exception( std::exception const & e ){
// unimpl
}
}

std::string string_to_hex(const std::string& input)
{
    static const char hex_digits[] = "0123456789ABCDEF";

    std::string output;
    output.reserve(input.length() * 2);
    for (unsigned char c : input)
    {
        output.push_back(hex_digits[c >> 4]);
        output.push_back(hex_digits[c & 15]);
    }
    return output;
}



int hex_value(char hex_digit)
{
    switch (hex_digit) {
    case '0': case '1': case '2': case '3': case '4':
    case '5': case '6': case '7': case '8': case '9':
        return hex_digit - '0';

    case 'A': case 'B': case 'C': case 'D': case 'E': case 'F':
        return hex_digit - 'A' + 10;

    case 'a': case 'b': case 'c': case 'd': case 'e': case 'f':
        return hex_digit - 'a' + 10;
    }
    //throw std::invalid_argument("invalid hex digit");
    return 0;
}

std::string hex_to_string(const std::string& input)
{
    const auto len = input.length();
    //if (len & 1) throw std::invalid_argument("odd length");

    std::string output;
    output.reserve(len / 2);
    for (auto it = input.begin(); it != input.end(); )
    {
        int hi = hex_value(*it++);
        int lo = hex_value(*it++);
        output.push_back(hi << 4 | lo);
    }
    return output;
}
#endif  // CONFIG_HAL_BOARD == HAL_BOARD_SITL

