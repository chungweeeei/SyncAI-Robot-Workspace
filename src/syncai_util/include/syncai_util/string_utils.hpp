#ifndef SYNCAI_UTIL__STRING_UTILS_HPP_
#define SYNCAI_UTIL__STRING_UTILS_HPP_

#include <string>
#include <vector>

namespace syncai_util
{

typedef std::vector<std::string> Tokens;

/*
 * @brief Remove leading slash from a topic name
 * @param in String of topic in
 * @return String out without slash
*/
std::string strip_leading_slash(const std::string & in);

///
/*
 * @brief Split a string at the delimiters
 * @param in String to split
 * @param Delimiter criteria
 * @return Tokens
*/
Tokens split(const std::string & tokenstring, char delimiter);

}  // namespace syncai_util

#endif  // SYNCAI_UTIL__STRING_UTILS_HPP_