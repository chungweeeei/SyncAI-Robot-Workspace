#ifndef SYNCAI_COSTMAP_2D__ARRAY_PARSER_HPP_
#define SYNCAI_COSTMAP_2D__ARRAY_PARSER_HPP_

#include <string>
#include <vector>

namespace syncai_costmap_2d
{
/** @brief Parse a vector of vectors of floats from a string.
 * @param error_return If no error, error_return is set to "".  If
 *        error, error_return will describe the error.
 * Syntax is [[1.0, 2.0], [3.3, 4.4, 5.5], ...]
 *
 * On error, error_return is set and the return value could be
 * anything, like part of a successful parse. */
std::vector<std::vector<float>> parseVVF(const std::string & input, std::string & error_return);
}  // namespace syncai_costmap_2d

#endif  // SYNCAI_COSTMAP_2D__ARRAY_PARSER_HPP_