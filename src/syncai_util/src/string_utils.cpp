#include "syncai_util/string_utils.hpp"

#include <string>

using std::string;

namespace syncai_util
{

std::string strip_leading_slash(const string & in)
{
  string out = in;

  if ((!in.empty()) && (in[0] == '/')) {
    out.erase(0, 1);
  }

  return out;
}

Tokens split(const string & tokenstring, char delimiter)
{
  Tokens tokens;

  size_t current_pos = 0;
  size_t pos = 0;
  while ((pos = tokenstring.find(delimiter, current_pos)) != string::npos) {
    tokens.push_back(tokenstring.substr(current_pos, pos - current_pos));
    current_pos = pos + 1;
  }
  tokens.push_back(tokenstring.substr(current_pos));
  return tokens;
}

}  // namespace syncai_util