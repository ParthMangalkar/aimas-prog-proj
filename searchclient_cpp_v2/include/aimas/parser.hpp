#pragma once

#include "aimas/core.hpp"

#include <iosfwd>

namespace aimas {

// Read a hospital-domain level from the server-protocol stream up to (and
// including) the #end line. Throws std::runtime_error on malformed input.
Level parse_level(std::istream& input);

}  // namespace aimas
