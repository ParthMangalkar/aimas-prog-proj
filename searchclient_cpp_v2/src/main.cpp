// searchclient_cpp_v2 — entry point.
//
// Server protocol:
//   - Read level definition (#domain ... #end) from stdin.
//   - Print client name on first stdout line.
//   - For each joint action emitted, read the server's per-agent acceptance
//     line back from stdin (we don't act on it but must consume).

#include "aimas/core.hpp"
#include "aimas/parser.hpp"
#include "aimas/solver.hpp"

#include <chrono>
#include <iostream>
#include <ostream>
#include <string>

int main(int /*argc*/, char* /*argv*/[])
{
    using namespace aimas;
    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);

    std::cout << "MAmaMASS" << std::endl;
    std::cout << "#modular v2 baseline" << std::endl;

    Level level = parse_level(std::cin);
    std::cerr << "[v2] Parsed level '" << level.name << "' "
              << level.rows << "x" << level.cols
              << " with " << level.agent_rows.size() << " agents." << std::endl;

    const auto t0 = std::chrono::steady_clock::now();
    Solver solver(level);
    auto plan = solver.solve();
    const auto t1 = std::chrono::steady_clock::now();
    const double secs =
        std::chrono::duration<double>(t1 - t0).count();

    if (plan.empty()) {
        std::cerr << "[v2] Unable to solve level (in "
                  << secs << "s)." << std::endl;
        return 0;
    }
    std::cerr << "[v2] Plan length " << plan.size()
              << " joint actions, found in " << secs << "s." << std::endl;

    // Emit joint actions to the server. Format per agent index: action name.
    const auto& tbl = actions();
    for (const auto& joint : plan) {
        for (std::size_t i = 0; i < joint.size(); ++i) {
            if (i) std::cout << '|';
            std::cout << tbl[joint[i]].name;
        }
        std::cout << '\n';
        std::cout.flush();
        std::string reply;
        if (!std::getline(std::cin, reply)) break;
    }
    return 0;
}
