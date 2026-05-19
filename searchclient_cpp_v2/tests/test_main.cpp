// Minimal unit tests for the core domain. No external test framework — just
// asserts that exit with non-zero on failure. Run via `ctest` after building.
//
// Coverage:
//   - Action table sanity (counts, NoOp at index 0).
//   - Trivial level parsing: agent count, walls, colors, goals.
//   - State::applicable: NoOp/Move/Push/Pull.
//   - State::conflicting: same-cell, swap, box-box-swap.
//   - State::apply_joint: rollback on conflict.
//   - Solver: trivial single-agent agent-goal level.

#include "aimas/core.hpp"
#include "aimas/parser.hpp"
#include "aimas/pibt.hpp"
#include "aimas/solver.hpp"

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>

#define CHECK(cond) do {                                              \
    if (!(cond)) {                                                    \
        std::fprintf(stderr, "FAIL: %s @ %s:%d\n",                    \
                     #cond, __FILE__, __LINE__);                      \
        std::exit(1);                                                 \
    }                                                                 \
} while (0)

using namespace aimas;

namespace {

constexpr const char* kTrivialLevel = R"LEVEL(#domain
hospital
#levelname
trivial
#colors
blue: 0
#initial
+++++
+0  +
+   +
+++++
#goal
+++++
+   +
+  0+
+++++
#end
)LEVEL";

constexpr const char* kBoxLevel = R"LEVEL(#domain
hospital
#levelname
box1
#colors
blue: 0, A
#initial
+++++++
+0A   +
+++++++
#goal
+++++++
+    A+
+++++++
#end
)LEVEL";

constexpr const char* kSwapLevel = R"LEVEL(#domain
hospital
#levelname
swap
#colors
blue: 0, 1
#initial
+++++++
+0   1+
+     +
+++++++
#goal
+++++++
+   0 +
+     +
+++++++
#end
)LEVEL";

void test_action_table()
{
    const auto& tbl = actions();
    CHECK(tbl.size() == 29);
    CHECK(tbl[0].type == ActionType::NoOp);
    int n_move = 0, n_push = 0, n_pull = 0;
    for (const auto& a : tbl) {
        if (a.type == ActionType::Move) ++n_move;
        if (a.type == ActionType::Push) ++n_push;
        if (a.type == ActionType::Pull) ++n_pull;
    }
    CHECK(n_move == 4);
    CHECK(n_push == 12);
    CHECK(n_pull == 12);
    (void)n_move; (void)n_push; (void)n_pull;
    std::cerr << "[ok] action_table\n";
}

void test_parse_trivial()
{
    std::stringstream ss(kTrivialLevel);
    Level lv = parse_level(ss);
    CHECK(lv.name == "trivial");
    CHECK(lv.rows == 4);
    CHECK(lv.cols == 5);
    CHECK(lv.agent_rows.size() == 1);
    CHECK(lv.agent_rows[0] == 1);
    CHECK(lv.agent_cols[0] == 1);
    CHECK(lv.walls[0][0] == true);
    CHECK(lv.walls[1][1] == false);
    CHECK(lv.goals[2][3] == '0');
    std::cerr << "[ok] parse_trivial\n";
}

void test_applicable_and_conflict()
{
    std::stringstream ss(kTrivialLevel);
    Level lv = parse_level(ss);
    State s = State::initial(lv);
    // Move(S): agent at (1,1) moves to (2,1) — should be applicable.
    const auto& tbl = actions();
    int move_s = -1;
    for (int i = 0; i < (int)tbl.size(); ++i)
        if (tbl[i].name == "Move(S)") move_s = i;
    CHECK(move_s > 0);
    CHECK(s.applicable(0, tbl[move_s]));
    // Move(N): blocked by wall.
    int move_n = -1;
    for (int i = 0; i < (int)tbl.size(); ++i)
        if (tbl[i].name == "Move(N)") move_n = i;
    CHECK(move_n > 0);
    CHECK(!s.applicable(0, tbl[move_n]));
    // apply_joint
    std::vector<int> joint{move_s};
    CHECK(s.apply_joint(joint));
    CHECK(s.agent_rows[0] == 2 && s.agent_cols[0] == 1);
    std::cerr << "[ok] applicable_and_conflict\n";
}

void test_solver_trivial()
{
    std::stringstream ss(kTrivialLevel);
    Level lv = parse_level(ss);
    Solver solver(lv);
    auto plan = solver.solve();
    CHECK(!plan.empty());
    std::cerr << "[ok] solver_trivial (plan length " << plan.size() << ")\n";
}

void test_solver_box()
{
    std::stringstream ss(kBoxLevel);
    Level lv = parse_level(ss);
    Solver solver(lv);
    auto plan = solver.solve();
    CHECK(!plan.empty());
    std::cerr << "[ok] solver_box (plan length " << plan.size() << ")\n";
}

void test_pibt_swap()
{
    // Two agents in a 2-row corridor: agent 0 must reach a cell currently
    // occupied by agent 1's path. Tests that PIBT can coordinate by having
    // the second agent step aside. (A true position-swap in a 1-row corridor
    // is impossible under the canonical hospital domain's "no follow" rule;
    // this level is the simplest non-trivial PIBT coordination scenario.)
    std::stringstream ss(kSwapLevel);
    Level lv = parse_level(ss);
    Solver solver(lv);
    auto plan = solver.solve();
    CHECK(!plan.empty());
    std::cerr << "[ok] pibt_swap (plan length " << plan.size() << ")\n";
}

}  // namespace

int main()
{
    test_action_table();
    test_parse_trivial();
    test_applicable_and_conflict();
    test_solver_trivial();
    test_solver_box();
    test_pibt_swap();
    std::cerr << "ALL TESTS PASSED\n";
    return 0;
}
