#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <experimental/mdspan>
#include <fmt/chrono.h>
#include <fmt/format.h>
#include <fmt/ranges.h>
#include <iostream>
#include <mutex>
#include <pstl/algorithm>
#include <pstl/execution>
#include <random>
#include <range/v3/all.hpp>
#include <set>
#include <unordered_map>
#include <vector>

#include "maze.h"

int main(int argc, char **argv) {

  if (argc < 2)
    return 1;
  jt::maze::Grid grid{static_cast<size_t>(std::strtol(argv[1], nullptr, 10)),
                      static_cast<size_t>(std::strtol(argv[2], nullptr, 10))};
  std::unordered_map<char, float> averages;
  // we run stats instead!
  auto method_id_for_char = [](auto m) {
    return std::distance(
                         std::begin(all_methods),
                         std::find(std::begin(all_methods), std::end(all_methods), m));
  };
  for (auto m : all_methods) {
    // if (m != 'B') continue;
    method = method_id_for_char(m);
    std::vector<int> dead_count;
    for (int i = 0; i < 100; i++) {
      grid.reset();
      gen_maze(grid);
      auto p = grid.positions();
      auto dead_ends =
          ranges::accumulate(p | ranges::views::transform([&grid](auto pos) {
                               return grid.is_dead_end_cell(pos) ? 1 : 0;
                             }),
                             0);
      dead_count.push_back(dead_ends);
    }
    averages[m] = float(ranges::accumulate(dead_count, 0)) / dead_count.size();
  }
  auto cell_count = grid.width_ * grid.height_;
  fmt::print("Average dead-ends per {}x#{} maze ({} cells):\n", grid.width_,
             grid.height_, cell_count);
  for (auto kv : averages) {
    auto &[k, v] = kv;
    fmt::print("{}: {}/{} ({}%)\n", method_name(k), v, cell_count,
               100 * v / cell_count);
    // AldousBroder : 115/400 (28%)
  }
  return 0;
}
