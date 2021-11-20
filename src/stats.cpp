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
  std::unordered_map<std::string, float> averages;
  jt::maze::ensure_registry();
  // we run stats instead!
  for (auto &m : jt::maze::GeneratorRegistry::AllMethods()) {

    std::vector<int> dead_count;
    for (int i = 0; i < 100; i++) {
      grid.reset();
      m.generate(grid);
      auto p = grid.positions();
      auto dead_ends =
          ranges::accumulate(p | ranges::views::transform([&grid](auto pos) {
                               return grid.is_dead_end_cell(pos) ? 1 : 0;
                             }),
                             0);
      dead_count.push_back(dead_ends);
    }
    averages[m.name] =
        float(ranges::accumulate(dead_count, 0)) / dead_count.size();
  }
  auto cell_count = grid.width_ * grid.height_;
  fmt::print("Average dead-ends per {}x#{} maze ({} cells):\n", grid.width_,
             grid.height_, cell_count);

  std::vector<std::pair<std::string, float>> sorted_avgs;
  std::copy(averages.begin(), averages.end(), std::back_inserter(sorted_avgs));

  std::sort(sorted_avgs.begin(), sorted_avgs.end(),
            [](const auto &a, const auto &b) { return a.second < b.second; });
  for (auto &kv : sorted_avgs) {
    auto &[k, v] = kv;
    fmt::print("    {}: {}/{} ({}%)\n", k, v, cell_count, 100 * v / cell_count);
  }
  return 0;
}
