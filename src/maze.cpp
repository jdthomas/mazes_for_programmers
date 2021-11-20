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
#if defined(__GLIBCXX__)
#include <execution>
namespace pstl = std;
#else
#include <pstl/algorithm>
#include <pstl/execution>
#endif
#include <random>
#include <range/v3/all.hpp>
#include <set>
#include <unordered_map>
#include <vector>

#include "maze.h"

namespace stdex = std::experimental;

namespace jt::maze {

std::vector<int> dijkstra_distances(const Grid &grid,
                                    CellCoordinate start_cell) {
  std::deque<std::tuple<CellCoordinate, int>> q;
  q.push_back({start_cell, 1});

  fmt::print("Computing distances using dijkstra from {} \n", start_cell);
  std::vector<int> distances(grid.width_ * grid.height_);
  auto cell_distances = stdex::mdspan<
      int, stdex::extents<stdex::dynamic_extent, stdex::dynamic_extent>>(
      distances.data(), grid.height_, grid.width_);

  while (!q.empty()) {
    auto current = q.front();
    auto &[cell, depth] = current;
    q.pop_front();
    if (cell_distances(cell.row, cell.col) != 0) {
      continue;
    }
    cell_distances(cell.row, cell.col) = depth;
    auto neighbors = grid.get_connected_neighbors(cell);
    ranges::transform(neighbors, ranges::back_inserter(q),
                      [d = depth + 1](auto n) {
                        return std::tuple{n, d};
                      });
  }
  return distances;
}

std::tuple<CellCoordinate, int> furthest_cell(const Grid &grid,
                                              std::vector<int> &distances) {
  auto d = stdex::mdspan<
      int, stdex::extents<stdex::dynamic_extent, stdex::dynamic_extent>>(
      distances.data(), grid.height_, grid.width_);
  auto p = grid.positions();
  auto m = ranges::max_element(
      p, [](const auto &a, const auto &b) { return a < b; },
      [&](const auto &idx) {
        auto &[row, col] = idx;
        assert(row < grid.height_ && col < grid.width_);
        return d(row, col);
      });
  auto [row, col] = *m;
  return {CellCoordinate{row, col}, d(row, col)};
}

std::vector<CellCoordinate> longest_path_(Grid &grid,
                                          std::vector<int> &distances) {
  auto [c, d1] = furthest_cell(grid, distances);
  fmt::print("longest path from: {}x{} to {} and spans {} steps \n",
             grid.height_ / 2, grid.width_ / 2, c, d1);

  auto new_distances = dijkstra_distances(grid, c);
  auto [goal, longest_path_dist] = furthest_cell(grid, new_distances);

  fmt::print("longest path from: {} to {} and spans {} steps \n", c, goal,
             longest_path_dist);

  // walk path

  auto d = stdex::mdspan<
      int, stdex::extents<stdex::dynamic_extent, stdex::dynamic_extent>>(
      new_distances.data(), grid.height_, grid.width_);

  std::vector<CellCoordinate> path;
  path.reserve(longest_path_dist);

  auto step = goal;
  while (d(step.row, step.col) > 1) {
    path.push_back(step);
    auto neighbors = grid.get_connected_neighbors(step);
    // fmt::print(" traversing path({}) on our way from {} at {} (d:{}), going
    // to "
    //            "one of {}, {}\n",
    //            path.size(), goal, step, d(step.row, step.col), neighbors,
    //            neighbors | ranges::views::transform(
    //                            [d](auto n) { return d(n.row, n.col); }));
    step = *ranges::min_element(neighbors, ranges::less{},
                                [d](auto c) { return d(c.row, c.col); });
    // fmt::print("Next: {} (d:{})\n", step, d(step.row, step.col));
  }
  path.push_back(step);

  // fmt::print("  found path: {}", path);
  std::cout.flush();

  // Update distances for rendering
  distances = new_distances;
  return path;
}

bool operator<(const CellCoordinate &a, const CellCoordinate &b) {
  return a.row < b.row || (a.row == b.row && a.col < b.col);
}

bool operator==(const CellCoordinate &a, const CellCoordinate &b) {
  return a.row == b.row && a.col == b.col;
}
Grid::Grid(size_t width, size_t height)
    : width_{width}, height_{height}, cells_{width * height},
      mdspan_{cells_.data(), height_, width_}, rd{}, gen{rd()} {

  auto m = as_mdspan();
  for (int row = 0; row < m.extent(0); row++) {
    m(row, m.extent(1) - 1).right = Wall::Boundry;
  }
  for (int col = 0; col < m.extent(1); col++) {
    m(m.extent(0) - 1, col).down = Wall::Boundry;
  }
};

std::vector<GeneratorRegistry::RegistryConfig*> GeneratorRegistry::registry_{};

}; // namespace jt::maze

