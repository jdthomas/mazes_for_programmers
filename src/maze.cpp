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

void binary_tree_maze(Grid &grid) {
  std::bernoulli_distribution d(0.5);

  auto per_cell_action = [&](const auto &cell) {
    auto go_down = d(grid.gen);
    auto s = grid.cell_south(cell);
    auto e = grid.cell_east(cell);
    if (s && (go_down || !e))
      grid.link(cell, *s);
    else if (e)
      grid.link(cell, *e);
  };

  auto p = grid.positions();
  std::for_each(p.begin(), p.end(), per_cell_action);
}

void binary_tree_maze_p(Grid &grid) {

  std::bernoulli_distribution d(0.5);
  auto per_cell_action = [&](const auto &cell) {
    auto go_down = d(grid.gen);
    auto s = grid.cell_south(cell);
    auto e = grid.cell_east(cell);
    if (s && (go_down || !e))
      grid.link(cell, *s);
    else if (e)
      grid.link(cell, *e);
  };

  auto p = grid.positions();
  std::for_each(pstl::execution::par, p.begin(), p.end(), per_cell_action);
}

void binary_tree_maze_p2(Grid &grid) {
  // Row-wise parallel impl

  std::bernoulli_distribution d(0.5);
  auto per_cell_action = [&](const auto &cell) {
    auto go_down = d(grid.gen);
    auto s = grid.cell_south(cell);
    auto e = grid.cell_east(cell);
    if (s && (go_down || !e))
      grid.link(cell, *s);
    else if (e)
      grid.link(cell, *e);
  };

  auto r = ranges::views::iota(static_cast<size_t>(0),
                               static_cast<size_t>(grid.height_));
  std::for_each(pstl::execution::par_unseq, r.begin(), r.end(),
                [&](const auto &row) {
                  for (size_t col = 0; col < grid.width_; col++) {
                    per_cell_action(CellCoordinate{row, col});
                  }
                });
}
// TODO: There should be a version of binary_tree where we make the down/left decision first then just link them. e.g. no need for condition in the hot loops.

void sidewinder_maze(Grid &grid) {
  std::bernoulli_distribution d(0.5);

  auto r = ranges::views::iota(static_cast<size_t>(0),
                               static_cast<size_t>(grid.height_));
  std::for_each(r.begin(), r.end(), [&grid, &d](const auto &row) {
    size_t run_start = 0;
    for (size_t col = 0; col < grid.width_; col++) {
      auto e = grid.cell_east({row, col});
      auto s = grid.cell_south({row, col});
      bool should_close = !e || (s && d(grid.gen));
      if (should_close) {
        std::uniform_int_distribution<> distrib(run_start, col);
        size_t c = static_cast<size_t>(distrib(grid.gen));
        grid.link({row, c}, {row + 1, c});
        run_start = col + 1;
        c = run_start;
      } else {
        grid.link({row, col}, *e);
      }
    }
  });
}

void sidewinder_maze_p(Grid &grid) {
  std::bernoulli_distribution d(0.5);

  auto r = ranges::views::iota(static_cast<size_t>(0),
                               static_cast<size_t>(grid.height_));
  std::for_each(pstl::execution::par_unseq, r.begin(), r.end(),
                [&grid, &d](const auto &row) {
                  size_t run_start = 0;
                  for (size_t col = 0; col < grid.width_; col++) {
                    auto e = grid.cell_east({row, col});
                    auto s = grid.cell_south({row, col});
                    bool should_close = !e || (s && d(grid.gen));
                    if (should_close) {
                      std::uniform_int_distribution<> distrib(run_start, col);
                      size_t c = static_cast<size_t>(distrib(grid.gen));
                      grid.link({row, c}, {row + 1, c});
                      run_start = col + 1;
                    } else {
                      grid.link({row, col}, *e);
                    }
                  }
                });
}

void random_walk_Aldous_Broder_maze(Grid &grid) {
  auto cell = grid.random_cell();
  int unvisited = grid.width_ * grid.height_ - 1;
  while (unvisited > 0) {
    auto neighbor = *grid.random_neighbor(cell);

    if (grid.is_closed_cell(neighbor)) {
      grid.link(cell, neighbor);
      unvisited--;
    } else {
      cell = neighbor;
    }
  }
}

void random_walk_Wilson_maze(Grid &grid) {
  auto unvisited = grid.positions() | ranges::to<std::set>;
  auto first = grid.random_cell();
  unvisited.erase(first);

  while (unvisited.size() > 0) {
    // fmt::print("unvisited: {}\n", unvisited.size());
    auto cell = *jt_range_front(unvisited | ranges::views::sample(1, grid.gen));
    // fmt::print("cell: {} dis={}\n", cell,
    // std::distance(std::begin(unvisited), cell_i));
    assert(unvisited.find(cell) != ranges::end(unvisited));
    std::vector<CellCoordinate> path{cell};

    while (unvisited.find(cell) != ranges::end(unvisited)) {
      // fmt::print("   : {}, \n", cell);
      // Update cell to a random neighbor of cell
      cell = *grid.random_neighbor(cell);

      // Check if cell is in our path (e.g. loop)
      auto loop_begin =
          ranges::find_if(path, [cell](const auto &b) { return b == cell; });
      if (loop_begin == ranges::end(path)) {
        // no loop
        path.push_back(cell);
      } else {
        // fmt::print("Removing from path={}. {} ({})\n", path,
        //            std::distance(std::begin(path), loop_begin),
        //            *loop_begin);
        path.erase(std::next(loop_begin), std::end(path));
        // fmt::print("afterr remove path={}\n", path);
      }
    }

    // fmt::print("Path done att {}, link it: {}\n", cell, path);
    auto _ = ranges::adjacent_find(path, [&grid](auto a, auto b) {
      grid.link(a, b);
      return false;
    });
    ranges::for_each(path, [&unvisited](auto a) { unvisited.erase(a); });
  }
}

void hunt_and_kill_maze(Grid &grid) {
  std::optional<CellCoordinate> current = grid.random_cell();

  while (current) {
    auto n = grid.random_closed_neighbor(*current);

    if (n) {
      grid.link(*current, *n);
      current = n;
    } else {
      auto p = grid.positions();
      auto next = ranges::find_if(p, [&](auto pos) {
        auto n = grid.get_all_neighbors(pos);
        auto unvisited_neighbors_of_pos =
            ranges::accumulate(n | ranges::views::transform([&grid](auto np) {
                                 return grid.is_closed_cell(np) ? 1 : 0;
                               }),
                               0);
        return (!grid.is_closed_cell(pos) && unvisited_neighbors_of_pos > 0);
      });
      current =
          (next == ranges::end(p)) ? std::nullopt : std::make_optional(*next);
    }
  }
}

void recursive_backtracking_maze(Grid &grid) {
  std::vector<CellCoordinate> stack{grid.random_cell()};

  while (!stack.empty()) {
    auto current = stack.back();

    auto n = grid.random_closed_neighbor(current);
    if (n) {
      grid.link(current, *n);
      stack.push_back(*n);
    } else {
      stack.pop_back();
    }
  }
}

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

}; // namespace jt::maze

size_t method = 0;
std::array<char, 6> all_methods{'B', 'S', 'R', 'W', 'K', 'C'};
std::string method_name(char m) {
  switch (m) {
  case 'B':
    return "BinaryTree";
  case 'S':
    return "Sidewinder";
  case 'R':
    return "AldousBroder";
  case 'W':
    return "Wilson";
  case 'K':
    return "HuntAndKill";
  case 'C':
    return "RecursiveBacktraking";
  }
  return "unknown";
}

std::vector<int> gen_maze(jt::maze::Grid &grid) {
  fmt::print("Generating maze by {}\n", method_name(all_methods[method]));
  auto method_c = all_methods[method];
  using std::chrono::high_resolution_clock;
  auto t1 = high_resolution_clock::now();
  switch (method_c) {
  case 'B':
    binary_tree_maze(grid);
    break;
  case 'S':
    sidewinder_maze(grid);
    break;
  case 'R':
    random_walk_Aldous_Broder_maze(grid);
    break;
  case 'W':
    random_walk_Wilson_maze(grid);
    break;
  case 'K':
    hunt_and_kill_maze(grid);
    break;
  case 'C':
    recursive_backtracking_maze(grid);
    break;
  }
  auto t2 = high_resolution_clock::now();

  std::chrono::duration<double, std::milli> delta_ms = t2 - t1;
  fmt::print("Generated maze in {}\n", delta_ms);
  fmt::print("{}\n", grid);
  auto distances =
      dijkstra_distances(grid, {grid.width_ / 2, grid.height_ / 2});
  // fmt::print("D:{}\n", distances);

  auto p = grid.positions();
  auto dead_ends =
      ranges::accumulate(p | ranges::views::transform([&grid](auto pos) {
                           return grid.is_dead_end_cell(pos) ? 1 : 0;
                         }),
                         0);
  fmt::print("Dead ends: {}\n", dead_ends);

  return distances;
}
