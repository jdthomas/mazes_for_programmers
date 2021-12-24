#include <fmt/chrono.h>
#include <fmt/format.h>
#include <fmt/ranges.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <experimental/mdspan>
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

namespace jt::maze {
void ensure_registry() {}
//////////////////////////////////////////////////////////////////////////////
// Binary Tree
//////////////////////////////////////////////////////////////////////////////
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
GeneratorRegistry::RegisterGenerator b('B', "BinaryTree", binary_tree_maze);

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
GeneratorRegistry::RegisterGenerator b2('b', "BinaryTree(Parallel - 1)",
                                        binary_tree_maze_p);

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
GeneratorRegistry::RegisterGenerator b3('D', "BinaryTree(Parallel - 2)",
                                        binary_tree_maze_p2);

// TODO: There should be a version of binary_tree where we make the down/left
// decision first then just link them. e.g. no need for condition in the hot
// loops.

//////////////////////////////////////////////////////////////////////////////
// Sidewinder
//////////////////////////////////////////////////////////////////////////////
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
GeneratorRegistry::RegisterGenerator s('S', "Sidewinder", sidewinder_maze);

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
GeneratorRegistry::RegisterGenerator s2('s', "Sidewinder(Parallel - 1)",
                                        sidewinder_maze_p);

//////////////////////////////////////////////////////////////////////////////
// Random Walks
//////////////////////////////////////////////////////////////////////////////
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
GeneratorRegistry::RegisterGenerator ab('A', "AldousBroder",
                                        random_walk_Aldous_Broder_maze);

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
GeneratorRegistry::RegisterGenerator w('W', "Wilson", random_walk_Wilson_maze);

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
GeneratorRegistry::RegisterGenerator k('K', "HuntAndKill", hunt_and_kill_maze);

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
GeneratorRegistry::RegisterGenerator r('R', "RecursiveBacktracking",
                                       recursive_backtracking_maze);

};  // namespace jt::maze
