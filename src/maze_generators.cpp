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
#include <queue>
#include <random>
#include <range/v3/all.hpp>
#include <set>
#include <unordered_map>
#include <unordered_set>
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
                  for (size_t col = 0; col < grid.widths_.back(); col++) {
                    per_cell_action(CellCoordinate{row, col});
                  }
                });
}
GeneratorRegistry::RegisterGenerator b3('D', "BinaryTree(Parallel - 2)",
                                        binary_tree_maze_p2);

//////////////////////////////////////////////////////////////////////////////
// Sidewinder
//////////////////////////////////////////////////////////////////////////////
void sidewinder_maze(Grid &grid) {
  std::bernoulli_distribution d(0.5);

  auto r = ranges::views::iota(static_cast<size_t>(0),
                               static_cast<size_t>(grid.height_));
  std::for_each(r.begin(), r.end(), [&grid, &d](const auto &row) {
    size_t run_start = 0;
    for (size_t col = 0; col < grid.widths_.back(); col++) {
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
                  for (size_t col = 0; col < grid.widths_.back(); col++) {
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
  int unvisited = grid.widths_.back() * grid.height_ - 1;
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

void kruskel_maze(Grid &grid) {
  // n a nutshell, then, the algorithm is:
  //      1. Assign each cell to its own set.
  //      2. Choose the pair of neighboring cells with the lowest cost passage
  //      between them.
  //      3. If the two cells belong to different sets, merge them.
  //      4. Repeat 2 and 3 until only a single set remains
  std::unordered_map<CellCoordinate, int, cell_coordinate_hash_fn>
      set_for_cell{};
  std::unordered_map<int, std::set<CellCoordinate>> cells_in_set{};
  std::vector<std::pair<CellCoordinate, CellCoordinate>> neighbors;
  auto can_merge = [&](auto left, auto right) {
    // fmt::print("Can merge {} to {}? {} == {}\n", left, right,
    // set_for_cell[left], set_for_cell[right]);
    return set_for_cell[left] != set_for_cell[right];
  };
  auto merge = [&](auto left, auto right) {
    grid.link(left, right);
    auto winner = set_for_cell[left];
    auto loser = set_for_cell[right];
    auto &losers = cells_in_set[loser];

    // fmt::print("Linking: {} ({}) to {} ({}) -- {}\n", left, winner, right,
    // loser, losers);

    // Add the loser set items to the winner set
    cells_in_set[winner].merge(losers);
    // And update all of those to same set
    for (const auto &cell : cells_in_set[winner]) {
      // fmt::print("moving {} to {} (was {})\n", cell, winner,
      // set_for_cell[cell]);
      set_for_cell[cell] = winner;
    }
    // And blow away loser set
    cells_in_set.erase(loser);
  };
  auto add_crossing = [&](auto cell) -> bool {
    auto n = grid.cell_north(cell);
    auto e = grid.cell_east(cell);
    auto s = grid.cell_south(cell);
    auto w = grid.cell_west(cell);

    if (!grid.is_closed_cell(cell) || !e || !w || !can_merge(*e, *w) || !n ||
        !s || !can_merge(*n, *s)) {
      return false;
    }
    neighbors.erase(std::remove_if(neighbors.begin(), neighbors.end(),
                                   [&](auto n) {
                                     auto &[left, right] = n;
                                     return left == cell || right == cell;
                                   }),
                    neighbors.end());

    std::bernoulli_distribution d(0.5);
    if (d(grid.gen)) {
      merge(*w, cell);
      merge(cell, *e);
      merge(*n, *s);
    } else {
      merge(*n, cell);
      merge(*s, cell);
      merge(*w, *e);
    }
    return true;
  };

  // Init State
  auto p = grid.positions();
  for (auto x : ranges::views::enumerate(p)) {
    auto &[i, cell] = x;
    set_for_cell[cell] = int(i);
    cells_in_set[int(i)].insert(cell);
    {
      auto south = grid.cell_south(cell);
      auto east = grid.cell_east(cell);
      if (south) neighbors.emplace_back(cell, *south);
      if (east) neighbors.emplace_back(cell, *east);
    }
  }
  // crossings
  if (grid.enable_weaving) {
    // TODO: Paramatarize how many crosses to attempt adding
    for (int i = 0; i < grid.widths_.back() * grid.height_; i++) {
      auto c = grid.random_cell();
      add_crossing(c);
    }
  }
  // Impl
  std::shuffle(neighbors.begin(), neighbors.end(), grid.gen);
  while (!neighbors.empty()) {
    auto [left, right] = neighbors.back();
    neighbors.pop_back();
    if (can_merge(left, right)) {
      merge(left, right);
    }
  }
  // fmt::print("state: \n\t{} \n\n\t{}\n", set_for_cell, cells_in_set);
}
GeneratorRegistry::RegisterGenerator kr('U', "Kruskel", kruskel_maze);

void prims_maze(Grid &grid) {
  std::uniform_real_distribution<float> d(0.0f, 1.0f);
  std::priority_queue<std::pair<float, CellCoordinate>> active;
  active.emplace(d(grid.gen), grid.random_cell());
  while (!active.empty()) {
    auto [r, cell] = active.top();
    auto n = grid.random_closed_neighbor(cell);
    if (n) {
      active.emplace(d(grid.gen), *n);
      grid.link(cell, *n);
    } else {
      active.pop();
    }
  }
}
GeneratorRegistry::RegisterGenerator pr('P', "Prims", prims_maze);

void all_walls_maze(Grid &grid) {
  // default is all walls
}
GeneratorRegistry::RegisterGenerator z('Z', "AllWalls", all_walls_maze);

};  // namespace jt::maze
