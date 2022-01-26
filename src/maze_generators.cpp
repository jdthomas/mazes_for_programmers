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
                               static_cast<size_t>(grid.height_ - 1));
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
  // FIXME: Fix last rows
  for (size_t col = 0; col < grid.widths_.back() - 1; col++) {
    const size_t row = grid.height_ - 1;
    grid.link({row, col}, {row, col + 1});
  }
}
GeneratorRegistry::RegisterGenerator s('S', "Sidewinder", sidewinder_maze);

void sidewinder_maze_p(Grid &grid) {
  std::bernoulli_distribution d(0.5);

  auto r = ranges::views::iota(static_cast<size_t>(0),
                               static_cast<size_t>(grid.height_ - 1));
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
  // FIXME: Fix last rows
  for (size_t col = 0; col < grid.widths_.back() - 1; col++) {
    const size_t row = grid.height_ - 1;
    grid.link({row, col}, {row, col + 1});
  }
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

//////////////////////////////////////////////////////////////////////////////
// Set Based
//////////////////////////////////////////////////////////////////////////////

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

namespace {
template <typename T>
void growing_maze(Grid &grid, T active_selector) {
  std::vector<CellCoordinate> active;
  active.emplace_back(grid.random_cell());
  while (!active.empty()) {
    auto cell = active_selector(active);
    auto n = grid.random_closed_neighbor(cell);
    if (n) {
      grid.link(cell, *n);
      active.emplace_back(*n);
    } else {
      active.erase(std::remove_if(active.begin(), active.end(),
                                  [&cell](const auto &a) { return a == cell; }),
                   active.end());
    }
  }
}
};  // namespace

void grow_sample_maze(Grid &grid) {
  return growing_maze(grid, [&](const auto &active) {
    // random element
    std::uniform_int_distribution<> dis(
        0, std::distance(active.begin(), active.end()) - 1);
    return active[dis(grid.gen)];
  });
}
GeneratorRegistry::RegisterGenerator gs('1', "GrowSample", grow_sample_maze);

void grow_last_maze(Grid &grid) {
  return growing_maze(grid, [&](const auto &active) {
    // last element
    return active.back();
  });
}
GeneratorRegistry::RegisterGenerator gl('2', "GrowLast", grow_last_maze);

void grow_last_or_sample_maze(Grid &grid) {
  return growing_maze(grid, [&](const auto &active) {
    // random element
    std::uniform_int_distribution<> dis(
        0, std::distance(active.begin(), active.end()) - 1);
    std::bernoulli_distribution d(0.5);
    return d(grid.gen) ? active[dis(grid.gen)] : active.back();
  });
}
GeneratorRegistry::RegisterGenerator glos('3', "GrowLastOrSample",
                                          grow_last_or_sample_maze);

void ellers_maze(Grid &grid) {
  std::unordered_map<size_t, int> set_for_col{};
  std::unordered_map<int, std::set<CellCoordinate>> cells_in_set{};
  std::bernoulli_distribution d(0.5);
  int next_set = 0;

  auto merge = [&](auto winner, auto loser) {
    // grid.link(left, right);
    // auto winner = set_for_col[left.col];
    // auto loser = set_for_col[right.col];
    auto &losers = cells_in_set[loser];

    // fmt::print("Linking: {} ({}) to {} ({}) -- {}\n", left, winner, right,
    // loser, losers);

    // Add the loser set items to the winner set
    cells_in_set[winner].merge(losers);
    // And update all of those to same set
    for (const auto &cell : cells_in_set[winner]) {
      // fmt::print("moving {} to {} (was {})\n", cell, winner,
      // set_for_col[cell]);
      set_for_col[cell.col] = winner;
    }
    // And blow away loser set
    cells_in_set.erase(loser);
  };

  auto record = [&](auto set, auto cell) {
    set_for_col[cell.col] = set;
    cells_in_set[set].insert(cell);
  };

  auto set_for = [&](auto cell) {
    if (0 == set_for_col.count(cell.col)) {
      record(next_set, cell);
      ++next_set;
    }
    return set_for_col[cell.col];
  };

  auto r = ranges::views::iota(static_cast<size_t>(0),
                               static_cast<size_t>(grid.height_));
  std::for_each(r.begin(), r.end(), [&](const auto &row) {
    for (size_t col = 0; col < grid.widths_.back(); col++) {
      CellCoordinate cell{row, col};
      auto w = grid.cell_west(cell);
      if (!w) continue;
      auto set = set_for(cell);
      auto prior_set = set_for(*w);
      bool should_link =
          set != prior_set && (!grid.cell_south(cell) || d(grid.gen));
      if (should_link) {
        grid.link(cell, *w);
        merge(prior_set, set);
      }
    }

    if (grid.cell_south(CellCoordinate{row, 0})) {
      decltype(set_for_col) prev_set_for_col;
      decltype(cells_in_set) prev_cells_in_set;
      std::swap(prev_set_for_col, set_for_col);
      std::swap(prev_cells_in_set, cells_in_set);
      // auto prev_set_for_col = set_for_col;
      // auto prev_cells_in_set = cells_in_set;
      // set_for_col.clear();
      // cells_in_set.clear();
      for (auto &[set, cells] : prev_cells_in_set) {
        std::bernoulli_distribution d3(0.33);
        int link_ct = 0;
        for (auto cell : cells) {
          if (d3(grid.gen)) {
            link_ct++;
            grid.link(cell, *grid.cell_south(cell));
            record(prev_set_for_col[cell.col], *grid.cell_south(cell));
          }
        }
        if (link_ct == 0) {
          // FIXME: Pick a random cell?
          auto cell = *cells.begin();
          grid.link(cell, *grid.cell_south(cell));
          record(prev_set_for_col[cell.col], *grid.cell_south(cell));
        }
      }
    }
  });
}
GeneratorRegistry::RegisterGenerator el('4', "Ellers", ellers_maze);

namespace {
void recursive_division_maze_divide_h(Grid &grid, size_t row, size_t col,
                                      size_t height, size_t width);
void recursive_division_maze_divide_v(Grid &grid, size_t row, size_t col,
                                      size_t height, size_t width);

void recursive_division_maze_divide(Grid &grid, size_t row, size_t col,
                                    size_t height, size_t width) {
  if (height <= 1 || width <= 1) {
    return;
  }
  if (height > width)
    recursive_division_maze_divide_h(grid, row, col, height, width);
  else
    recursive_division_maze_divide_v(grid, row, col, height, width);
}
void recursive_division_maze_divide_h(Grid &grid, size_t row, size_t col,
                                      size_t height, size_t width) {
  std::uniform_int_distribution<> dis_h(0, height - 1);
  std::uniform_int_distribution<> dis_w(0, width);
  auto divide_south_of = dis_h(grid.gen);
  auto passage_at = dis_w(grid.gen);
  for (int x = 0; x < width - 1; ++x) {
    if (x == passage_at) {
      continue;
    }
    CellCoordinate cell{row + divide_south_of, col + x};
    auto s = grid.cell_south(cell);
    if (s) {
      grid.unlink(cell, *s);
    }
  }
  recursive_division_maze_divide(grid, row, col, divide_south_of + 1, width);
  recursive_division_maze_divide(grid, row + divide_south_of + 1, col,
                                 height - divide_south_of - 1, width);
}
void recursive_division_maze_divide_v(Grid &grid, size_t row, size_t col,
                                      size_t height, size_t width) {
  std::uniform_int_distribution<> dis_h(0, height);
  std::uniform_int_distribution<> dis_w(0, width - 1);
  auto divide_east_of = dis_w(grid.gen);
  auto passage_at = dis_h(grid.gen);
  for (int x = 0; x < height - 1; ++x) {
    if (x == passage_at) {
      continue;
    }
    CellCoordinate cell{row + x, col + divide_east_of};
    auto e = grid.cell_east(cell);
    if (e) {
      grid.unlink(cell, *e);
    }
  }
  recursive_division_maze_divide(grid, row, col, height, divide_east_of + 1);
  recursive_division_maze_divide(grid, row, col + divide_east_of + 1, height,
                                 width - divide_east_of - 1);
}
};  // namespace

void recursive_division_maze(Grid &grid) {
  grid.reset_open();
  recursive_division_maze_divide(grid, 0, 0, grid.height_, grid.widths_.back());
}
GeneratorRegistry::RegisterGenerator rd('5', "RecursiveDivision",
                                        recursive_division_maze);

void no_walls_maze(Grid &grid) { grid.reset_open(); }
GeneratorRegistry::RegisterGenerator o('N', "NoWalls", no_walls_maze);
void all_walls_maze(Grid &grid) {
  // default is all walls
}
GeneratorRegistry::RegisterGenerator z('Z', "AllWalls", all_walls_maze);

};  // namespace jt::maze
