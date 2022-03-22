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

namespace stdex = std::experimental;

namespace jt::maze {
std::random_device Grid::rd{};

std::vector<int> calculate_variable_widths(int height) {
  auto widths = ranges::views::iota(1, static_cast<int>(height + 1)) |
                ranges::views::partial_sum([height](auto acc, auto row) {
                  const float row_height = 1.0 / height;
                  const float rad = float(row) / height;
                  const float circ = 2 * M_PI * rad;

                  const auto estimated_cell_width = circ / acc;
                  const auto ratio =
                      std::round(estimated_cell_width / row_height);
                  const int cells = acc * ratio;
                  fmt::print("pi={}\n", M_PI);
                  fmt::print("rad={}, rh={} circ={} height={}\n", rad,
                             row_height, circ, height);
                  fmt::print("acc={}, row={} ratio={} cells={}\n", acc, row,
                             ratio, cells);
                  return cells;
                });
  // Make a mask from these
  fmt::print("widths: {}\n", widths);
  return widths | ranges::to<std::vector<int>>;
}

std::vector<int> dijkstra_distances(const Grid &grid,
                                    CellCoordinate start_cell) {
  std::deque<std::tuple<CellCoordinate, int>> q;
  q.push_back({start_cell, 1});

  fmt::print("Computing distances using dijkstra from {} \n", start_cell);
  std::vector<int> distances(grid.grid_settings.widths.back() *
                             grid.grid_settings.height);
  auto cell_distances = stdex::mdspan<
      int, stdex::extents<stdex::dynamic_extent, stdex::dynamic_extent>>(
      distances.data(), grid.grid_settings.height,
      grid.grid_settings.widths.back());

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
                        // FIXME: Need to add an extra cost if crossing an
                        // undercell
                        return std::tuple{n, d};
                      });
  }
  return distances;
}

std::tuple<CellCoordinate, int> furthest_cell(const Grid &grid,
                                              std::vector<int> &distances) {
  auto d = stdex::mdspan<
      int, stdex::extents<stdex::dynamic_extent, stdex::dynamic_extent>>(
      distances.data(), grid.grid_settings.height,
      grid.grid_settings.widths.back());
  auto p = grid.positions();
  auto m = ranges::max_element(
      p, [](const auto &a, const auto &b) { return a < b; },
      [&](const auto &idx) {
        auto &[row, col] = idx;
        assert(row < grid.grid_settings.height &&
               col < grid.grid_settings.widths.back());
        return d(row, col);
      });
  auto [row, col] = *m;
  return {CellCoordinate{row, col}, d(row, col)};
}

std::vector<CellCoordinate> longest_path_(Grid &grid,
                                          std::vector<int> &distances) {
  auto [c, d1] = furthest_cell(grid, distances);
  fmt::print("longest path from: {}x{} to {} and spans {} steps \n",
             grid.grid_settings.height / 2,
             grid.grid_settings.widths.back() / 2, c, d1);

  auto new_distances = dijkstra_distances(grid, c);
  auto [goal, longest_path_dist] = furthest_cell(grid, new_distances);

  fmt::print("longest path from: {} to {} and spans {} steps \n", c, goal,
             longest_path_dist);

  // walk path

  auto d = stdex::mdspan<
      int, stdex::extents<stdex::dynamic_extent, stdex::dynamic_extent>>(
      new_distances.data(), grid.grid_settings.height,
      grid.grid_settings.widths.back());

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

std::vector<GeneratorRegistry::RegistryConfig> GeneratorRegistry::registry_{};

void braid_maze(Grid &grid, float p) {
  auto pos = grid.positions();
  {
    auto dead_ends =
        ranges::accumulate(pos | ranges::views::transform([&](auto c) {
                             return grid.is_dead_end_cell(c) ? 1 : 0;
                           }),
                           0);
    fmt::print("--- Dead ends before braid: {}\n", dead_ends);
  }
  for (const auto &cell :
       pos | ranges::views::filter([&grid](const auto &cell) {
         return grid.is_dead_end_cell(cell);
       })) {
    std::bernoulli_distribution d(p);
    if (!d(grid.gen)) {
      continue;
    }
    auto neighbors = grid.get_unconnected_neighbors(cell);
    ranges::shuffle(neighbors);
    ranges::actions::sort(neighbors, std::less{}, [&](auto c) {
      return grid.get_unconnected_neighbors(c).size();
    });
    // pick best unlinked neighbor and link
    // auto best = neighbors | ranges::views::filter( count_neighbors == 1);
    // fmt::print("{} : {} -> {}\n", neighbors, cell, neighbors.front());
    grid.link(cell, neighbors.front());
  }
  {
    auto dead_ends =
        ranges::accumulate(pos | ranges::views::transform([&](auto c) {
                             return grid.is_dead_end_cell(c) ? 1 : 0;
                           }),
                           0);
    fmt::print("--- Dead ends after braid: {}\n", dead_ends);
  }
}

bool Grid::is_crossing_undercell(CellCoordinate over,
                                 CellCoordinate under) const {
  return is_under_cell(under) && is_linked(over, under) &&
         !is_linked(under, over);
}

// Helpers to get neighboring CellCoordinate for each direction
std::optional<CellCoordinate> Grid::cell_north(CellCoordinate c) const {
  return get_all_neighbors_(c)[to_underlying(Direction::N)];
}
std::optional<CellCoordinate> Grid::cell_east(CellCoordinate c) const {
  return get_all_neighbors_(c)[to_underlying(Direction::E)];
}
std::optional<CellCoordinate> Grid::cell_south(CellCoordinate c) const {
  return get_all_neighbors_(c)[to_underlying(Direction::S)];
}
std::optional<CellCoordinate> Grid::cell_west(CellCoordinate c) const {
  return get_all_neighbors_(c)[to_underlying(Direction::W)];
}

std::optional<CellCoordinate> Grid::cell_north_west(CellCoordinate c) const {
  if constexpr (cell_shape_is_square()) {
    const bool even_col = c.col % 2 == 0;
    return even_col ? std::nullopt
                    : get_all_neighbors_(c)[to_underlying(Direction::W)];
  } else {
    return get_all_neighbors_(c)[to_underlying(Direction::NW)];
  }
}

std::optional<CellCoordinate> Grid::cell_north_east(CellCoordinate c) const {
  if constexpr (cell_shape_is_square()) {
    const bool even_col = c.col % 2 == 0;
    return even_col ? std::nullopt
                    : get_all_neighbors_(c)[to_underlying(Direction::E)];
  } else {
    return get_all_neighbors_(c)[to_underlying(Direction::NE)];
  }
}

std::optional<CellCoordinate> Grid::cell_south_west(CellCoordinate c) const {
  if constexpr (cell_shape_is_square()) {
    const bool even_col = c.col % 2 == 0;
    return !even_col ? std::nullopt
                     : get_all_neighbors_(c)[to_underlying(Direction::W)];
  } else {
    return get_all_neighbors_(c)[to_underlying(Direction::SW)];
  }
}

std::optional<CellCoordinate> Grid::cell_south_east(CellCoordinate c) const {
  if constexpr (cell_shape_is_square()) {
    const bool even_col = c.col % 2 == 0;
    return !even_col ? std::nullopt
                     : get_all_neighbors_(c)[to_underlying(Direction::E)];
  } else {
    return get_all_neighbors_(c)[to_underlying(Direction::SE)];
  }
}

bool Grid::is_connected_directly_north(CellCoordinate c) const {
  auto n = cell_north(c);
  return n && is_linked(c, *n);
}
bool Grid::is_connected_directly_west(CellCoordinate c) const {
  auto n = cell_west(c);
  return n && is_linked(c, *n);
}
bool Grid::is_connected_directly_south(CellCoordinate c) const {
  auto n = cell_south(c);
  return n && is_linked(c, *n);
}
bool Grid::is_connected_directly_east(CellCoordinate c) const {
  auto n = cell_east(c);
  return n && is_linked(c, *n);
}

//
std::optional<CellCoordinate> Grid::connected_cell_north(
    CellCoordinate c) const {
  auto n = cell_north(c);
  if (grid_settings.enable_weaving && n && is_crossing_undercell(c, *n))
    return cell_north(*n);
  return n && is_linked(c, *n) ? n : std::nullopt;
}
std::optional<CellCoordinate> Grid::connected_cell_east(
    CellCoordinate c) const {
  auto n = cell_east(c);
  if (grid_settings.enable_weaving && n && is_crossing_undercell(c, *n))
    return cell_east(*n);
  return n && is_linked(c, *n) ? n : std::nullopt;
}
std::optional<CellCoordinate> Grid::connected_cell_south(
    CellCoordinate c) const {
  auto n = cell_south(c);
  if (grid_settings.enable_weaving && n && is_crossing_undercell(c, *n))
    return cell_south(*n);
  return n && is_linked(c, *n) ? n : std::nullopt;
}
std::optional<CellCoordinate> Grid::connected_cell_west(
    CellCoordinate c) const {
  auto n = cell_west(c);
  if (grid_settings.enable_weaving && n && is_crossing_undercell(c, *n))
    return cell_west(*n);
  return n && is_linked(c, *n) ? n : std::nullopt;
}

std::optional<CellCoordinate> Grid::connected_cell_north_west(
    CellCoordinate c) const {
  auto n = cell_north_west(c);
  return n && is_linked(c, *n) ? n : std::nullopt;
}
std::optional<CellCoordinate> Grid::connected_cell_south_west(
    CellCoordinate c) const {
  auto n = cell_south_west(c);
  return n && is_linked(c, *n) ? n : std::nullopt;
}
std::optional<CellCoordinate> Grid::connected_cell_north_east(
    CellCoordinate c) const {
  auto n = cell_north_east(c);
  return n && is_linked(c, *n) ? n : std::nullopt;
}
std::optional<CellCoordinate> Grid::connected_cell_south_east(
    CellCoordinate c) const {
  auto n = cell_south_east(c);
  return n && is_linked(c, *n) ? n : std::nullopt;
}

// Helpers for getting all neighbors
Grid::AdjacentCells Grid::get_all_neighbors_(CellCoordinate c) const {
  auto neighbors = adjacent_cells(c);
  // Fixup for wrapping (never wrap N/S, optionally wrap E/W)
  neighbors |= ranges::actions::transform(
      [this, c](auto n) -> std::optional<CellCoordinate> {
        return !n || (std::abs(c.row - n->row) > 1) ||
                       (std::abs(c.col - n->col) > 1 &&
                        !grid_settings.allow_ew_wrap)
                   ? std::nullopt
                   : n;
      });
  // FIXME: filter for masking
  // neighbors |= ranges::actions::transform(
  //     [this, c](auto n) -> std::optional<CellCoordinate> {
  //       return !n || masked_at(n) ? std::nullopt : n;
  //     });
  return neighbors;
}

// Helper for getting all not-connected neighbors
Grid::AdjacentCells Grid::get_unconnected_neighbors_(CellCoordinate c) {
  auto neighbors = get_all_neighbors_(c);
  neighbors |= ranges::actions::transform([this, &c](const auto &c2) {
    return c2 && !is_linked(c, *c2) ? c2 : std::nullopt;
  });
  return neighbors;
}

// Helpers for getting all connected neighbors
Grid::AdjacentCells Grid::get_connected_neighbors_(CellCoordinate c) const {
  auto neighbors = get_all_neighbors_(c);
  neighbors |= ranges::actions::transform(
      [this, c](auto n) { return n && is_linked(c, *n) ? n : std::nullopt; });
  return neighbors;
}

size_t Grid::count_connected_neighbors(const CellCoordinate c) const {
  auto tmp = get_connected_neighbors_(c);
  return ranges::distance(
      tmp | ranges::views::filter([](auto n) { return bool(n); }));
};

// Helper to check if a cell has no connections
bool Grid::is_closed_cell(CellCoordinate c) const {
  return count_connected_neighbors(c) == 0;
}

// Helper to check if a cell is a dead end (has only one connecttion)
bool Grid::is_dead_end_cell(CellCoordinate c) const {
  return count_connected_neighbors(c) == 1;
}

bool Grid::is_h_passage_cell(CellCoordinate c) const {
  return !connected_cell_north(c) && !connected_cell_south(c) &&
         connected_cell_east(c) && connected_cell_west(c);
}
bool Grid::is_v_passage_cell(CellCoordinate c) const {
  return connected_cell_north(c) && connected_cell_south(c) &&
         !connected_cell_east(c) && !connected_cell_west(c);
}

bool Grid::can_tunnel_north(CellCoordinate c) const {
  auto n = cell_north(c);
  return n && cell_north(*n) && !is_under_cell(*n) && is_h_passage_cell(*n);
}
bool Grid::can_tunnel_south(CellCoordinate c) const {
  auto n = cell_south(c);
  return n && cell_south(*n) && !is_under_cell(*n) && is_h_passage_cell(*n);
}
bool Grid::can_tunnel_west(CellCoordinate c) const {
  auto n = cell_west(c);
  return n && cell_west(*n) && !is_under_cell(*n) && is_v_passage_cell(*n) &&
         c.col > 0 && n->col > 0;
}
bool Grid::can_tunnel_east(CellCoordinate c) const {
  auto n = cell_east(c);
  return n && cell_east(*n) && !is_under_cell(*n) && is_v_passage_cell(*n) &&
         c.col < grid_settings.widths[c.row] - 1 &&
         n->col < grid_settings.widths[n->row] - 1;
}

////////////////////////////////////////////////////////////////////////////////

bool Grid::is_linked(CellCoordinate c1, CellCoordinate c2) const {
  auto neighbors = get_all_neighbors_(c1);
  auto n = ranges::find_if(neighbors, [c2](auto n) { return n && *n == c2; });
  if (n == neighbors.end()) {
    // throw std::runtime_error(fmt::format("Bad neighbor: {} -> {}", c1,
    // c2));
    return false;
  }
  return (grid_view_(c1.row, c1.col).walls &
          (1 << ranges::distance(neighbors.begin(), n))) == 0;
}

void Grid::unlink(CellCoordinate c1, CellCoordinate c2) {
  link_(c1, c2, false);
  link_(c2, c1, false);
}
// Helper for linking two cells togetherr (must be neighbors)
void Grid::link(CellCoordinate c1, CellCoordinate c2) {
  // Weaving:
  // Check if we are a neighbor-neighbor
  if (grid_settings.enable_weaving && (std::abs(c1.row - c2.row) == 2) ||
      (std::abs(c1.row - c2.row) == (grid_settings.widths[c1.row] - 2)) ||
      (std::abs(c1.col - c2.col) == 2)
      // ((grid_settings.widths[c1.row] + c1.col - c2.col) %
      // grid_settings.widths[c1.row]) == 2 ||
      // ((grid_settings.height + c1.row - c2.row) % grid_settings.height) ==
      // 2
  ) {
    // Weave!
    CellCoordinate between = {(c1.row + c2.row) / 2, (c1.col + c2.col) / 2};
    // fmt::print("Setting undercell for {}->{}->{}\n", c1, between, c2);
    auto m = as_mdspan();
    m(between.row, between.col).set_flag(Cell::Flags::UnderCell);  // FIXME
    // Link TO the middle, but not back out?
    link_(c1, between, true);
    link_(c2, between, true);
  } else {
    link_(c1, c2, true);
    link_(c2, c1, true);
  }
}
void Grid::link_(CellCoordinate c1, CellCoordinate c2, bool link_or_unlink) {
  auto neighbors = get_all_neighbors_(c1);
  auto n = ranges::find_if(neighbors, [c2](auto n) { return n && *n == c2; });
  if (n == neighbors.end()) {
    // throw std::runtime_error(fmt::format("Bad neighbor: {} -> {}", c1,
    // c2));
  }
  if (link_or_unlink) {
    grid_view_(c1.row, c1.col).walls &=
        ~(1 << ranges::distance(neighbors.begin(), n));
  } else {
    grid_view_(c1.row, c1.col).walls |=
        (1 << ranges::distance(neighbors.begin(), n));
  }
}

bool Grid::is_under_cell(CellCoordinate c) const {
  auto m = as_mdspan();
  return m(c.row, c.col).check_flag(Cell::Flags::UnderCell);
}

// Reset grid to all solid walls
void Grid::reset() { ranges::fill(grid_, a_closed_cell); }
void Grid::reset_open() { ranges::fill(grid_, a_open_cell); }

// mask helpers
auto Grid::mask_as_mdspan() const {
  // fixme
  return mask_as_mdspan_;
}
bool Grid::masked_at(CellCoordinate c) const {
  auto m = mask_as_mdspan();
  return grid_settings.mask.valid_mask && 0 != m(c.row, c.col);
}

CellCoordinate Grid::random_cell() {
  std::uniform_int_distribution<> d_w(0, grid_settings.widths.back() - 1);
  std::uniform_int_distribution<> d_h(0, grid_settings.height - 1);
  CellCoordinate c;
  do {
    c = CellCoordinate{d_h(gen), d_w(gen)};
  } while (masked_at(c));
  return c;
}

std::vector<CellCoordinate> Grid::get_all_neighbors(CellCoordinate c) const {
  auto n = get_all_neighbors_(c);
  if (grid_settings.enable_weaving) {
    auto n = get_all_neighbors_(c);
    auto rv = n | ranges::views::filter([](auto o) { return bool(o); }) |
              ranges::views::transform([](auto o) { return *o; }) |
              ranges::to<std::vector>;
    if (can_tunnel_north(c)) {
      rv.push_back(*cell_north(*cell_north(c)));
    }
    if (can_tunnel_south(c)) {
      rv.push_back(*cell_south(*cell_south(c)));
    }
    if (can_tunnel_east(c)) {
      rv.push_back(*cell_east(*cell_east(c)));
    }
    if (can_tunnel_west(c)) {
      rv.push_back(*cell_west(*cell_west(c)));
    }
    return rv;
  }
  return n | ranges::views::filter([](auto o) { return bool(o); }) |
         ranges::views::transform([](auto o) { return *o; }) |
         ranges::to<std::vector>;
}

std::vector<CellCoordinate> Grid::get_connected_neighbors(
    CellCoordinate c) const {
  if (grid_settings.enable_weaving) {
    auto n = get_connected_neighbors_(c);
    auto rv = n | ranges::views::filter([](auto o) { return bool(o); }) |
              ranges::views::transform([](auto o) { return *o; }) |
              ranges::views::transform([this, c](auto n) {
                if (is_crossing_undercell(c, n)) {
                  // neighbor is an under cell AND we are crossing it, so skip
                  // o and return the cell past it -- TODO: Handle wrapping!
                  auto n2 = CellCoordinate{n.row + (n.row - c.row),
                                           n.col + (n.col - c.col)};
                  // fmt::print(
                  //     "Crossing an undercell, replacing {} neighbor {} with
                  //     {}\n", c, n, n2);
                  return n2;
                }
                return n;
              }) |
              ranges::to<std::vector>;
    return rv;
  }
  auto n = get_connected_neighbors_(c);
  return n | ranges::views::filter([](auto o) { return bool(o); }) |
         ranges::views::transform([](auto o) { return *o; }) |
         ranges::to<std::vector>;
}

std::vector<CellCoordinate> Grid::get_unconnected_neighbors(CellCoordinate c) {
  auto n = get_unconnected_neighbors_(c);
  return n | ranges::views::filter([](auto o) { return bool(o); }) |
         ranges::views::transform([](auto o) { return *o; }) |
         ranges::to<std::vector>;
}

// Helper for getting random neighbor
std::optional<CellCoordinate> Grid::random_neighbor(CellCoordinate c) {
  auto n = get_all_neighbors_(c);
  return jt_range_front(n |
                        ranges::views::filter([](auto o) { return bool(o); }) |
                        ranges::views::transform([](auto o) { return *o; }) |
                        ranges::views::sample(1, gen));
}

// Helper for getting a random neighbor that is closed (no connections)
std::optional<CellCoordinate> Grid::random_closed_neighbor(CellCoordinate c) {
  if (grid_settings.enable_weaving) {
    auto neighbors = get_all_neighbors(c);
    return jt_range_front(neighbors | ranges::views::filter([this](auto n) {
                            return is_closed_cell(n);
                          }) |
                          ranges::views::sample(1, gen));
  }
  auto neighbors = get_all_neighbors_(c);
  return jt_range_front(neighbors | ranges::views::filter([this](auto n) {
                          return n && is_closed_cell(*n);
                        }) |
                        ranges::views::transform([](auto o) { return *o; }) |
                        ranges::views::sample(1, gen));
}

};  // namespace jt::maze
