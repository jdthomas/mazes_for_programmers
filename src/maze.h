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
#include <optional>
#include <random>
#include <range/v3/all.hpp>
#include <set>
#include <unordered_map>
#include <vector>

namespace jt::maze {
namespace stdex = std::experimental;

// Not sure why ranges::front() is not working for my sampled ranges, but
// this seems to do the trick.
template <typename R> auto jt_range_front(R &&rng) {
  return (ranges::begin(rng) == ranges::end(rng))
             ? std::nullopt
             : std::make_optional(*ranges::begin(rng));
}

enum class Wall { Solid, Open, Boundry };

struct CellCoordinate {
  size_t row, col;
};

bool operator<(const CellCoordinate &a, const CellCoordinate &b);
bool operator==(const CellCoordinate &a, const CellCoordinate &b);

struct Grid {
  Grid(size_t width, size_t height);
  size_t width_, height_;

private:
  // TODO: Get rid of "Cell" and use extra dimension in mdspan
  struct Cell {
    Wall down, right;
  };
  std::vector<Cell> cells_;
  stdex::mdspan<Cell,
                stdex::extents<stdex::dynamic_extent, stdex::dynamic_extent>>
      mdspan_;

  std::random_device rd;

public:
  std::mt19937 gen;

  auto as_mdspan() const { return mdspan_; }

  auto &operator()(size_t row, size_t col) const
  /*__attribute__((deprecated))*/ {
    auto m = as_mdspan();
    return m(row, col);
  }

  auto &operator()(CellCoordinate cell) const {
    const auto &[row, col] = cell;
    auto m = as_mdspan();
    return m(row, col);
  }

  // Helper to generate all CellCoordinate for each cell
  auto positions() const {
    return ranges::views::cartesian_product(
               ranges::views::iota(static_cast<size_t>(0),
                                   static_cast<size_t>(height_)),
               ranges::views::iota(static_cast<size_t>(0),
                                   static_cast<size_t>(width_))) |
           ranges::views::transform([](const auto &p) {
             const auto &[row, col] = p;
             return CellCoordinate{row, col};
           });
  }

  // Helpers to get neighboring CellCoordinate for each direction
  auto cell_north(CellCoordinate c) const {
    return c.row == 0 ? std::nullopt
                      : std::make_optional(CellCoordinate{c.row - 1, c.col});
  }
  auto cell_east(CellCoordinate c) const {
    return c.col >= (width_ - 1)
               ? std::nullopt
               : std::make_optional(CellCoordinate{c.row, c.col + 1});
  }
  auto cell_south(CellCoordinate c) const {
    return c.row >= (height_ - 1)
               ? std::nullopt
               : std::make_optional(CellCoordinate{c.row + 1, c.col});
  }
  auto cell_west(CellCoordinate c) const {
    return c.col == 0 ? std::nullopt
                      : std::make_optional(CellCoordinate{c.row, c.col - 1});
  }

  // Generate a random CellCoordinate within the grid
  auto random_cell() {
    std::uniform_int_distribution<> d_w(0, width_ - 1);
    std::uniform_int_distribution<> d_h(0, height_ - 1);
    return CellCoordinate{static_cast<size_t>(d_h(gen)),
                          static_cast<size_t>(d_w(gen))};
  }

  // Helpers for getting all connected neighbors
  std::array<std::optional<CellCoordinate>, 4>
  get_connected_neighbors_(CellCoordinate c) const {
    auto n = cell_north(c);
    auto e = cell_east(c);
    auto s = cell_south(c);
    auto w = cell_west(c);
    auto m = as_mdspan();
    return {
        n && m(n->row, n->col).down == Wall::Open ? n : std::nullopt,
        e && m(c.row, c.col).right == Wall::Open ? e : std::nullopt,
        s && m(c.row, c.col).down == Wall::Open ? s : std::nullopt,
        w && m(w->row, w->col).right == Wall::Open ? w : std::nullopt,
    };
  }

  std::vector<CellCoordinate> get_connected_neighbors(CellCoordinate c) const {
    auto n = get_connected_neighbors_(c);
    return n | ranges::views::filter([](auto o) { return bool(o); }) |
           ranges::views::transform([](auto o) { return *o; }) |
           ranges::to<std::vector>;
  }

  // Helpers for getting all neighbors
  std::array<std::optional<CellCoordinate>, 4>
  get_all_neighbors_(CellCoordinate c) const {
    return {cell_north(c), cell_east(c), cell_south(c), cell_west(c)};
  }

  std::vector<CellCoordinate> get_all_neighbors(CellCoordinate c) const {
    auto n = get_all_neighbors_(c);
    return n | ranges::views::filter([](auto o) { return bool(o); }) |
           ranges::views::transform([](auto o) { return *o; }) |
           ranges::to<std::vector>;
  }

  // Helper for getting random neighbor
  std::optional<CellCoordinate> random_neighbor(CellCoordinate c) {
    auto an = get_all_neighbors(c);
    return jt_range_front(an | ranges::views::sample(1, gen));
  }

  // Helper for getting a random neighbor that is closed (no connections)
  std::optional<CellCoordinate> random_closed_neighbor(CellCoordinate c) {
    auto neighbors = get_all_neighbors(c);
    return jt_range_front(neighbors | ranges::views::filter([this](auto n) {
                            return this->is_closed_cell(n);
                          }) |
                          ranges::views::sample(1, gen));
  }

  // Helper for linking two cells togetherr (must be neighbors)
  void link(CellCoordinate c1, CellCoordinate c2) {
    auto &[row_1, col_1] = c1;
    auto &[row_2, col_2] = c2;
    ssize_t dx = col_2 - col_1;
    ssize_t dy = row_2 - row_1;
    // fmt::print("linking: {} to {} delta: {}.{}\n", c1, c2, dx, dy);
    // if (std::abs(dx) > 1 || std::abs(dy) > 1 ||
    //     (std::abs(dx) > 0 && std::abs(dy) > 0))
    //   throw std::runtime_error("Linking cells must be neighbors");

    auto m = as_mdspan();
    if (dx == -1) {
      m(row_1, col_1 - 1).right = Wall::Open;
      // fmt::print("Setting right of {} {}\n", col_1 - 1, row_1);
    } else if (dx == 1) {
      m(row_1, col_1).right = Wall::Open;
      // fmt::print("Setting right of {} {}\n", col_1, row_1);
    } else if (dy == -1) {
      m(row_1 - 1, col_1).down = Wall::Open;
      // fmt::print("Setting down of {} {}\n", col_1, row_1 - 1);
    } else if (dy == 1) {
      m(row_1, col_1).down = Wall::Open;
      // fmt::print("Setting down of {} {}\n", col_1, row_1);
    }
  }

  // Helper to check if a cell has no connections
  bool is_closed_cell(CellCoordinate c) const {
    auto tmp = get_connected_neighbors_(c);
    return ranges::accumulate(
               tmp | ranges::views::transform([](auto o) { return o ? 1 : 0; }),
               0) == 0;
  }

  // Helper to check if a cell is a dead end (has only one connecttion)
  bool is_dead_end_cell(CellCoordinate c) const {
    auto tmp = get_connected_neighbors_(c);
    return ranges::accumulate(
               tmp | ranges::views::transform([](auto o) { return o ? 1 : 0; }),
               0) == 1;
  }

  // Reset grid to all solid walls
  void reset() {
    for (auto &c : cells_) {
      c.down = Wall::Solid;
      c.right = Wall::Solid;
    }
  }
};

// TODO: Some meta structure here to hold the grid + distances + name of method
// + ...

// struct AllTheThings {
//   Grid grid;
//   std::string generator_name;
//   auto disttances;
//   auto longest_path_len;
//   auto longest_path;
//
// };

void binary_tree_maze(Grid &grid);
void binary_tree_maze_p(Grid &grid);
void binary_tree_maze_p2(Grid &grid);
void sidewinder_maze(Grid &grid);
void sidewinder_maze_p(Grid &grid);
void random_walk_Aldous_Broder_maze(Grid &grid);
void random_walk_Wilson_maze(Grid &grid);
void hunt_and_kill_maze(Grid &grid);
void recursive_backtracking_maze(Grid &grid);
std::vector<int> dijkstra_distances(const Grid &grid,
                                    CellCoordinate start_cell);
std::tuple<CellCoordinate, int> furthest_cell(const Grid &grid,
                                              std::vector<int> &distances);
std::vector<CellCoordinate> longest_path_(Grid &grid,
                                          std::vector<int> &distances);
}; // namespace jt::maze

// FIXME
extern std::array<char, 6> all_methods;
extern size_t method;
std::string method_name(char m);
std::vector<int> gen_maze(jt::maze::Grid &grid);

////////////////////////////////////////////////////////////////////////////////
// Ascii output
////////////////////////////////////////////////////////////////////////////////
static char h_wall_to_char(const jt::maze::Wall wall) {
  return wall == jt::maze::Wall::Open    ? ' '
         : wall == jt::maze::Wall::Solid ? '-'
                                         : 'X';
}
static char v_wall_to_char(const jt::maze::Wall wall) {
  return wall == jt::maze::Wall::Open    ? ' '
         : wall == jt::maze::Wall::Solid ? '|'
                                         : 'X';
}

template <> struct fmt::formatter<jt::maze::CellCoordinate> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx);

  template <typename FormatContext>
  auto format(jt::maze::CellCoordinate const &coord, FormatContext &ctx);
};

template <typename ParseContext>
constexpr auto
fmt::formatter<jt::maze::CellCoordinate>::parse(ParseContext &ctx) {
  return ctx.begin();
}

template <typename FormatContext>
auto fmt::formatter<jt::maze::CellCoordinate>::format(
    jt::maze::CellCoordinate const &coord, FormatContext &ctx) {
  return fmt::format_to(ctx.out(), "[{},{}]", coord.row, coord.col);
}

template <> struct fmt::formatter<jt::maze::Grid> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx);

  template <typename FormatContext>
  auto format(jt::maze::Grid const &grid, FormatContext &ctx);
};

template <typename ParseContext>
constexpr auto fmt::formatter<jt::maze::Grid>::parse(ParseContext &ctx) {
  return ctx.begin();
}

template <typename FormatContext>
auto fmt::formatter<jt::maze::Grid>::format(jt::maze::Grid const &grid,
                                            FormatContext &ctx) {
  if (grid.width_ > 200 / 4) {
    return fmt::format_to(ctx.out(), "Maze too big for console!");
  }
  // Draw an initial boundry
  for (int col = 0; col < grid.width_; col++) {
    auto c = h_wall_to_char(jt::maze::Wall::Boundry);
    fmt::format_to(ctx.out(), "{}{}{}+", c, c, c);
  }
  fmt::format_to(ctx.out(), "\n");

  for (int row = 0; row < grid.height_; row++) {
    // Draw verticals
    for (int col = 0; col < grid.width_; col++) {
      fmt::format_to(ctx.out(), "   {}", v_wall_to_char(grid(row, col).right));
    }
    fmt::format_to(ctx.out(), "\n");

    // Draw horizantals
    for (int col = 0; col < grid.width_; col++) {
      auto c = h_wall_to_char(grid(row, col).down);
      fmt::format_to(ctx.out(), "{}{}{}+", c, c, c);
    }
    fmt::format_to(ctx.out(), "\n");
  }

  return fmt::format_to(ctx.out(), "... {} x {}\n", grid.width_, grid.height_);
}
