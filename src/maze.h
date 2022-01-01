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
#include <optional>
#include <random>
#include <range/v3/all.hpp>
#include <set>
#include <type_traits>
#include <unordered_map>
#include <vector>

namespace jt::maze {
namespace stdex = std::experimental;

std::vector<size_t> calculate_variable_widths(size_t height);

template <typename E>
constexpr auto to_underlying(E e) noexcept {
  return static_cast<std::underlying_type_t<E>>(e);
}

// Not sure why ranges::front() is not working for my sampled ranges, but
// this seems to do the trick.
template <typename R>
auto jt_range_front(R &&rng) {
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

struct GridMask {
  size_t width, height;
  std::vector<uint8_t> mask;
};

enum class CellShape { Square, Hex, Triange };

struct GridReprCommon {
  GridReprCommon(size_t width, size_t height)
      : widths_(height, width), height_{height} {};

  size_t height_;
  std::vector<size_t> widths_;
  static constexpr CellShape cs = CellShape::Square;

  bool allow_ew_wrap = true;

  using AdjacentCells = std::array<std::optional<CellCoordinate>, 8>;
  enum class Direction { N, NE, E, SE, S, SW, W, NW };
  AdjacentCells adjacent_cells(CellCoordinate c) const {
    static_assert(to_underlying(Direction::N) == 0);
    static_assert(to_underlying(Direction::E) == 2);
    static_assert(to_underlying(Direction::S) == 4);
    static_assert(to_underlying(Direction::W) == 6);
    const auto h = height_;
    const auto w = widths_[c.row];
    // Square
    if constexpr (CellShape::Square == cs) {
      return {
          CellCoordinate{(c.row - 1 + h) % h, (c.col + 0 + w) % w},  // North
          std::nullopt,  // NorthEast
          CellCoordinate{(c.row + 0 + h) % h, (c.col + 1 + w) % w},  // East
          std::nullopt,  // SouthEast
          CellCoordinate{(c.row + 1 + h) % h, (c.col - 0 + w) % w},  // South
          std::nullopt,  // SouthWest
          CellCoordinate{(c.row - 0 + h) % h, (c.col - 1 + w) % w},  // West
          std::nullopt,  // NorthWest
      };
    }
    // Hex
    if constexpr (CellShape::Hex == cs) {
      const ssize_t dn = c.col % 2 ? 1 : 0;
      const ssize_t ds = c.col % 2 ? 0 : 1;

      return {
          CellCoordinate{(c.row - 1 + h) % h, (c.col + 0 + w) % w},  // North
          CellCoordinate{(c.row - dn + h) % h,
                         (c.col - 1 + w) % w},  // NorthEast
          std::nullopt,                         // East
          CellCoordinate{(c.row + ds + h) % h,
                         (c.col + 1 + w) % w},  // SouthEast
          CellCoordinate{(c.row + 1 + h) % h, (c.col - 0 + w) % w},  // South
          CellCoordinate{(c.row + ds + h) % h,
                         (c.col + 1 + w) % w},  // SouthWest
          std::nullopt,                         // West
          CellCoordinate{(c.row - dn + h) % h,
                         (c.col - 1 + w) % w},  // NorthWest
      };
    }
    // Variable width?
    // TODO: From book on polar grid
    // if row > 0
    //   cell.cw = self[row, col + 1]
    //   cell.ccw = self[row, col - 1]
    //   ratio = @grid[row].length / @grid[row - 1].length
    //   parent = @grid[row - 1][col / ratio]
    //   parent.outward << cell
    //   cell.inward = parent
    // end
  };

  // Helpers to get neighboring CellCoordinate for each direction
  auto cell_north(CellCoordinate c) const {
    return get_all_neighbors_(c)[to_underlying(Direction::N)];
  }
  auto cell_east(CellCoordinate c) const {
    return get_all_neighbors_(c)[to_underlying(Direction::E)];
  }
  auto cell_south(CellCoordinate c) const {
    return get_all_neighbors_(c)[to_underlying(Direction::S)];
  }
  auto cell_west(CellCoordinate c) const {
    return get_all_neighbors_(c)[to_underlying(Direction::W)];
  }

  auto cell_north_west(CellCoordinate c) const {
    if constexpr (cs == CellShape::Square) {
      const bool even_col = c.col % 2 == 0;
      return even_col
                 ? std::nullopt
                 : get_all_neighbors_(c)[to_underlying(Direction::W)];
    } else {
      return get_all_neighbors_(c)[to_underlying(Direction::NW)];
    }
  }
  auto cell_north_east(CellCoordinate c) const {
    if constexpr (cs == CellShape::Square) {
      const bool even_col = c.col % 2 == 0;
      return even_col
                 ? std::nullopt
                 : get_all_neighbors_(c)[to_underlying(Direction::E)];
    } else {
      return get_all_neighbors_(c)[to_underlying(Direction::NE)];
    }
  }
  auto cell_south_west(CellCoordinate c) const {
    if constexpr (cs == CellShape::Square) {
      const bool even_col = c.col % 2 == 0;
      return !even_col
                 ? std::nullopt
                 : get_all_neighbors_(c)[to_underlying(Direction::W)];
    } else {
      return get_all_neighbors_(c)[to_underlying(Direction::SW)];
    }
  }
  auto cell_south_east(CellCoordinate c) const {
    if constexpr (cs == CellShape::Square) {
      const bool even_col = c.col % 2 == 0;
      return !even_col
                 ? std::nullopt
                 : get_all_neighbors_(c)[to_underlying(Direction::W)];
    } else {
      return get_all_neighbors_(c)[to_underlying(Direction::SE)];
    }
  }

  //
  auto connected_cell_north(CellCoordinate c) const {
    auto n = cell_north(c);
    return n && is_linked(c, *n) ? n : std::nullopt;
  }
  auto connected_cell_east(CellCoordinate c) const {
    auto n = cell_east(c);
    return n && is_linked(c, *n) ? n : std::nullopt;
  }
  auto connected_cell_south(CellCoordinate c) const {
    auto n = cell_south(c);
    return n && is_linked(c, *n) ? n : std::nullopt;
  }
  auto connected_cell_west(CellCoordinate c) const {
    auto n = cell_west(c);
    return n && is_linked(c, *n) ? n : std::nullopt;
  }

  auto connected_cell_north_west(CellCoordinate c) const {
    auto n = cell_north_west(c);
    return n && is_linked(c, *n) ? n : std::nullopt;
  }
  auto connected_cell_south_west(CellCoordinate c) const {
    auto n = cell_south_west(c);
    return n && is_linked(c, *n) ? n : std::nullopt;
  }
  auto connected_cell_north_east(CellCoordinate c) const {
    auto n = cell_north_east(c);
    return n && is_linked(c, *n) ? n : std::nullopt;
  }
  auto connected_cell_south_east(CellCoordinate c) const {
    auto n = cell_south_east(c);
    return n && is_linked(c, *n) ? n : std::nullopt;
  }

  // Helpers for getting all neighbors
  AdjacentCells get_all_neighbors_(CellCoordinate c) const {
    auto neighbors = adjacent_cells(c);
    // Fixup for wrapping (never wrap N/S, optionally wrap E/W)
    neighbors |= ranges::actions::transform(
        [this, c](auto n) -> std::optional<CellCoordinate> {
          return !n || (std::abs(ssize_t(c.row - n->row)) > 1) ||
                         (std::abs(ssize_t(c.col - n->col)) > 1 &&
                          !allow_ew_wrap)
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
  AdjacentCells get_unconnected_neighbors_(CellCoordinate c) {
    auto neighbors = get_all_neighbors_(c);
    neighbors |= ranges::actions::transform([this, &c](const auto &c2) {
      return c2 && !is_linked(c, *c2) ? c2 : std::nullopt;
    });
    return neighbors;
  }

  // Helpers for getting all connected neighbors
  AdjacentCells get_connected_neighbors_(CellCoordinate c) const {
    auto neighbors = adjacent_cells(c);
    neighbors |= ranges::actions::transform(
        [this, c](auto n) { return n && is_linked(c, *n) ? n : std::nullopt; });
    return neighbors;
  }

  size_t count_connected_neighbors(const CellCoordinate c) const {
    auto tmp = get_connected_neighbors_(c);
    return ranges::distance(
        tmp | ranges::views::filter([](auto n) { return bool(n); }));
  };

  // Helper to check if a cell has no connections
  bool is_closed_cell(CellCoordinate c) const {
    return count_connected_neighbors(c) == 0;
  }

  // Helper to check if a cell is a dead end (has only one connecttion)
  bool is_dead_end_cell(CellCoordinate c) const {
    return count_connected_neighbors(c) == 1;
  }

  // these depend on the grid reprensation
  virtual bool is_linked(CellCoordinate c1, CellCoordinate c2) const = 0;
  virtual void link(CellCoordinate c1, CellCoordinate c2) = 0;
};

struct SparceGridRepr : GridReprCommon {
  struct Cell {
    std::vector<CellCoordinate> linked_neighbors;
  };

  std::vector<Cell> grid_;
  stdex::mdspan<Cell,
                stdex::extents<stdex::dynamic_extent, stdex::dynamic_extent>>
      grid_view_;
};

struct DenseGridRepr : GridReprCommon {
  DenseGridRepr(size_t width, size_t height)
      : GridReprCommon(width, height),
        grid_{width * height},
        grid_view_{grid_.data(), height_, widths_.back()} {};
  struct Cell {
    Wall down, right;
  };

  std::vector<Cell> grid_;
  stdex::mdspan<Cell,
                stdex::extents<stdex::dynamic_extent, stdex::dynamic_extent>>
      grid_view_;

  auto as_mdspan() const { return grid_view_; }

  bool is_linked(CellCoordinate c1, CellCoordinate c2) const override {
    auto &[row_1, col_1] = c1;
    auto &[row_2, col_2] = c2;
    ssize_t dx = col_2 - col_1;
    ssize_t dy = row_2 - row_1;
    auto m = as_mdspan();
    if (dx == -1 || dx == widths_[row_1] - 1) {
      return m(row_1, (widths_[row_1] + col_1 - 1) % widths_[row_1]).right ==
             Wall::Open;
    } else if (dx == 1 || dx == -widths_[row_1] + 1) {
      return m(row_1, col_1).right == Wall::Open;
    } else if (dy == -1) {
      return m(row_1 - 1, col_1).down == Wall::Open;
    } else if (dy == 1) {
      return m(row_1, col_1).down == Wall::Open;
    }
    return false;
  }

  // Helper for linking two cells togetherr (must be neighbors)
  void link(CellCoordinate c1, CellCoordinate c2) override {
    auto &[row_1, col_1] = c1;
    auto &[row_2, col_2] = c2;
    ssize_t dx = col_2 - col_1;
    ssize_t dy = row_2 - row_1;
    // fmt::print("linking: {} to {} delta: {}.{}\n", c1, c2, dx, dy);
    // if (std::abs(dx) > 1 || std::abs(dy) > 1 ||
    //     (std::abs(dx) > 0 && std::abs(dy) > 0))
    //   throw std::runtime_error("Linking cells must be neighbors");

    auto m = as_mdspan();
    if (dx == -1 || dx == widths_[row_1] - 1) {
      m(row_1, (widths_[row_1] + col_1 - 1) % widths_[row_1]).right =
          Wall::Open;
      // fmt::print("Setting right of {} {}\n", col_1 - 1, row_1);
    } else if (dx == 1 || dx == -widths_[row_1] + 1) {
      m(row_1, col_1).right = Wall::Open;
      // fmt::print("Setting right of {} {}\n", col_1, row_1);
    } else if (dy == -1) {
      m(row_1 - 1, col_1).down = Wall::Open;
      // fmt::print("Setting down of {} {}\n", col_1, row_1 - 1);
    } else if (dy == 1) {
      m(row_1, col_1).down = Wall::Open;
      // fmt::print("Setting down of {} {}\n", col_1, row_1);
    } else {
      throw std::runtime_error(
          fmt::format("Bad neighbor: {} -> {}, dx={}", c1, c2, dx));
    }
  }

  // Helper to generate all CellCoordinate for each cell
  auto positions() const {
    return ranges::views::cartesian_product(
               ranges::views::iota(static_cast<size_t>(0),
                                   static_cast<size_t>(height_)),
               ranges::views::iota(static_cast<size_t>(0),
                                   static_cast<size_t>(widths_.back()))) |
           ranges::views::transform([](const auto &p) {
             const auto &[row, col] = p;
             return CellCoordinate{row, col};
           });
  }

  auto &operator()(CellCoordinate cell) const {
    const auto &[row, col] = cell;
    auto m = as_mdspan();
    return m(row, col);
  }

  // Reset grid to all solid walls
  void reset() {
    for (auto &c : grid_) {
      c.down = Wall::Solid;
      c.right = Wall::Solid;
    }
  }
};

struct Grid : DenseGridRepr {
  Grid(size_t width, size_t height);
  Grid(GridMask msk) : Grid(msk.width, msk.height) { mask = msk.mask; };

 private:
  std::random_device rd{};

  std::vector<uint8_t> mask{};
  stdex::mdspan<uint8_t,
                stdex::extents<stdex::dynamic_extent, stdex::dynamic_extent>>
      mask_as_mdspan_;

 public:
  std::mt19937 gen;

  // mask helpers
  auto mask_as_mdspan() const {
    // fixme
    return mask_as_mdspan_;
  }
  bool masked_at(CellCoordinate c) const {
    auto m = mask_as_mdspan();
    return 0 != m(c.row, c.col);
  }

  // Helpers to get individual neighbors
  // auto connected_cell_north(CellCoordinate c) const {
  //   auto m = as_mdspan();
  //   auto n = cell_north(c);
  //   return n && m(n->row, n->col).down == Wall::Open ? n : std::nullopt;
  // }
  // auto connected_cell_east(CellCoordinate c) const {
  //   auto m = as_mdspan();
  //   auto e = cell_east(c);
  //   return e && m(c.row, c.col).right == Wall::Open ? e : std::nullopt;
  // }
  // auto connected_cell_south(CellCoordinate c) const {
  //   auto m = as_mdspan();
  //   auto s = cell_south(c);
  //   return s && m(c.row, c.col).down == Wall::Open ? s : std::nullopt;
  // }
  // auto connected_cell_west(CellCoordinate c) const {
  //   auto m = as_mdspan();
  //   auto w = cell_west(c);
  //   return w && m(w->row, w->col).right == Wall::Open ? w : std::nullopt;
  // }

  // Generate a random CellCoordinate within the grid
  auto random_cell() {
    std::uniform_int_distribution<> d_w(0, widths_.back() - 1);
    std::uniform_int_distribution<> d_h(0, height_ - 1);
    CellCoordinate c;
    do {
      c = CellCoordinate{static_cast<size_t>(d_h(gen)),
                         static_cast<size_t>(d_w(gen))};
    } while (masked_at(c));
    return c;
  }

  std::vector<CellCoordinate> get_all_neighbors(CellCoordinate c) const {
    auto n = get_all_neighbors_(c);
    return n | ranges::views::filter([](auto o) { return bool(o); }) |
           ranges::views::transform([](auto o) { return *o; }) |
           ranges::to<std::vector>;
  }

  std::vector<CellCoordinate> get_connected_neighbors(CellCoordinate c) const {
    auto n = get_connected_neighbors_(c);
    return n | ranges::views::filter([](auto o) { return bool(o); }) |
           ranges::views::transform([](auto o) { return *o; }) |
           ranges::to<std::vector>;
  }

  std::vector<CellCoordinate> get_unconnected_neighbors(CellCoordinate c) {
    auto n = get_unconnected_neighbors_(c);
    return n | ranges::views::filter([](auto o) { return bool(o); }) |
           ranges::views::transform([](auto o) { return *o; }) |
           ranges::to<std::vector>;
  }

  // Helper for getting random neighbor
  std::optional<CellCoordinate> random_neighbor(CellCoordinate c) {
    auto n = get_all_neighbors_(c);
    return jt_range_front(
        n | ranges::views::filter([](auto o) { return bool(o); }) |
        ranges::views::transform([](auto o) { return *o; }) |
        ranges::views::sample(1, gen));
  }

  // Helper for getting a random neighbor that is closed (no connections)
  std::optional<CellCoordinate> random_closed_neighbor(CellCoordinate c) {
    auto neighbors = get_all_neighbors_(c);
    return jt_range_front(neighbors | ranges::views::filter([this](auto n) {
                            return n && is_closed_cell(*n);
                          }) |
                          ranges::views::transform([](auto o) { return *o; }) |
                          ranges::views::sample(1, gen));
  }
};

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

void braid_maze(Grid &grid, float p = 1.0);

class GeneratorRegistry {
 public:
  struct RegistryConfig {
    // RegistryConfig(RegistryConfig const &) = delete;
    // RegistryConfig(RegistryConfig&&) = delete;
    explicit RegistryConfig(char sn, std::string n,
                            std::function<void(Grid &)> f)
        : short_name(sn), name(std::move(n)), generate(f) {}
    char short_name;
    std::string name;
    std::function<void(Grid &)> generate;
  };

  // Silly empty struct who's constructor adds us to the global registry
  struct RegisterGenerator {
    RegisterGenerator(char sn, std::string n, std::function<void(Grid &)> f) {
      // Ensure that sn is not already in the registry??
      fmt::print("Registering {}\n", n);
      GeneratorRegistry::registry_.emplace_back(sn, std::move(n), f);
    }
  };

  static const std::vector<RegistryConfig> &AllMethods() { return registry_; }

  static const RegistryConfig &GetMazeGeneratorByShortName(char sn) {
    auto m = std::find_if(registry_.begin(), registry_.end(),
                          [sn](const auto &x) { return x.short_name == sn; });
    if (m == registry_.end()) {
      throw std::runtime_error("Method not found");
    }
    return *m;
  }

  static size_t GetMazeGeneratorIndexByShortName(char sn) {
    auto m = std::find_if(registry_.begin(), registry_.end(),
                          [sn](const auto &x) { return x.short_name == sn; });
    return std::distance(registry_.begin(), m);
  }

  // static RegistryConfig &GetMazeGeneratorByName();
  static const RegistryConfig &GetMazeGeneratorByIndex(size_t idx) {
    return registry_[idx];
  }
  static size_t GetMazeGeneratorCount() { return registry_.size(); };

 private:
  static std::vector<RegistryConfig> registry_;
};

void ensure_registry();
};  // namespace jt::maze

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

template <>
struct fmt::formatter<jt::maze::CellCoordinate> {
  template <typename ParseContext>
  constexpr auto parse(ParseContext &ctx);

  template <typename FormatContext>
  auto format(jt::maze::CellCoordinate const &coord, FormatContext &ctx);
};

template <typename ParseContext>
constexpr auto fmt::formatter<jt::maze::CellCoordinate>::parse(
    ParseContext &ctx) {
  return ctx.begin();
}

template <typename FormatContext>
auto fmt::formatter<jt::maze::CellCoordinate>::format(
    jt::maze::CellCoordinate const &coord, FormatContext &ctx) {
  return fmt::format_to(ctx.out(), "[{},{}]", coord.row, coord.col);
}

template <>
struct fmt::formatter<jt::maze::Grid> {
  template <typename ParseContext>
  constexpr auto parse(ParseContext &ctx);

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
  if (grid.widths_.back() > 200 / 4) {
    return fmt::format_to(ctx.out(), "Maze too big for console!");
  }
  // Draw an initial boundry
  for (int col = 0; col < grid.widths_.back(); col++) {
    auto c = h_wall_to_char(jt::maze::Wall::Boundry);
    fmt::format_to(ctx.out(), "{}{}{}+", c, c, c);
  }
  fmt::format_to(ctx.out(), "\n");

  for (size_t row = 0; row < grid.height_; row++) {
    // Draw verticals
    for (size_t col = 0; col < grid.widths_.back(); col++) {
      fmt::format_to(
          ctx.out(), "   {}",
          v_wall_to_char(grid(jt::maze::CellCoordinate{row, col}).right));
    }
    fmt::format_to(ctx.out(), "\n");

    // Draw horizantals
    for (size_t col = 0; col < grid.widths_.back(); col++) {
      auto c = h_wall_to_char(grid(jt::maze::CellCoordinate{row, col}).down);
      fmt::format_to(ctx.out(), "{}{}{}+", c, c, c);
    }
    fmt::format_to(ctx.out(), "\n");
  }

  return fmt::format_to(ctx.out(), "... {} x {}\n", grid.widths_.back(),
                        grid.height_);
}
