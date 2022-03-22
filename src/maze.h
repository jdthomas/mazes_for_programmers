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

std::vector<int> calculate_variable_widths(int height);

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

struct CellCoordinate {
  int row, col;
};

bool operator<(const CellCoordinate &a, const CellCoordinate &b);
bool operator==(const CellCoordinate &a, const CellCoordinate &b);
struct cell_coordinate_hash_fn {
  std::size_t operator()(const CellCoordinate &cell) const {
    std::size_t h1 = std::hash<int>()(cell.row);
    std::size_t h2 = std::hash<int>()(cell.col);

    return h1 ^ h2;
  }
};

struct GridMask {
  bool valid_mask = false;
  std::vector<uint8_t> mask;
};
struct GridSettings {
  int height = 4;
  int width = 4;
  bool allow_ew_wrap = false;
  bool enable_weaving = true;
  std::vector<int> widths;
  GridMask mask;
  int random_seed = 0;  // 0 for random_device feed
};

enum class CellShape { Square, Hex, Triange, SquareVarWidth };

class Grid {
 public:
  GridSettings grid_settings;
  std::mt19937 gen;

  Grid(int width, int height)
      : Grid(GridSettings{
            .width = width,
            .height = height,
            .mask = GridMask{.mask = std::vector<uint8_t>(width * height)},
        }){};

  Grid(GridSettings settings)
      : grid_settings{settings},
        gen{grid_settings.random_seed == 0 ? rd() : grid_settings.random_seed},
        grid_{static_cast<size_t>(grid_settings.width * grid_settings.height),
              a_closed_cell},
        grid_view_{grid_.data(), grid_settings.height, grid_settings.width},
        mask{settings.mask.mask},
        mask_as_mdspan_{mask.data(), grid_settings.height,
                        grid_settings.width} {
    // FIXME: Fill widths with widths
    if (grid_settings.widths.size() == 0) {
      grid_settings.widths =
          std::vector<int>(grid_settings.height, grid_settings.width);
    }
    assert(grid_settings.width == grid_settings.widths.back());
  };

  struct Cell {
    union {
      uint8_t walls;
      struct {
        uint8_t n : 1;
        uint8_t ne : 1;
        uint8_t e : 1;
        uint8_t se : 1;
        uint8_t s : 1;
        uint8_t sw : 1;
        uint8_t w : 1;
        uint8_t nw : 1;
      };
    };
    uint8_t flags = 0;
    enum class Flags : uint8_t {
      UnderCell = 1,
    };
    void set_flag(Flags f) { flags |= to_underlying(f); }
    bool check_flag(Flags f) const { return 0 != (to_underlying(f) & flags); }
  };

 private:
  static std::random_device rd;

  using AdjacentCells = std::array<std::optional<CellCoordinate>, 8>;

  enum class Direction { N, NE, E, SE, S, SW, W, NW };

  static constexpr CellShape cs = CellShape::Square;
  static constexpr bool cell_shape_is_square() {
    return cs == CellShape::SquareVarWidth || cs == CellShape::Square;
  }

  static constexpr Cell a_closed_cell{0xff, 0x00};
  static constexpr Cell a_open_cell{0x00, 0x00};

  std::vector<Cell> grid_;
  stdex::mdspan<Cell,
                stdex::extents<stdex::dynamic_extent, stdex::dynamic_extent>>
      grid_view_;

  std::vector<uint8_t> mask{};
  stdex::mdspan<uint8_t,
                stdex::extents<stdex::dynamic_extent, stdex::dynamic_extent>>
      mask_as_mdspan_;

 private:
  AdjacentCells adjacent_cells(CellCoordinate c) const {
    static_assert(to_underlying(Direction::N) == 0);
    static_assert(to_underlying(Direction::E) == 2);
    static_assert(to_underlying(Direction::S) == 4);
    static_assert(to_underlying(Direction::W) == 6);
    const auto h = grid_settings.height;
    const auto w = grid_settings.widths[c.row];
    // Square
    if constexpr (cs == CellShape::Square) {
      return {
          CellCoordinate{(c.row - 1 + h) % h, (c.col + 0 + w) % w},  // N
          std::nullopt,                                              // NE
          CellCoordinate{(c.row + 0 + h) % h, (c.col + 1 + w) % w},  // E
          std::nullopt,                                              // SE
          CellCoordinate{(c.row + 1 + h) % h, (c.col - 0 + w) % w},  // S
          std::nullopt,                                              // SW
          CellCoordinate{(c.row - 0 + h) % h, (c.col - 1 + w) % w},  // W
          std::nullopt,                                              // NW
      };
    }
    // Hex
    if constexpr (CellShape::Hex == cs) {
      const bool even_col = c.col % 2 == 0;
      const ssize_t diag_north = even_col ? -1 : 0;
      const ssize_t diag_south = even_col ? 0 : 1;
      const auto wrap_h = [&](int row) { return (row + h) % h; };
      const auto wrap_w = [&](int col) { return (col + w) % w; };

      return {
          CellCoordinate{wrap_h(c.row - 1), c.col},                       // N
          CellCoordinate{wrap_h(c.row + diag_north), wrap_w(c.col + 1)},  // NE
          std::nullopt,                                                   // E
          CellCoordinate{wrap_h(c.row + diag_south), wrap_w(c.col + 1)},  // SE
          CellCoordinate{wrap_h(c.row + 1), c.col},                       // S
          CellCoordinate{wrap_h(c.row + diag_south), wrap_w(c.col - 1)},  // SW
          std::nullopt,                                                   // W
          CellCoordinate{wrap_h(c.row + diag_north), wrap_w(c.col - 1)},  // NW
      };
    }

#if 0
    if constexpr (CellShape::Triangle == cs) {
      const bool even_row = c.row % 2 == 0;
      return {
          !even_row
              ? std::nullopt
              : CellCoordinate{(c.row - 1 + h) % h, (c.col + w) % w},      // N
          !even_row
              ? std::nullopt
              : CellCoordinate{(c.row - 1 + h) % h, (c.col + 1 + w) % w},  // NE
          std::nullopt,                                                    // E
          even_row ? std::nullopt
              : CellCoordinate{(c.row + 1 + h) % h, (c.col + 1 + w) % w},  // SE
          even_row ? std::nullopt
                   : CellCoordinate{(c.row + 1 + h) % h, (c.col + w) % w}, // S
          even_row ? std::nullopt
              : CellCoordinate{(c.row + 1 + h) % h, (c.col - 1 + w) % w},  // SW
          std::nullopt,                                                    // W
          !even_row ? std::nullopt
              : CellCoordinate{(c.row - 1 + h) % h, (c.col - 1 + w) % w},  // NW
      };
    }
#endif
    if constexpr (CellShape::SquareVarWidth == cs) {
      auto wrapped_cell = [&](const auto &cell, int row_delta, int col_delta) {
        auto row = (cell.row + row_delta + grid_settings.height) %
                   grid_settings.height;
        auto w = grid_settings.widths[row];
        auto ratio = double(grid_settings.widths[row]) /
                     double(grid_settings.widths[cell.row]);
        auto col = (int(std::round(ratio * cell.col)) + col_delta + w) % w;
        if (ratio != 1.0)
          fmt::print("cell={} -> [{},{}], r={} c={}, ratio={} ({}/{})\n", cell,
                     row, col, row_delta, col_delta, ratio,
                     double(grid_settings.widths[row]),
                     double(grid_settings.widths[cell.row]));
        return CellCoordinate{row, col};
      };
      return {
          wrapped_cell(c, -1, 0),  // N
          std::nullopt,            // NE
          wrapped_cell(c, 0, +1),  // E
          std::nullopt,            // SE
          wrapped_cell(c, +1, 0),  // S
          std::nullopt,            // SW
          wrapped_cell(c, 0, -1),  // W
          std::nullopt,            // NW
      };
    }
  };

 public:
  // Helper to generate all CellCoordinate for each cell
  auto positions() const {
    return ranges::views::cartesian_product(
               ranges::views::iota(0, grid_settings.height),
               ranges::views::iota(0, grid_settings.widths.back())) |
           ranges::views::transform([](const auto &p) {
             const auto &[row, col] = p;
             return CellCoordinate{row, col};
           });
  }

  // Helpers to get neighboring CellCoordinate for each direction
  std::optional<CellCoordinate> cell_north(CellCoordinate c) const;
  std::optional<CellCoordinate> cell_east(CellCoordinate c) const;
  std::optional<CellCoordinate> cell_south(CellCoordinate c) const;
  std::optional<CellCoordinate> cell_west(CellCoordinate c) const;

  std::optional<CellCoordinate> cell_north_west(CellCoordinate c) const;
  std::optional<CellCoordinate> cell_north_east(CellCoordinate c) const;
  std::optional<CellCoordinate> cell_south_west(CellCoordinate c) const;
  std::optional<CellCoordinate> cell_south_east(CellCoordinate c) const;

  bool is_crossing_undercell(CellCoordinate over, CellCoordinate under) const;

  bool is_connected_directly_north(CellCoordinate c) const;
  bool is_connected_directly_west(CellCoordinate c) const;
  bool is_connected_directly_south(CellCoordinate c) const;
  bool is_connected_directly_east(CellCoordinate c) const;
  //
  std::optional<CellCoordinate> connected_cell_north(CellCoordinate c) const;
  std::optional<CellCoordinate> connected_cell_east(CellCoordinate c) const;
  std::optional<CellCoordinate> connected_cell_south(CellCoordinate c) const;
  std::optional<CellCoordinate> connected_cell_west(CellCoordinate c) const;

  std::optional<CellCoordinate> connected_cell_north_west(
      CellCoordinate c) const;
  std::optional<CellCoordinate> connected_cell_south_west(
      CellCoordinate c) const;
  std::optional<CellCoordinate> connected_cell_north_east(
      CellCoordinate c) const;
  std::optional<CellCoordinate> connected_cell_south_east(
      CellCoordinate c) const;

  // Helpers for getting all neighbors
  AdjacentCells get_all_neighbors_(CellCoordinate c) const;

  // Helper for getting all not-connected neighbors
  AdjacentCells get_unconnected_neighbors_(CellCoordinate c);

  // Helpers for getting all connected neighbors
  AdjacentCells get_connected_neighbors_(CellCoordinate c) const;

  size_t count_connected_neighbors(const CellCoordinate c) const;

  // Helper to check if a cell has no connections
  bool is_closed_cell(CellCoordinate c) const;

  // Helper to check if a cell is a dead end (has only one connecttion)
  bool is_dead_end_cell(CellCoordinate c) const;

  bool is_h_passage_cell(CellCoordinate c) const;
  bool is_v_passage_cell(CellCoordinate c) const;

  bool can_tunnel_north(CellCoordinate c) const;
  bool can_tunnel_south(CellCoordinate c) const;
  bool can_tunnel_west(CellCoordinate c) const;
  bool can_tunnel_east(CellCoordinate c) const;

  ////////////////////////////////////////////////////////////////////////////////

  auto as_mdspan() const { return grid_view_; }

  bool is_linked(CellCoordinate c1, CellCoordinate c2) const;

  void unlink(CellCoordinate c1, CellCoordinate c2);
  // Helper for linking two cells togetherr (must be neighbors)
  void link(CellCoordinate c1, CellCoordinate c2);
  void link_(CellCoordinate c1, CellCoordinate c2, bool link_or_unlink);

  bool is_under_cell(CellCoordinate c) const;

  auto &operator()(CellCoordinate cell) const {
    const auto &[row, col] = cell;
    auto m = as_mdspan();
    return m(row, col);
  }

  // Reset grid to all solid walls
  void reset();
  void reset_open();

  // mask helpers
  auto mask_as_mdspan() const;
  bool masked_at(CellCoordinate c) const;

  // Generate a random CellCoordinate within the grid
  CellCoordinate random_cell();

  std::vector<CellCoordinate> get_all_neighbors(CellCoordinate c) const;

  std::vector<CellCoordinate> get_connected_neighbors(CellCoordinate c) const;

  std::vector<CellCoordinate> get_unconnected_neighbors(CellCoordinate c);

  // Helper for getting random neighbor
  std::optional<CellCoordinate> random_neighbor(CellCoordinate c);

  // Helper for getting a random neighbor that is closed (no connections)
  std::optional<CellCoordinate> random_closed_neighbor(CellCoordinate c);
};

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

// TODO: Restore these to work on the generic operations
template <typename FormatContext>
auto fmt::formatter<jt::maze::Grid>::format(jt::maze::Grid const &grid,
                                            FormatContext &ctx) {
  if (grid.grid_settings.widths.back() > 200 / 4) {
    return fmt::format_to(ctx.out(), "Maze too big for console!");
  }
  // Draw an initial boundry
  for (int col = 0; col < grid.grid_settings.widths.back(); col++) {
    auto c = '-';
    fmt::format_to(ctx.out(), "{}{}{}+", c, c, c);
  }
  fmt::format_to(ctx.out(), "\n");

  for (int row = 0; row < grid.grid_settings.height; row++) {
    // Draw verticals
    for (int col = 0; col < grid.grid_settings.widths.back(); col++) {
      char c = grid(jt::maze::CellCoordinate{row, col}).e ? '|' : ' ';
      // v_wall_to_char(grid(jt::maze::CellCoordinate{row, col}).right)
      fmt::format_to(ctx.out(), "   {}", c);
    }
    fmt::format_to(ctx.out(), "\n");

    // Draw horizantals
    for (int col = 0; col < grid.grid_settings.widths.back(); col++) {
      // auto c = h_wall_to_char(grid(jt::maze::CellCoordinate{row, col}).down);
      char c = grid(jt::maze::CellCoordinate{row, col}).s ? '-' : ' ';
      fmt::format_to(ctx.out(), "{}{}{}+", c, c, c);
    }
    fmt::format_to(ctx.out(), "\n");
  }

  return fmt::format_to(ctx.out(), "... {} x {}\n",
                        grid.grid_settings.widths.back(),
                        grid.grid_settings.height);
}
