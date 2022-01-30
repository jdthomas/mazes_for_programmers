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

struct CellCoordinate {
  size_t row, col;
};

bool operator<(const CellCoordinate &a, const CellCoordinate &b);
bool operator==(const CellCoordinate &a, const CellCoordinate &b);
struct cell_coordinate_hash_fn {
  std::size_t operator()(const CellCoordinate &cell) const {
    std::size_t h1 = std::hash<size_t>()(cell.row);
    std::size_t h2 = std::hash<size_t>()(cell.col);

    return h1 ^ h2;
  }
};

struct GridMask {
  size_t width, height;
  std::vector<uint8_t> mask;
};

enum class CellShape { Square, Hex, Triange };

struct GridReprCommon {
  GridReprCommon(size_t width, size_t height)
      : widths_(height, width), height_{height} {};

 protected:
  static constexpr CellShape cs = CellShape::Square;

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
      const auto wrap_h = [&](ssize_t row) { return (row + h) % h; };
      const auto wrap_w = [&](ssize_t col) { return (col + w) % w; };

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

 public:
  bool allow_ew_wrap = false;
  bool enable_weaving = true;
  size_t height_;
  std::vector<size_t> widths_;

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
      return even_col ? std::nullopt
                      : get_all_neighbors_(c)[to_underlying(Direction::W)];
    } else {
      return get_all_neighbors_(c)[to_underlying(Direction::NW)];
    }
  }

  auto cell_north_east(CellCoordinate c) const {
    if constexpr (cs == CellShape::Square) {
      const bool even_col = c.col % 2 == 0;
      return even_col ? std::nullopt
                      : get_all_neighbors_(c)[to_underlying(Direction::E)];
    } else {
      return get_all_neighbors_(c)[to_underlying(Direction::NE)];
    }
  }

  auto cell_south_west(CellCoordinate c) const {
    if constexpr (cs == CellShape::Square) {
      const bool even_col = c.col % 2 == 0;
      return !even_col ? std::nullopt
                       : get_all_neighbors_(c)[to_underlying(Direction::W)];
    } else {
      return get_all_neighbors_(c)[to_underlying(Direction::SW)];
    }
  }

  auto cell_south_east(CellCoordinate c) const {
    if constexpr (cs == CellShape::Square) {
      const bool even_col = c.col % 2 == 0;
      return !even_col ? std::nullopt
                       : get_all_neighbors_(c)[to_underlying(Direction::E)];
    } else {
      return get_all_neighbors_(c)[to_underlying(Direction::SE)];
    }
  }

  bool is_crossing_undercell(CellCoordinate over, CellCoordinate under) const {
    return is_under_cell(under) && is_linked(over, under) &&
           !is_linked(under, over);
  }

  bool is_connected_directly_north(CellCoordinate c) const {
    auto n = cell_north(c);
    return n && is_linked(c, *n);
  }
  bool is_connected_directly_west(CellCoordinate c) const {
    auto n = cell_west(c);
    return n && is_linked(c, *n);
  }
  bool is_connected_directly_south(CellCoordinate c) const {
    auto n = cell_south(c);
    return n && is_linked(c, *n);
  }
  bool is_connected_directly_east(CellCoordinate c) const {
    auto n = cell_east(c);
    return n && is_linked(c, *n);
  }
  //
  auto connected_cell_north(CellCoordinate c) const {
    auto n = cell_north(c);
    if (enable_weaving && n && is_crossing_undercell(c, *n))
      return cell_north(*n);
    return n && is_linked(c, *n) ? n : std::nullopt;
  }
  auto connected_cell_east(CellCoordinate c) const {
    auto n = cell_east(c);
    if (enable_weaving && n && is_crossing_undercell(c, *n))
      return cell_east(*n);
    return n && is_linked(c, *n) ? n : std::nullopt;
  }
  auto connected_cell_south(CellCoordinate c) const {
    auto n = cell_south(c);
    if (enable_weaving && n && is_crossing_undercell(c, *n))
      return cell_south(*n);
    return n && is_linked(c, *n) ? n : std::nullopt;
  }
  auto connected_cell_west(CellCoordinate c) const {
    auto n = cell_west(c);
    if (enable_weaving && n && is_crossing_undercell(c, *n))
      return cell_west(*n);
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
    auto neighbors = get_all_neighbors_(c);
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

  bool is_h_passage_cell(CellCoordinate c) const {
    return !connected_cell_north(c) && !connected_cell_south(c) &&
           connected_cell_east(c) && connected_cell_west(c);
  }
  bool is_v_passage_cell(CellCoordinate c) const {
    return connected_cell_north(c) && connected_cell_south(c) &&
           !connected_cell_east(c) && !connected_cell_west(c);
  }

  bool can_tunnel_north(CellCoordinate c) const {
    auto n = cell_north(c);
    return n && cell_north(*n) && !is_under_cell(*n) && is_h_passage_cell(*n);
  }
  bool can_tunnel_south(CellCoordinate c) const {
    auto n = cell_south(c);
    return n && cell_south(*n) && !is_under_cell(*n) && is_h_passage_cell(*n);
  }
  bool can_tunnel_west(CellCoordinate c) const {
    auto n = cell_west(c);
    return n && cell_west(*n) && !is_under_cell(*n) && is_v_passage_cell(*n) &&
           c.col > 0 && n->col > 0;
  }
  bool can_tunnel_east(CellCoordinate c) const {
    auto n = cell_east(c);
    return n && cell_east(*n) && !is_under_cell(*n) && is_v_passage_cell(*n) &&
           c.col < widths_[c.row] - 1 && n->col < widths_[n->row] - 1;
  }

  // these depend on the grid reprensation
  virtual bool is_linked(CellCoordinate c1, CellCoordinate c2) const = 0;
  virtual void link(CellCoordinate c1, CellCoordinate c2) = 0;
  virtual void unlink(CellCoordinate c1, CellCoordinate c2) = 0;
  virtual bool is_under_cell(CellCoordinate c) const = 0;
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
        grid_{width * height, a_closed_cell},
        grid_view_{grid_.data(), height_, widths_.back()} {};
  // enum class Direction { N, NE, E, SE, S, SW, W, NW };
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
  static constexpr Cell a_closed_cell{0xff, 0x00};
  static constexpr Cell a_open_cell{0x00, 0x00};

  std::vector<Cell> grid_;
  stdex::mdspan<Cell,
                stdex::extents<stdex::dynamic_extent, stdex::dynamic_extent>>
      grid_view_;

  auto as_mdspan() const { return grid_view_; }

  bool is_linked(CellCoordinate c1, CellCoordinate c2) const override {
    auto neighbors = get_all_neighbors_(c1);
    auto n = ranges::find_if(neighbors, [c2](auto n) { return n && *n == c2; });
    if (n == neighbors.end()) {
      throw std::runtime_error(fmt::format("Bad neighbor: {} -> {}", c1, c2));
      return false;
    }
    return (grid_view_(c1.row, c1.col).walls &
            (1 << ranges::distance(neighbors.begin(), n))) == 0;
  }

  void unlink(CellCoordinate c1, CellCoordinate c2) override {
    link_(c1, c2, false);
    link_(c2, c1, false);
  }
  // Helper for linking two cells togetherr (must be neighbors)
  void link(CellCoordinate c1, CellCoordinate c2) override {
    // Weaving:
    // Check if we are a neighbor-neighbor
    if (enable_weaving && (std::abs(ssize_t(c1.row - c2.row)) == 2) ||
        (std::abs(ssize_t(c1.row - c2.row)) == (widths_[c1.row] - 2)) ||
        (std::abs(ssize_t(c1.col - c2.col)) == 2)
        // ((widths_[c1.row] + c1.col - c2.col) % widths_[c1.row]) == 2 ||
        // ((height_ + c1.row - c2.row) % height_) == 2
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
  void link_(CellCoordinate c1, CellCoordinate c2, bool link_or_unlink) {
    auto neighbors = get_all_neighbors_(c1);
    auto n = ranges::find_if(neighbors, [c2](auto n) { return n && *n == c2; });
    if (n == neighbors.end()) {
      throw std::runtime_error(fmt::format("Bad neighbor: {} -> {}", c1, c2));
    }
    if (link_or_unlink) {
      grid_view_(c1.row, c1.col).walls &=
          ~(1 << ranges::distance(neighbors.begin(), n));
    } else {
      grid_view_(c1.row, c1.col).walls |=
          (1 << ranges::distance(neighbors.begin(), n));
    }
  }

  bool is_under_cell(CellCoordinate c) const override {
    auto m = as_mdspan();
    return m(c.row, c.col).check_flag(Cell::Flags::UnderCell);
  }

  auto &operator()(CellCoordinate cell) const {
    const auto &[row, col] = cell;
    auto m = as_mdspan();
    return m(row, col);
  }

  // Reset grid to all solid walls
  void reset() { ranges::fill(grid_, a_closed_cell); }
  void reset_open() { ranges::fill(grid_, a_open_cell); }
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
    if (enable_weaving) {
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

  std::vector<CellCoordinate> get_connected_neighbors(CellCoordinate c) const {
    if (enable_weaving) {
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
    if (enable_weaving) {
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
void kruskel_maze(Grid &grid);
void prims_maze(Grid &grid);
void grow_sample_maze(Grid &grid);
void grow_last_maze(Grid &grid);
void grow_last_or_sample_maze(Grid &grid);
void ellers_maze(Grid &grid);
void recursive_division_maze(Grid &grid);
void no_walls_maze(Grid &grid);
void all_walls_maze(Grid &grid);

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
  if (grid.widths_.back() > 200 / 4) {
    return fmt::format_to(ctx.out(), "Maze too big for console!");
  }
  // Draw an initial boundry
  for (int col = 0; col < grid.widths_.back(); col++) {
    auto c = '-';
    fmt::format_to(ctx.out(), "{}{}{}+", c, c, c);
  }
  fmt::format_to(ctx.out(), "\n");

  for (size_t row = 0; row < grid.height_; row++) {
    // Draw verticals
    for (size_t col = 0; col < grid.widths_.back(); col++) {
      char c = grid(jt::maze::CellCoordinate{row, col}).e ? '|' : ' ';
      // v_wall_to_char(grid(jt::maze::CellCoordinate{row, col}).right)
      fmt::format_to(ctx.out(), "   {}", c);
    }
    fmt::format_to(ctx.out(), "\n");

    // Draw horizantals
    for (size_t col = 0; col < grid.widths_.back(); col++) {
      // auto c = h_wall_to_char(grid(jt::maze::CellCoordinate{row, col}).down);
      char c = grid(jt::maze::CellCoordinate{row, col}).s ? '-' : ' ';
      fmt::format_to(ctx.out(), "{}{}{}+", c, c, c);
    }
    fmt::format_to(ctx.out(), "\n");
  }

  return fmt::format_to(ctx.out(), "... {} x {}\n", grid.widths_.back(),
                        grid.height_);
}
