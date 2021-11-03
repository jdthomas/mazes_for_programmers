#include <array>
#include <experimental/mdspan>
#include <fmt/format.h>
#include <random>
#include <vector>

namespace stdex = std::experimental;

namespace jt::maze {

enum class Wall { Solid, Open, Boundry };

struct Cell {
  Wall down, right;
};

struct Grid {
  Grid(size_t width, size_t height);
  size_t width_, height_;
  std::vector<Cell> cells_;
  auto as_mdspan() const {
    return stdex::mdspan<
        Cell, stdex::extents<stdex::dynamic_extent, stdex::dynamic_extent>>(
        const_cast<Cell *>(cells_.data()), height_, width_);
  }
};

Grid::Grid(size_t width, size_t height)
    : width_{width}, height_{height}, cells_{width * height} {

  // std::random_device rd;
  // std::mt19937 gen(rd());
  // std::bernoulli_distribution d(0.5);
  // for(auto& cell: cells_) {
  //   cell.down = d(gen) ? Wall::Solid : Wall::Open;
  //   cell.right = d(gen) ? Wall::Solid : Wall::Open;
  // }
  auto m = as_mdspan();
  for (int row = 0; row < m.extent(0); row++) {
    m(row, m.extent(1) - 1).right= Wall::Boundry;
  }
  for (int col = 0; col < m.extent(1); col++) {
    m(m.extent(0) - 1, col).down = Wall::Boundry;
  }
};

}; // namespace jt::maze

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
  auto m = grid.as_mdspan();
  // Draw an initial boundry
  for (int col = 0; col < m.extent(1); col++) {
    auto c = h_wall_to_char(jt::maze::Wall::Boundry);
    fmt::format_to(ctx.out(), "{}{}{}+", c, c, c);
  }
  fmt::format_to(ctx.out(), "\n");

  for (int row = 0; row < m.extent(0); row++) {
    // Draw verticals
    for (int col = 0; col < m.extent(1); col++) {
      fmt::format_to(ctx.out(), "   {}", v_wall_to_char(m(row, col).right));
    }
    fmt::format_to(ctx.out(), "\n");

    // Draw horizantals
    for (int col = 0; col < m.extent(1); col++) {
      auto c = h_wall_to_char(m(row, col).down);
      fmt::format_to(ctx.out(), "{}{}{}+", c, c, c);
    }
    fmt::format_to(ctx.out(), "\n");
  }

  return fmt::format_to(ctx.out(), "... {} x {}\n", grid.width_, grid.height_);
}

namespace jt::maze {

void binary_tree_maze(Grid &grid) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::bernoulli_distribution d(0.5);
  for (auto &cell : grid.cells_) {
    auto go_down = d(gen) && cell.down != Wall::Boundry;
    if (go_down) {
      cell.down = Wall::Open;
    } else if (cell.right != Wall::Boundry) {
      cell.right = Wall::Open;
    } else if (cell.down != Wall::Boundry) {
      cell.down = Wall::Open;
    }
  }
}

void sidewinder_maze(Grid &grid) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::bernoulli_distribution d(0.5);
  auto m = grid.as_mdspan();
  // Draw an initial boundry
  for (int row = 0; row < m.extent(0); row++) {
    for (int col = 0, run_start = 0; col < m.extent(1); col++) {
      bool at_vert_boundry = m(row, col).down == Wall::Boundry;
      bool at_horz_boundry = m(row, col).right == Wall::Boundry;
      bool should_close = at_horz_boundry || (!at_vert_boundry && d(gen));
      if (should_close) {
        std::uniform_int_distribution<> distrib(run_start, col);
        m(row, distrib(gen)).down = Wall::Open;
        run_start = col + 1;
      } else {
        m(row, col).right = Wall::Open;
      }
    }
  }
}

}; // namespace jt::maze

int main(int argc, char **argv) {

  jt::maze::Grid g{static_cast<size_t>(std::strtol(argv[1], nullptr, 10)),
                   static_cast<size_t>(std::strtol(argv[2], nullptr, 10))};
  if (argv[3][0] == 'B') {
    binary_tree_maze(g);
  } else if (argv[3][0] == 'S') {
    sidewinder_maze(g);
  }
  fmt::print("{}\n", g);
  return 0;
}
