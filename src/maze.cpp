#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <experimental/mdspan>
#include <fmt/format.h>
#include <fmt/ranges.h>
#include <iostream>
#include <random>
#include <range/v3/all.hpp>
#include <set>
#include <vector>

#include <SFML/Graphics.hpp>

namespace stdex = std::experimental;

namespace jt::maze {

enum class Wall { Solid, Open, Boundry };

struct Cell {
  Wall down, right;
};

struct CellCoordinate {
  size_t row, col;
};

bool operator<(const CellCoordinate &a, const CellCoordinate &b) {
  return a.row < b.row || (a.row == b.row && a.col < b.col);
}

bool operator==(const CellCoordinate &a, const CellCoordinate &b) {
  return a.row == b.row && a.col == b.col;
}

struct Grid {
  Grid(size_t width, size_t height);
  size_t width_, height_;
  std::vector<Cell> cells_;
  auto as_mdspan() const {
    return stdex::mdspan<
        Cell, stdex::extents<stdex::dynamic_extent, stdex::dynamic_extent>>(
        const_cast<Cell *>(cells_.data()), height_, width_);
  }

  auto &operator()(size_t row, size_t col) const __attribute__((deprecated)) {
    auto m = as_mdspan();
    return m(row, col);
  }

  auto &operator()(CellCoordinate cell) const {
    const auto &[row, col] = cell;
    auto m = as_mdspan();
    return m(row, col);
  }

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

  auto random_cell() {
    // TODO: Move random stuffs to class
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> d_w(0, width_ - 1);
    std::uniform_int_distribution<> d_h(0, height_ - 1);
    return CellCoordinate{static_cast<size_t>(d_h(gen)),
                          static_cast<size_t>(d_w(gen))};
  }

  std::array<std::optional<CellCoordinate>, 4>
  get_connected_neighbors_(CellCoordinate c) const {
    auto &[row, col] = c;
    auto m = as_mdspan();
    return {
        (row > 0 && m(row - 1, col).down == Wall::Open)
            ? std::make_optional(CellCoordinate{row - 1, col})
            : std::nullopt,
        (row < m.extent(0) - 1 && m(row, col).down == Wall::Open)
            ? std::make_optional(CellCoordinate{row + 1, col})
            : std::nullopt,
        (col > 0 && m(row, col - 1).right == Wall::Open)
            ? std::make_optional(CellCoordinate{row, col - 1})
            : std::nullopt,
        (col < m.extent(1) - 1 && m(row, col).right == Wall::Open)
            ? std::make_optional(CellCoordinate{row, col + 1})
            : std::nullopt,
    };
  }

  std::vector<CellCoordinate> get_connected_neighbors(CellCoordinate c) const {
    auto n = get_connected_neighbors_(c);
    return n | ranges::views::filter([](auto o) { return bool(o); }) |
           ranges::views::transform([](auto o) { return *o; }) |
           ranges::to<std::vector>;
  }

  std::array<std::optional<CellCoordinate>, 4>
  get_all_neighbors_(CellCoordinate c) const {
    auto &[row, col] = c;
    auto m = as_mdspan();
    return {
        (row > 0) ? std::make_optional(CellCoordinate{row - 1, col})
                  : std::nullopt,
        (row < m.extent(0) - 1)
            ? std::make_optional(CellCoordinate{row + 1, col})
            : std::nullopt,
        (col > 0) ? std::make_optional(CellCoordinate{row, col - 1})
                  : std::nullopt,
        (col < m.extent(1) - 1)
            ? std::make_optional(CellCoordinate{row, col + 1})
            : std::nullopt,
    };
  }

  std::vector<CellCoordinate> get_all_neighbors(CellCoordinate c) const {
    auto n = get_all_neighbors_(c);
    return n | ranges::views::filter([](auto o) { return bool(o); }) |
           ranges::views::transform([](auto o) { return *o; }) |
           ranges::to<std::vector>;
  }

  void link(CellCoordinate c1, CellCoordinate c2) {
    auto &[row_1, col_1] = c1;
    auto &[row_2, col_2] = c2;
    long dx = col_2 - col_1;
    long dy = row_2 - row_1;
    fmt::print("linking: {}.{} to {}.{} delta: {}.{}\n", col_1, row_1, col_2,
               row_2, dx, dy);
    if (std::abs(dx) > 1 || std::abs(dy) > 1 ||
        (std::abs(dx) > 0 && std::abs(dy) > 0))
      throw std::runtime_error("Linking cells must be neighbors");

    auto m = as_mdspan();
    if (dx == -1) {
      m(row_1, col_1 - 1).right = Wall::Open;
      fmt::print("Setting right of {} {}\n", col_1 - 1, row_1);
    } else if (dx == 1) {
      m(row_1, col_1).right = Wall::Open;
      fmt::print("Setting right of {} {}\n", col_1, row_1);
    } else if (dy == -1) {
      m(row_1 - 1, col_1).down = Wall::Open;
      fmt::print("Setting down of {} {}\n", col_1, row_1 - 1);
    } else if (dy == 1) {
      m(row_1, col_1).down = Wall::Open;
      fmt::print("Setting down of {} {}\n", col_1, row_1);
    }
  }

  bool is_closed_cell(CellCoordinate c) const {
    auto tmp = get_connected_neighbors_(c);
    return ranges::accumulate(
               tmp | ranges::views::transform([](auto o) { return o ? 1 : 0; }),
               0) == 0;
  }

  bool is_dead_end_cell(CellCoordinate c) const {
    auto tmp = get_connected_neighbors_(c);
    return ranges::accumulate(
               tmp | ranges::views::transform([](auto o) { return o ? 1 : 0; }),
               0) == 1;
  }

  void reset() {
    for (auto &c : cells_) {
      c.down = Wall::Solid;
      c.right = Wall::Solid;
    }
  }
};

Grid::Grid(size_t width, size_t height)
    : width_{width}, height_{height}, cells_{width * height} {

  auto m = as_mdspan();
  for (int row = 0; row < m.extent(0); row++) {
    m(row, m.extent(1) - 1).right = Wall::Boundry;
  }
  for (int col = 0; col < m.extent(1); col++) {
    m(m.extent(0) - 1, col).down = Wall::Boundry;
  }
};

}; // namespace jt::maze

namespace jt::maze {

void binary_tree_maze(Grid &grid) {
  fmt::print("Generating maze by binary tree\n");
  std::random_device rd;
  std::mt19937 gen(rd());
  std::bernoulli_distribution d(0.5);

  auto p = grid.positions();
  ranges::for_each(p, [&](const auto &cellcoord) {
    auto &cell = grid(cellcoord);
    auto go_down = d(gen) && cell.down != Wall::Boundry;
    if (go_down) {
      cell.down = Wall::Open;
    } else if (cell.right != Wall::Boundry) {
      cell.right = Wall::Open;
    } else if (cell.down != Wall::Boundry) {
      cell.down = Wall::Open;
    }
  });
}

void sidewinder_maze(Grid &grid) {
  fmt::print("Generating maze by sidewinder\n");
  std::random_device rd;
  std::mt19937 gen(rd());
  std::bernoulli_distribution d(0.5);

  for (size_t row = 0; row < grid.height_; row++) {
    for (size_t col = 0, run_start = 0; col < grid.width_; col++) {
      bool at_vert_boundry = grid(row, col).down == Wall::Boundry;
      bool at_horz_boundry = grid(row, col).right == Wall::Boundry;
      bool should_close = at_horz_boundry || (!at_vert_boundry && d(gen));
      if (should_close) {
        std::uniform_int_distribution<> distrib(run_start, col);
        size_t c = static_cast<size_t>(distrib(gen));
        // grid(row, c).down = Wall::Open;
        grid.link({row, c}, {row + 1, c});
        assert(grid(row, c).down == Wall::Open);
        run_start = col + 1;
      } else {
        grid(row, col).right = Wall::Open;
      }
    }
  }
}

void random_walk_Aldous_Broder_maze(Grid &grid) {
  fmt::print("Generating maze by Aldous/Broder's\n");
  std::random_device rd;
  std::mt19937 gen(rd());

  // Select random starting cell
  auto cell = grid.random_cell();
  int unvisited = grid.width_ * grid.height_ - 1;
  while (unvisited > 0) {
    fmt::print("visiting: {}, {}\n", unvisited, cell);
    auto neighbors = grid.get_all_neighbors(cell);
    for (auto &n : neighbors) {
      fmt::print("  n: {}\n", n);
    }
    std::uniform_int_distribution<> d_n(0, neighbors.size() - 1);
    auto x = d_n(gen);
    // fmt::print("Selected {} from 0-{}\n", x, neighbors.size());
    auto neighbor = neighbors[x];

    if (grid.is_closed_cell(neighbor)) {
      grid.link(cell, neighbor);
      unvisited--;
    } else {
      cell = neighbor;
    }
  }
}

void random_walk_Wilson_maze(Grid &grid) {
  fmt::print("Generating maze by Wilson's\n");
  std::random_device rd;
  std::mt19937 gen(rd());

  auto unvisited = grid.positions() | ranges::to<std::set>;

  // Random from set :/
  {
    std::uniform_int_distribution<> d(0, unvisited.size() - 1);
    auto first = std::begin(unvisited);
    for (size_t i = d(gen); i > 0; i--) {
      first = std::next(first);
    }
    unvisited.erase(first);
  }

  while (unvisited.size() > 0) {
    fmt::print("unvisited: {}\n", unvisited.size());
    // Random from set :/
    auto cell_i = std::begin(unvisited);
    {
      std::uniform_int_distribution<> d(0, unvisited.size() - 1);
      for (size_t i = d(gen); i > 0; i--) {
        cell_i = std::next(cell_i);
      }
    }
    auto cell = *cell_i;
    // fmt::print("cell: {} dis={}\n", cell,
    // std::distance(std::begin(unvisited), cell_i));
    assert(unvisited.find(cell) != ranges::end(unvisited));
    std::vector<CellCoordinate> path{cell};

    while (unvisited.find(cell) != ranges::end(unvisited)) {
      // fmt::print("   : {}, \n", cell);
      // Update cell to a random neighbor of cell
      auto neighbors = grid.get_all_neighbors(cell);
      std::uniform_int_distribution<> d(0, neighbors.size() - 1);
      cell = neighbors[d(gen)];

      // Check if cell is in our path (e.g. loop)
      auto loop_begin =
          ranges::find_if(path, [cell](const auto &b) { return b == cell; });
      if (loop_begin == ranges::end(path)) {
        // no loop
        path.push_back(cell);
      } else {
        // fmt::print("Removing from path={}. {} ({})\n", path,
        //            std::distance(std::begin(path), loop_begin), *loop_begin);
        path.erase(std::next(loop_begin), std::end(path));
        // fmt::print("afterr remove path={}\n", path);
      }
    }

    fmt::print("Path done att {}, link it: {}\n", cell, path);
    auto _ = ranges::adjacent_find(path, [&grid](auto a, auto b) {
      grid.link(a, b);
      return false;
    });
    ranges::for_each(path, [&unvisited](auto a) { unvisited.erase(a); });
  }
}

void hunt_and_kill_maze(Grid &grid) {
  fmt::print("Generating maze by hunting and killing \n");
  std::random_device rd;
  std::mt19937 gen(rd());
  std::optional<CellCoordinate> current = grid.random_cell();

  while (current) {
    auto neighborhood = grid.get_all_neighbors(*current);
    auto neighbors = neighborhood | ranges::views::filter([grid](auto n) {
                       return grid.is_closed_cell(n);
                     }) |
                     ranges::to<std::vector>;

    // fmt::print("current: {}.{}, neighbors: {}\n", row, col, neighbors);

    if (!neighbors.empty()) {
      std::uniform_int_distribution<> d(0, neighbors.size() - 1);
      auto n = neighbors[d(gen)];
      grid.link(*current, n);
      current = n;
    } else {
      current = std::nullopt;

      // fmt::print("hunting...\n");
      for (auto pos : grid.positions()) {
        // if pos is not closed cell and has at least one closed cell neighbor,
        // set current to pos and break
        auto n = grid.get_all_neighbors(pos);
        auto unvisited_neighbors_of_pos =
            ranges::accumulate(n | ranges::views::transform([grid](auto np) {
                                 return grid.is_closed_cell(np) ? 1 : 0;
                               }),
                               0);
        if (!grid.is_closed_cell(pos) && unvisited_neighbors_of_pos > 0) {
          current = pos;
          break;
        }
      }
    }
  }
}

void recursive_backtracking_maze(Grid &grid) {
  fmt::print("Generating maze by backtracking\n");

  std::random_device rd;
  std::mt19937 gen(rd());

  std::vector<CellCoordinate> stack;
  stack.push_back(grid.random_cell());

  while (!stack.empty()) {
    auto current = stack.back();

    auto neighborhood = grid.get_all_neighbors(current);
    auto neighbors = neighborhood | ranges::views::filter([grid](auto n) {
                       return grid.is_closed_cell(n);
                     }) |
                     ranges::to<std::vector>;
    if (neighbors.empty()) {
      stack.pop_back();
    } else {
      std::uniform_int_distribution<> d(0, neighbors.size() - 1);
      auto n = neighbors[d(gen)];
      grid.link(current, n);
      stack.push_back(n);
    }
  }
}

auto dijkstra_distances(const Grid &grid, CellCoordinate start_cell) {
  std::deque<std::tuple<CellCoordinate, int>> q;
  q.push_back({start_cell, 1});

  fmt::print("Computing distances using dijkstra from {} \n", start_cell);
  std::vector<int> distances(grid.width_ * grid.height_);
  auto m = stdex::mdspan<
      int, stdex::extents<stdex::dynamic_extent, stdex::dynamic_extent>>(
      distances.data(), grid.height_, grid.width_);

  while (!q.empty()) {
    auto current = q.front();
    auto &[cell, depth] = current;
    q.pop_front();
    if (m(cell.row, cell.col) != 0) {
      continue;
    }
    m(cell.row, cell.col) = depth;
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
    fmt::print(" traversing path({}) on our way from {} at {} (d:{}), going to "
               "one of {}, {}\n",
               path.size(), goal, step, d(step.row, step.col), neighbors,
               neighbors | ranges::views::transform(
                               [d](auto n) { return d(n.row, n.col); }));
    step = *ranges::min_element(neighbors, ranges::less{},
                                [d](auto c) { return d(c.row, c.col); });
    // fmt::print("Next: {} (d:{})\n", step, d(step.row, step.col));
  }
  path.push_back(step);

  fmt::print("  found path: {}", path);
  std::cout.flush();

  // Update distances for rendering
  distances = new_distances;
  return path;
}
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
  return fmt::format_to(ctx.out(), "{}x{}", coord.row, coord.col);
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

void draw_path(const jt::maze::Grid &grid, sf::RenderWindow &window,
               std::vector<jt::maze::CellCoordinate> &path) {
  const float cell_width = 800.0 / (grid.width_ + 2);
  const float cell_height = 600.0 / (grid.height_ + 2);
  const auto line_color = sf::Color::Red;

  const auto center_of_cell = [cell_width, cell_height](auto row, auto col) {
    // Col/row correctly swapped here, i guess i have always been drwing these
    // sideways :P
    return std::pair{(1.5 + col) * cell_width, (1.5 + row) * cell_height};
  };

  if (path.empty())
    return;

  auto x = ranges::adjacent_find(
      path,
      [&window](auto a, auto b) {
        sf::Vertex l[] = {a, b};
        window.draw(l, 2, sf::Lines);
        return false;
      },
      [center_of_cell, line_color, &grid](auto pos) {
        auto &[row, col] = pos;
        assert(row < grid.height_ && col < grid.width_);
        auto draw_pos = center_of_cell(row, col);
        return sf::Vertex(sf::Vector2f(draw_pos.first, draw_pos.second),
                          line_color);
      });
}

void draw_maze(
    const jt::maze::Grid &grid, sf::RenderWindow &window,
    std::function<sf::Color(const jt::maze::Grid &, int, int)> colorizer,
    std::vector<jt::maze::CellCoordinate> &path) {
  auto window_size = window.getSize();
  const float cell_width = 800.0 / (grid.width_ + 2);
  const float cell_height = 600.0 / (grid.height_ + 2);
  const auto line_color = sf::Color::Green;
  // Fill the cells??
  for (int row = 0; row < grid.height_; row++) {
    for (int col = 0; col < grid.width_; col++) {
      float x1 = cell_width * (col + 1);
      float x2 = cell_width * (col + 2);

      float y1 = cell_height * (row + 1);
      float y2 = cell_height * (row + 2);

      // Fill?
      {
        sf::RectangleShape cell(sf::Vector2f(cell_width, cell_height));
        cell.setPosition(x1, y1);
        cell.setFillColor(colorizer(grid, row, col));
        window.draw(cell);
      }
    }
  }

  // Draw top border
  {
    float x1 = cell_width * (0 + 1);

    float y1 = cell_height * (0 + 1);
    float y2 = cell_height * (grid.height_ + 1);
    sf::Vertex line[] = {sf::Vertex(sf::Vector2f(x1, y1), line_color),
                         sf::Vertex(sf::Vector2f(x1, y2), line_color)};

    window.draw(line, 2, sf::Lines);
  }

  // Draw left border
  {
    float x1 = cell_width * (0 + 1);
    float x2 = cell_width * (grid.width_ + 1);

    float y1 = cell_height * (0 + 1);
    sf::Vertex line[] = {sf::Vertex(sf::Vector2f(x1, y1), line_color),
                         sf::Vertex(sf::Vector2f(x2, y1), line_color)};

    window.draw(line, 2, sf::Lines);
  }
  // Draw rest of maze
  for (int row = 0; row < grid.height_; row++) {
    for (int col = 0; col < grid.width_; col++) {
      float x1 = cell_width * (col + 1);
      float x2 = cell_width * (col + 2);

      float y1 = cell_height * (row + 1);
      float y2 = cell_height * (row + 2);

      // Right wall
      if (grid(row, col).right != jt::maze::Wall::Open) {
        sf::Vertex line[] = {sf::Vertex(sf::Vector2f(x2, y1), line_color),
                             sf::Vertex(sf::Vector2f(x2, y2), line_color)};

        window.draw(line, 2, sf::Lines);
      }

      // Bottom wall
      if (grid(row, col).down != jt::maze::Wall::Open) {
        sf::Vertex line[] = {sf::Vertex(sf::Vector2f(x1, y2), line_color),
                             sf::Vertex(sf::Vector2f(x2, y2), line_color)};

        window.draw(line, 2, sf::Lines);
      }
    }
  }

  // auto p = grid.positions() | ranges::views::transform([](auto p) {
  //            auto [i, j] = p;
  //            return std::make_pair(i, j);
  //          }) |
  //          ranges::to<std::vector>;
  draw_path(grid, window, path);
}

static char method = 'B';

auto gen_maze(jt::maze::Grid &grid) {
  if (method == 'B') {
    binary_tree_maze(grid);
  } else if (method == 'S') {
    sidewinder_maze(grid);
  } else if (method == 'R') {
    random_walk_Aldous_Broder_maze(grid);
  } else if (method == 'W') {
    random_walk_Wilson_maze(grid);
  } else if (method == 'K') {
    hunt_and_kill_maze(grid);
  } else if (method == 'C') {
    recursive_backtracking_maze(grid);
  }
  fmt::print("{}\n", grid);
  auto distances =
      dijkstra_distances(grid, {grid.width_ / 2, grid.height_ / 2});
  fmt::print("D:{}\n", distances);

  return distances;
}

void gui_main(jt::maze::Grid grid, std::vector<int> distances,
              std::vector<jt::maze::CellCoordinate> path) {

  auto d = stdex::mdspan<
      int, stdex::extents<stdex::dynamic_extent, stdex::dynamic_extent>>(
      distances.data(), grid.height_, grid.width_);
  auto max_path_len = *std::max_element(distances.begin(), distances.end());
  std::function<sf::Color(const jt::maze::Grid &, int, int)> colorizer =
      [&](auto g, auto c, auto r) {
        int shade = 255 * d(c, r) / max_path_len;
        return sf::Color(shade, shade, shade);
      };

  // create the window
  sf::RenderWindow window(sf::VideoMode(800, 600), "Maze window");

  // run the program as long as the window is open
  while (window.isOpen()) {
    // check all the window's events that were triggered since the last
    // iteration of the loop
    sf::Event event;
    while (window.pollEvent(event)) {
      // "close requested" event: we close the window
      switch (event.type) {
      case sf::Event::Closed:
        window.close();
        break;
      case sf::Event::KeyPressed:
        if (event.key.code == sf::Keyboard::Space) {
          fmt::print("Regenerating maze!\n");
          grid.reset();
          distances = gen_maze(grid);

          path = longest_path_(grid, distances);
          d = stdex::mdspan<int, stdex::extents<stdex::dynamic_extent,
                                                stdex::dynamic_extent>>(
              distances.data(), grid.height_, grid.width_);
          max_path_len = *std::max_element(distances.begin(), distances.end());
        }
        break;
      default:
        break;
      }
    }

    // clear the window with black color
    window.clear(sf::Color::Black);

    // draw everything here...
    draw_maze(grid, window, colorizer, path);

    // end the current frame
    window.display();
  }
}

int main(int argc, char **argv) {

  if (argc < 3)
    return 1;
  jt::maze::Grid grid{static_cast<size_t>(std::strtol(argv[1], nullptr, 10)),
                      static_cast<size_t>(std::strtol(argv[2], nullptr, 10))};
  method = argv[3][0];

  auto distances = gen_maze(grid);

  auto path = longest_path_(grid, distances);

  gui_main(grid, distances, path);
  return 0;
}
