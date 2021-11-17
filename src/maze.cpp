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
#include <pstl/algorithm>
#include <pstl/execution>
#include <random>
#include <range/v3/all.hpp>
#include <set>
#include <unordered_map>
#include <vector>

#include <SFML/Graphics.hpp>

namespace stdex = std::experimental;

namespace jt::maze {
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

bool operator<(const CellCoordinate &a, const CellCoordinate &b) {
  return a.row < b.row || (a.row == b.row && a.col < b.col);
}

bool operator==(const CellCoordinate &a, const CellCoordinate &b) {
  return a.row == b.row && a.col == b.col;
}

struct Grid {
  Grid(size_t width, size_t height);
  size_t width_, height_;

private:
  // TODO: Get rid of "Cell" and use extra dimension in mdspan
  struct Cell {
    Wall down, right;
  };
  std::vector<Cell> cells_;

  std::random_device rd;
  std::mt19937 gen;

public:
  auto as_mdspan() const {
    return stdex::mdspan<
        Cell, stdex::extents<stdex::dynamic_extent, stdex::dynamic_extent>>(
        const_cast<Cell *>(cells_.data()), height_, width_);
  }

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
    long dx = col_2 - col_1;
    long dy = row_2 - row_1;
    // fmt::print("linking: {} to {} delta: {}.{}\n", c1, c2, dx, dy);
    if (std::abs(dx) > 1 || std::abs(dy) > 1 ||
        (std::abs(dx) > 0 && std::abs(dy) > 0))
      throw std::runtime_error("Linking cells must be neighbors");

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

Grid::Grid(size_t width, size_t height)
    : width_{width}, height_{height}, cells_{width * height}, rd{}, gen{rd()} {

  auto m = as_mdspan();
  for (int row = 0; row < m.extent(0); row++) {
    m(row, m.extent(1) - 1).right = Wall::Boundry;
  }
  for (int col = 0; col < m.extent(1); col++) {
    m(m.extent(0) - 1, col).down = Wall::Boundry;
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

}; // namespace jt::maze

namespace jt::maze {

void binary_tree_maze(Grid &grid) {
  fmt::print("Generating maze by binary tree\n");
  std::random_device rd;
  std::mt19937 gen(rd());
  std::bernoulli_distribution d(0.5);

  auto per_cell_action = [&grid,&d,&gen](const auto &cell) {
                  auto go_down = d(gen);
                  auto s = grid.cell_south(cell);
                  auto e = grid.cell_east(cell);
                  if (s && (go_down || !e))
                    grid.link(cell, *s);
                  else if (e)
                    grid.link(cell, *e);
  };

#if 1
  auto p = grid.positions();
  std::for_each(pstl::execution::par, p.begin(), p.end(),
                per_cell_action);
#else
  // Might be able to do better if we run rows in parallel but sequentially
  // columns for data locality .
  auto r = ranges::views::iota(static_cast<size_t>(0),
                               static_cast<size_t>(grid.height_));
  std::for_each(pstl::execution::par_unseq, r.begin(), r.end(),
                [&per_cell_action, width=grid.width_](const auto &row) {
                  for (size_t col = 0; col < width; col++) {
                    per_cell_action(CellCoordinate{row,col});
                  }
                });
#endif
}

void sidewinder_maze(Grid &grid) {
  fmt::print("Generating maze by sidewinder\n");
  std::random_device rd;
  std::mt19937 gen(rd());
  std::bernoulli_distribution d(0.5);

#if 0
  auto p = grid.positions();
  size_t run_start = 0;
  // This depends on the fact that grid.positions is in row-col order. You
  // could parallize by row by keeping row_start separate, would still depend
  // on iterating within row in order though.
  ranges::for_each(p, [&](const auto &cell) {
    auto &[row, col] = cell;
    auto e = grid.cell_east(cell);
    auto s = grid.cell_south(cell);
    bool should_close = !e || (s && d(gen));
    if (should_close) {
      std::uniform_int_distribution<> distrib(run_start, col);
      size_t c = static_cast<size_t>(distrib(gen));
      grid.link({row, c}, {row + 1, c});
      run_start = (col + 1) % grid.width_;
    } else {
      grid.link(cell, *e);
    }
  });
#else
  auto r = ranges::views::iota(static_cast<size_t>(0),
                               static_cast<size_t>(grid.height_));
  std::for_each(pstl::execution::par_unseq, r.begin(), r.end(),
                [&grid, &d, &gen](const auto &row) {
                  size_t run_start = 0;
                  for (size_t col = 0; col < grid.width_; col++) {
                    auto e = grid.cell_east({row, col});
                    auto s = grid.cell_south({row, col});
                    bool should_close = !e || (s && d(gen));
                    if (should_close) {
                      std::uniform_int_distribution<> distrib(run_start, col);
                      size_t c = static_cast<size_t>(distrib(gen));
                      grid.link({row, c}, {row + 1, c});
                      run_start = col + 1;
                    } else {
                      grid.link({row, col}, *e);
                    }
                  }
                });
#endif
}

void random_walk_Aldous_Broder_maze(Grid &grid) {
  fmt::print("Generating maze by Aldous/Broder's\n");

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

void random_walk_Wilson_maze(Grid &grid) {
  fmt::print("Generating maze by Wilson's\n");
  std::random_device rd;
  std::mt19937 gen(rd());

  auto unvisited = grid.positions() | ranges::to<std::set>;
  auto first = grid.random_cell();
  unvisited.erase(first);

  while (unvisited.size() > 0) {
    // fmt::print("unvisited: {}\n", unvisited.size());
    auto cell = *jt_range_front(unvisited | ranges::views::sample(1, gen));
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

void recursive_backtracking_maze(Grid &grid) {
  fmt::print("Generating maze by backtracking\n");

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

auto dijkstra_distances(const Grid &grid, CellCoordinate start_cell) {
  std::deque<std::tuple<CellCoordinate, int>> q;
  q.push_back({start_cell, 1});

  fmt::print("Computing distances using dijkstra from {} \n", start_cell);
  std::vector<int> distances(grid.width_ * grid.height_);
  auto cell_distances = stdex::mdspan<
      int, stdex::extents<stdex::dynamic_extent, stdex::dynamic_extent>>(
      distances.data(), grid.height_, grid.width_);

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

static size_t method = 0;
std::array<char, 6> all_methods{'B', 'S', 'R', 'W', 'K', 'C'};
std::string method_name(char m) {
  switch (m) {
  case 'B':
    return "BinaryTree";
  case 'S':
    return "Sidewinder";
  case 'R':
    return "AldousBroder";
  case 'W':
    return "Wilson";
  case 'K':
    return "HuntAndKill";
  case 'C':
    return "RecursiveBacktraking";
  }
  return "unknown";
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
  for (const auto &cell : grid.positions()) {
    const auto &[row, col] = cell;
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
  for (const auto &cell : grid.positions()) {
    const auto &[row, col] = cell;
    float x1 = cell_width * (col + 1);
    float x2 = cell_width * (col + 2);

    float y1 = cell_height * (row + 1);
    float y2 = cell_height * (row + 2);

    // Right wall
    if (grid(cell).right != jt::maze::Wall::Open) {
      sf::Vertex line[] = {sf::Vertex(sf::Vector2f(x2, y1), line_color),
                           sf::Vertex(sf::Vector2f(x2, y2), line_color)};

      window.draw(line, 2, sf::Lines);
    }

    // Bottom wall
    if (grid(cell).down != jt::maze::Wall::Open) {
      sf::Vertex line[] = {sf::Vertex(sf::Vector2f(x1, y2), line_color),
                           sf::Vertex(sf::Vector2f(x2, y2), line_color)};

      window.draw(line, 2, sf::Lines);
    }
  }

  draw_path(grid, window, path);

  // Labels
  static sf::Font font;
  static std::once_flag load_font_flag;
  std::call_once(load_font_flag, []() {
    if (!font.loadFromFile("/Library/Fonts/Roboto-Thin.ttf")) {
      throw std::runtime_error("Cannot find font");
    }
  });
  sf::Text text;
  text.setFont(font);
  text.setString(method_name(all_methods[method]));
  text.setCharacterSize(22); // in pixels, not points!
  text.setFillColor(sf::Color::White);
  window.draw(text);
}

auto gen_maze(jt::maze::Grid &grid) {
  auto method_c = all_methods[method];
  using std::chrono::high_resolution_clock;
  auto t1 = high_resolution_clock::now();
  switch (method_c) {
  case 'B':
    binary_tree_maze(grid);
    break;
  case 'S':
    sidewinder_maze(grid);
    break;
  case 'R':
    random_walk_Aldous_Broder_maze(grid);
    break;
  case 'W':
    random_walk_Wilson_maze(grid);
    break;
  case 'K':
    hunt_and_kill_maze(grid);
    break;
  case 'C':
    recursive_backtracking_maze(grid);
    break;
  }
  auto t2 = high_resolution_clock::now();

  std::chrono::duration<double, std::milli> delta_ms = t2 - t1;
  fmt::print("Generated maze in {}\n", delta_ms);
  fmt::print("{}\n", grid);
  auto distances =
      dijkstra_distances(grid, {grid.width_ / 2, grid.height_ / 2});
  // fmt::print("D:{}\n", distances);

  auto p = grid.positions();
  auto dead_ends =
      ranges::accumulate(p | ranges::views::transform([&grid](auto pos) {
                           return grid.is_dead_end_cell(pos) ? 1 : 0;
                         }),
                         0);
  fmt::print("Dead ends: {}\n", dead_ends);

  return distances;
}

void gui_main(jt::maze::Grid &grid, std::vector<int> distances,
              std::vector<jt::maze::CellCoordinate> path) {

  auto d = stdex::mdspan<
      int, stdex::extents<stdex::dynamic_extent, stdex::dynamic_extent>>(
      distances.data(), grid.height_, grid.width_);
  auto max_path_len = *std::max_element(distances.begin(), distances.end());
  std::function<sf::Color(const jt::maze::Grid &, int, int)> colorizer =
      [&max_path_len, &d](auto &g, auto c, auto r) {
        int shade = 255 * d(c, r) / max_path_len;
        return sf::Color(shade, shade, shade);
      };

  // create the window
  sf::RenderWindow window(sf::VideoMode(800, 600), "Maze window");

  auto regen_maze = [&]() {
    fmt::print("Regenerating maze!\n");
    grid.reset();
    distances = gen_maze(grid);

    path = longest_path_(grid, distances);
    d = stdex::mdspan<
        int, stdex::extents<stdex::dynamic_extent, stdex::dynamic_extent>>(
        distances.data(), grid.height_, grid.width_);
    max_path_len = *std::max_element(distances.begin(), distances.end());
  };

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

      case sf::Event::TextEntered:
        switch (event.text.unicode) {
        case 'j':
        case 'J':
          method = (method + 1) % all_methods.size();
          break;
        case 'k':
        case 'K':
          method = (method + all_methods.size() - 1) % all_methods.size();
          break;
        }
        regen_maze();
        break;
      case sf::Event::KeyPressed:
        if (event.key.code == sf::Keyboard::Space) {
          regen_maze();
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
  auto method_id_for_char = [](auto m) {
    return std::distance(
        std::begin(all_methods),
        std::find(std::begin(all_methods), std::end(all_methods), m));
  };
  method = method_id_for_char(argv[3][0]);
  // stats?
  if (argv[3][0] == 'X') {
    std::unordered_map<char, float> averages;
    // we run stats instead!
    for (auto m : all_methods) {
      // if (m != 'B') continue;
      method = method_id_for_char(m);
      std::vector<int> dead_count;
      for (int i = 0; i < 100; i++) {
        grid.reset();
        gen_maze(grid);
        auto p = grid.positions();
        auto dead_ends =
            ranges::accumulate(p | ranges::views::transform([&grid](auto pos) {
                                 return grid.is_dead_end_cell(pos) ? 1 : 0;
                               }),
                               0);
        dead_count.push_back(dead_ends);
      }
      averages[m] =
          float(ranges::accumulate(dead_count, 0)) / dead_count.size();
    }
    auto cell_count = grid.width_ * grid.height_;
    fmt::print("Average dead-ends per {}x#{} maze ({} cells):\n", grid.width_,
               grid.height_, cell_count);
    for (auto kv : averages) {
      auto &[k, v] = kv;
      fmt::print("{}: {}/{} ({}%)\n", method_name(k), v, cell_count,
                 100 * v / cell_count);
      // AldousBroder : 115/400 (28%)
    }
    exit(0);
  }

  auto distances = gen_maze(grid);

  auto path = longest_path_(grid, distances);

  gui_main(grid, distances, path);
  return 0;
}
