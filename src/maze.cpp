#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <experimental/mdspan>
#include <fmt/format.h>
#include <fmt/ranges.h>
#include <random>
#include <range/v3/all.hpp>
#include <vector>

#include <SFML/Graphics.hpp>

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

  auto &operator()(size_t i, size_t j) const {
    auto m = as_mdspan();
    return m(i, j);
  }

  auto positions() {
    return ranges::views::cartesian_product(
        ranges::views::iota(0, (int)height_),
        ranges::views::iota(0, (int)width_));
  }

  auto random_cell() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> d_w(0, width_ - 1);
    std::uniform_int_distribution<> d_h(0, height_ - 1);
    return std::make_pair(d_h(gen), d_w(gen));
  }

  std::array<std::optional<std::pair<size_t, size_t>>, 4>
  get_connected_neighbors_(size_t row, size_t col) const {
    auto m = as_mdspan();
    return {
        (row > 0 && m(row - 1, col).down == Wall::Open)
            ? std::make_optional(std::make_pair(row - 1, col))
            : std::nullopt,
        (row < m.extent(0) - 1 && m(row, col).down == Wall::Open)
            ? std::make_optional(std::make_pair(row + 1, col))
            : std::nullopt,
        (col > 0 && m(row, col - 1).right == Wall::Open)
            ? std::make_optional(std::make_pair(row, col - 1))
            : std::nullopt,
        (col < m.extent(1) - 1 && m(row, col).right == Wall::Open)
            ? std::make_optional(std::make_pair(row, col + 1))
            : std::nullopt,
    };
  }

  std::vector<std::pair<size_t, size_t>>
  get_connected_neighbors(size_t row, size_t col) const {
    auto n = get_connected_neighbors_(row, col);
    return n | ranges::views::filter([](auto o) { return bool(o); }) |
           ranges::views::transform([](auto o) { return *o; }) |
           ranges::to<std::vector>;
  }

  std::array<std::optional<std::pair<size_t, size_t>>, 4>
  get_all_neighbors_(size_t row, size_t col) const {
    auto m = as_mdspan();
    return {
        (row > 0) ? std::make_optional(std::make_pair(row - 1, col))
                  : std::nullopt,
        (row < m.extent(0) - 1)
            ? std::make_optional(std::make_pair(row + 1, col))
            : std::nullopt,
        (col > 0) ? std::make_optional(std::make_pair(row, col - 1))
                  : std::nullopt,
        (col < m.extent(1) - 1)
            ? std::make_optional(std::make_pair(row, col + 1))
            : std::nullopt,
    };
  }

  std::vector<std::pair<size_t, size_t>> get_all_neighbors(size_t row,
                                                           size_t col) const {
    auto n = get_all_neighbors_(row, col);
    return n | ranges::views::filter([](auto o) { return bool(o); }) |
           ranges::views::transform([](auto o) { return *o; }) |
           ranges::to<std::vector>;
  }

  void link(ssize_t row_1, ssize_t col_1, ssize_t row_2, ssize_t col_2) {
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

  bool is_closed_cell(size_t row, size_t col) const {
    auto tmp = get_connected_neighbors_(row, col);
    return ranges::accumulate(
               tmp | ranges::views::transform([](auto o) { return o ? 1 : 0; }),
               0) == 0;
  }

  bool is_dead_end_cell(size_t row, size_t col) const {
    auto tmp = get_connected_neighbors_(row, col);
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
  ranges::for_each(p, [&](const auto &idx) {
    auto &[row, col] = idx;
    auto &cell = grid(row, col);
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

  for (int row = 0; row < grid.height_; row++) {
    for (int col = 0, run_start = 0; col < grid.width_; col++) {
      bool at_vert_boundry = grid(row, col).down == Wall::Boundry;
      bool at_horz_boundry = grid(row, col).right == Wall::Boundry;
      bool should_close = at_horz_boundry || (!at_vert_boundry && d(gen));
      if (should_close) {
        std::uniform_int_distribution<> distrib(run_start, col);
        auto c = distrib(gen);
        // grid(row, c).down = Wall::Open;
        grid.link(row, c, row + 1, c);
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
  std::uniform_int_distribution<> d_w(0, grid.width_ - 1);
  std::uniform_int_distribution<> d_h(0, grid.height_ - 1);

  // Select random starting cell
  int row = d_h(gen), col = d_w(gen);
  int unvisited = grid.width_ * grid.height_ - 1;
  while (unvisited > 0) {
    fmt::print("visiting: {}, {}.{}\n", unvisited, row, col);
    auto neighbors = grid.get_all_neighbors(row, col);
    for (auto &n : neighbors) {
      fmt::print("  n: {} {}\n", n.first, n.second);
    }
    std::uniform_int_distribution<> d_n(0, neighbors.size() - 1);
    auto x = d_n(gen);
    // fmt::print("Selected {} from 0-{}\n", x, neighbors.size());
    auto neighbor = neighbors[x];

    if (grid.is_closed_cell(neighbor.first, neighbor.second)) {
      grid.link(row, col, neighbor.first, neighbor.second);
      unvisited--;
    } else {
      row = neighbor.first;
      col = neighbor.second;
    }
  }
}

void random_walk_Wilson_maze(Grid &grid) {
  fmt::print("Generating maze by Wilson's\n");
  throw std::runtime_error("TBD");
  // std::random_device rd;
  // std::mt19937 gen(rd());
  // std::uniform_int_distribution<> d_w(0, grid.width_);
  // std::uniform_int_distribution<> d_h(0, grid.height_);

  // const std::tuple<int,int> first{d_w(gen), d_h(gen)};

  // auto ps = grid.positions();
  // auto unvisited = ps | ranges::views::filter([first](const auto p) { return
  // p != first; }) | ranges::to<std::vector>; ranges::shuffle(unvisited);

  // while(unvisited.size()) {
  // }
}

void hunt_and_kill_maze(Grid &grid) {
  fmt::print("Generating maze by hunting and killing \n");
  std::random_device rd;
  std::mt19937 gen(rd());
  std::optional<std::pair<ssize_t, ssize_t>> current = grid.random_cell();

  while (current) {
    auto &[row, col] = *current;
    auto neighborhood = grid.get_all_neighbors(row, col);
    auto neighbors = neighborhood | ranges::views::filter([grid](auto n) {
                       return grid.is_closed_cell(n.first, n.second);
                     }) |
                     ranges::to<std::vector>;

    // fmt::print("current: {}.{}, neighbors: {}\n", row, col, neighbors);

    if (!neighbors.empty()) {
      std::uniform_int_distribution<> d(0, neighbors.size() - 1);
      auto n = neighbors[d(gen)];
      grid.link(row, col, n.first, n.second);
      current = n;
    } else {
      current = std::nullopt;

      // fmt::print("hunting...\n");
      for (auto pos : grid.positions()) {
        // if pos is not closed cell and has at least one closed cell neighbor,
        // set current to pos and break
        auto [i, j] = pos;
        auto n = grid.get_all_neighbors(i, j);
        auto unvisited_neighbors_of_pos = ranges::accumulate(
            n | ranges::views::transform([grid](auto np) {
              return grid.is_closed_cell(np.first, np.second) ? 1 : 0;
            }),
            0);
        if (!grid.is_closed_cell(i, j) && unvisited_neighbors_of_pos > 0) {
          current = {i, j};
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

  std::vector<std::pair<ssize_t, ssize_t>> stack;
  stack.push_back(grid.random_cell());

  while (!stack.empty()) {
    auto current = stack.back();
    auto &[row, col] = current;

    auto neighborhood = grid.get_all_neighbors(row, col);
    auto neighbors = neighborhood | ranges::views::filter([grid](auto n) {
                       return grid.is_closed_cell(n.first, n.second);
                     }) |
                     ranges::to<std::vector>;
    if (neighbors.empty()) {
      stack.pop_back();
    } else {
      std::uniform_int_distribution<> d(0, neighbors.size() - 1);
      auto n = neighbors[d(gen)];
      grid.link(row, col, n.first, n.second);
      stack.push_back(n);
    }
  }
}

auto dijkstra_distances(const Grid &grid, size_t start_x = 0,
                        size_t start_y = 0) {
  std::deque<std::tuple<size_t, size_t, int>> q;
  q.push_back({start_y, start_x, 1});

  std::vector<int> distances(grid.width_ * grid.height_);
  auto m = stdex::mdspan<
      int, stdex::extents<stdex::dynamic_extent, stdex::dynamic_extent>>(
      distances.data(), grid.height_, grid.width_);

  while (!q.empty()) {
    auto cell = q.front();
    auto &[x, y, depth] = cell;
    q.pop_front();
    if (m(x, y) != 0) {
      continue;
    }
    m(x, y) = depth;
    auto neighbors = grid.get_connected_neighbors(x, y);
    ranges::transform(neighbors, ranges::back_inserter(q),
                      [d = depth + 1](auto n) {
                        return std::make_tuple(n.first, n.second, d);
                      });
  }
  return distances;
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

void draw_maze(
    const jt::maze::Grid &grid, sf::RenderWindow &window,
    std::function<sf::Color(const jt::maze::Grid &, int, int)> colorizer) {
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
  auto distances = dijkstra_distances(grid, grid.width_ / 2, grid.height_ / 2);
  fmt::print("D:{}\n", distances);
  return distances;
}

void gui_main(jt::maze::Grid grid, std::vector<int> distances) {

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
    draw_maze(grid, window, colorizer);

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

  gui_main(grid, distances);
  return 0;
}
