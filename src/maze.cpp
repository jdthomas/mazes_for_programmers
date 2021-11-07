#include <algorithm>
#include <array>
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

  std::array<std::optional<std::pair<size_t, size_t>>, 4>
  get_reachable_neighbors(size_t row, size_t col) const {
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

  std::array<std::optional<std::pair<size_t, size_t>>, 4>
  get_all_neighbors(size_t row, size_t col) const {
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
  // Draw an initial boundry
  for (int row = 0; row < grid.height_; row++) {
    for (int col = 0, run_start = 0; col < grid.width_; col++) {
      bool at_vert_boundry = grid(row, col).down == Wall::Boundry;
      bool at_horz_boundry = grid(row, col).right == Wall::Boundry;
      bool should_close = at_horz_boundry || (!at_vert_boundry && d(gen));
      if (should_close) {
        std::uniform_int_distribution<> distrib(run_start, col);
        grid(row, distrib(gen)).down = Wall::Open;
        run_start = col + 1;
      } else {
        grid(row, col).right = Wall::Open;
      }
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
    auto neighbors = grid.get_reachable_neighbors(x, y);
    ranges::transform(neighbors |
                          ranges::views::filter([](auto o) { return bool(o); }),
                      ranges::back_inserter(q), [d = depth + 1](auto n) {
                        return std::make_tuple(n->first, n->second, d);
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
