#include <algorithm>
#include <array>
#include <deque>
#include <experimental/mdspan>
#include <fmt/format.h>
#include <fmt/ranges.h>
#include <random>
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
  auto get_neighbors(size_t row, size_t col) const {
    auto m = as_mdspan();
    struct Neighbors {
      std::optional<std::pair<size_t, size_t>> north, east, south, west;
    };
    // returns up to four pairs of points that are accessible from this cell
    return Neighbors{
        .north = (row > 0 && m(row - 1, col).down == Wall::Open)
                     ? std::make_optional(std::make_pair(row - 1, col))
                     : std::nullopt,
        .south = (row < m.extent(0) - 1 && m(row, col).down == Wall::Open)
                     ? std::make_optional(std::make_pair(row + 1, col))
                     : std::nullopt,
        .west = (col > 0 && m(row, col - 1).right == Wall::Open)
                    ? std::make_optional(std::make_pair(row, col - 1))
                    : std::nullopt,
        .east = (col < m.extent(1) - 1 && m(row, col).right == Wall::Open)
                    ? std::make_optional(std::make_pair(row, col + 1))
                    : std::nullopt,
    };
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
    // fmt::print("q: {}, {},{} - {}\n", q.size(), x, y, depth);
    q.pop_front();
    if (m(x, y) != 0) {
      continue;
    }
    m(x, y) = depth;
    auto neighbors = grid.get_neighbors(x, y);
    if (neighbors.north) {
      q.push_back({neighbors.north->first, neighbors.north->second, depth + 1});
    }
    if (neighbors.east) {
      q.push_back({neighbors.east->first, neighbors.east->second, depth + 1});
    }
    if (neighbors.south) {
      q.push_back({neighbors.south->first, neighbors.south->second, depth + 1});
    }
    if (neighbors.west) {
      q.push_back({neighbors.west->first, neighbors.west->second, depth + 1});
    }
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

void draw_maze(
    const jt::maze::Grid &g, sf::RenderWindow &window,
    std::function<sf::Color(const jt::maze::Grid &, int, int)> colorizer) {
  auto window_size = window.getSize();
  auto m = g.as_mdspan();
  const float cell_width = 800.0 / (m.extent(1) + 2);
  const float cell_height = 600.0 / (m.extent(0) + 2);
  const auto line_color = sf::Color::Green;
  // Fill the cells??
  for (int row = 0; row < m.extent(0); row++) {
    for (int col = 0; col < m.extent(1); col++) {
      float x1 = cell_width * (col + 1);
      float x2 = cell_width * (col + 2);

      float y1 = cell_height * (row + 1);
      float y2 = cell_height * (row + 2);

      // Fill?
      {
        sf::RectangleShape cell(sf::Vector2f(cell_width, cell_height));
        cell.setPosition(x1, y1);
        cell.setFillColor(colorizer(g, row, col));
        window.draw(cell);
      }
    }
  }

  // Draw top border
  {
    float x1 = cell_width * (0 + 1);

    float y1 = cell_height * (0 + 1);
    float y2 = cell_height * (m.extent(0) + 1);
    sf::Vertex line[] = {sf::Vertex(sf::Vector2f(x1, y1), line_color),
                         sf::Vertex(sf::Vector2f(x1, y2), line_color)};

    window.draw(line, 2, sf::Lines);
  }

  // Draw left border
  {
    float x1 = cell_width * (0 + 1);
    float x2 = cell_width * (m.extent(1) + 1);

    float y1 = cell_height * (0 + 1);
    sf::Vertex line[] = {sf::Vertex(sf::Vector2f(x1, y1), line_color),
                         sf::Vertex(sf::Vector2f(x2, y1), line_color)};

    window.draw(line, 2, sf::Lines);
  }
  // Draw rest of maze
  for (int row = 0; row < m.extent(0); row++) {
    for (int col = 0; col < m.extent(1); col++) {
      float x1 = cell_width * (col + 1);
      float x2 = cell_width * (col + 2);

      float y1 = cell_height * (row + 1);
      float y2 = cell_height * (row + 2);

      // Right wall
      if (m(row, col).right != jt::maze::Wall::Open) {
        sf::Vertex line[] = {sf::Vertex(sf::Vector2f(x2, y1), line_color),
                             sf::Vertex(sf::Vector2f(x2, y2), line_color)};

        window.draw(line, 2, sf::Lines);
      }

      // Bottom wall
      if (m(row, col).down != jt::maze::Wall::Open) {
        sf::Vertex line[] = {sf::Vertex(sf::Vector2f(x1, y2), line_color),
                             sf::Vertex(sf::Vector2f(x2, y2), line_color)};

        window.draw(line, 2, sf::Lines);
      }
    }
  }
}

void gui_main(
    jt::maze::Grid g,
    std::function<sf::Color(const jt::maze::Grid &, int, int)> colorizer) {
  // create the window
  sf::RenderWindow window(sf::VideoMode(800, 600), "Maze window");

  // run the program as long as the window is open
  while (window.isOpen()) {
    // check all the window's events that were triggered since the last
    // iteration of the loop
    sf::Event event;
    while (window.pollEvent(event)) {
      // "close requested" event: we close the window
      if (event.type == sf::Event::Closed)
        window.close();
    }

    // clear the window with black color
    window.clear(sf::Color::Black);

    // draw everything here...
    draw_maze(g, window, colorizer);

    // end the current frame
    window.display();
  }
}

int main(int argc, char **argv) {

  jt::maze::Grid g{static_cast<size_t>(std::strtol(argv[1], nullptr, 10)),
                   static_cast<size_t>(std::strtol(argv[2], nullptr, 10))};
  if (argv[3][0] == 'B') {
    binary_tree_maze(g);
  } else if (argv[3][0] == 'S') {
    sidewinder_maze(g);
  }
  fmt::print("{}\n", g);
  auto distances = dijkstra_distances(g, g.width_/2, g.height_/2);
  auto d = stdex::mdspan<
      int, stdex::extents<stdex::dynamic_extent, stdex::dynamic_extent>>(
      distances.data(), g.height_, g.width_);
  auto max_path_len = *std::max_element(distances.begin(), distances.end());
  fmt::print("D:{}\n", distances);
  gui_main(g, [&](auto g, auto c, auto r) {
    int shade = 255 * d(c, r) / max_path_len;
    return sf::Color(shade, shade, shade);
  });
  return 0;
}
