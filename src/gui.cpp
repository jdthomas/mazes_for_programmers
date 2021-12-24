#include <fmt/chrono.h>
#include <fmt/format.h>
#include <fmt/ranges.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cxxopts.hpp>
#include <deque>
#include <experimental/mdspan>
#include <iostream>
#include <mutex>
#include <png++/png.hpp>
#if defined(__GLIBCXX__)
#include <execution>
namespace pstl = std;
#else
#include <pstl/algorithm>
#include <pstl/execution>
#endif
#include <SFML/Graphics.hpp>
#include <random>
#include <range/v3/all.hpp>
#include <set>
#include <unordered_map>
#include <vector>

#include "maze.h"

namespace stdex = std::experimental;
using namespace jt::maze;

////////////////////////////////////////////////////////////////////////////////
// GUI output
////////////////////////////////////////////////////////////////////////////////

struct DrawableMaze {
  DrawableMaze(GridMask msk, const GeneratorRegistry::RegistryConfig &method)
      : grid{msk} {
    init(method);
  }
  DrawableMaze(size_t width, size_t height,
               const GeneratorRegistry::RegistryConfig &method)
      : grid{width, height} {
    init(method);
  }

  void init(const GeneratorRegistry::RegistryConfig &method) {
    using std::chrono::high_resolution_clock;
    auto t1 = high_resolution_clock::now();
    method.generate(grid);
    auto t2 = high_resolution_clock::now();

    CellCoordinate start_pos =
        grid.random_cell();  // {grid.width_ / 2, grid.height_ / 2};
    distances = dijkstra_distances(grid, start_pos);
    path = longest_path_(grid, distances);
    method_name = method.name;
    // Player starts at the start of "path"
    player_path.push_back(path.front());

    auto t3 = high_resolution_clock::now();
    std::chrono::duration<double, std::milli> gen_delta_ms = t2 - t1;
    fmt::print("Generated maze in {}\n", gen_delta_ms);

    std::chrono::duration<double, std::milli> path_delta_ms = t3 - t2;
    fmt::print("Compute longest path maze with dijkstra in {}\n",
               path_delta_ms);

    max_path_len = *std::max_element(distances.begin(), distances.end());

    colorizer = [this](auto &g, auto c, auto r) {
      auto d = stdex::mdspan<
          int, stdex::extents<stdex::dynamic_extent, stdex::dynamic_extent>>(
          this->distances.data(), this->grid.height_, this->grid.width_);
      int shade = 255 * d(c, r) / this->max_path_len;
      return sf::Color(shade, shade, shade);
    };

    // Just some stats
    auto p = grid.positions();
    auto dead_ends =
        ranges::accumulate(p | ranges::views::transform([this](auto pos) {
                             return this->grid.is_dead_end_cell(pos) ? 1 : 0;
                           }),
                           0);
    fmt::print("Dead ends: {}\n", dead_ends);
  }

  Grid grid;
  std::vector<int> distances;
  std::vector<CellCoordinate> path;
  std::string method_name;
  size_t max_path_len;
  std::function<sf::Color(const Grid &, int, int)> colorizer;  // FIXME
  bool show_solution = false;
  bool polar_maze = false;
  std::vector<CellCoordinate> player_path{};
};

auto draw_path = [](const Grid &grid, sf::RenderWindow &window,
                    const std::vector<CellCoordinate> &path,
                    const auto center_of_cell,
                    const sf::Color line_color = sf::Color::Red) -> void {
  if (path.empty()) return;

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
};

void draw_circle_centered(sf::RenderWindow &window,
                          std::pair<float, float> center, float radius,
                          sf::Color color) {
  sf::CircleShape shape(radius);
  shape.setPosition(center.first - radius, center.second - radius);
  shape.setFillColor(color);
  window.draw(shape);
}

struct CellCorner {
  float x, y;
};
struct CellPoly {
  // cornders from top left clockwise...
  CellCorner a, b, c, d, i;
  // a ----- b
  //     i   |
  // d ----- c
};

auto draw_maze_common = [](sf::RenderWindow &window, const DrawableMaze &dmaze,
                           auto cell_polygon, auto center_of_cell,
                           auto cell_width, auto cell_height) -> void {
  const auto line_color = sf::Color::Green;
  if (dmaze.show_solution) {
    for (const auto &cell : dmaze.grid.positions()) {
      auto corners = cell_polygon(cell);
      sf::ConvexShape polygon;
      polygon.setPointCount(4);
      polygon.setPoint(0, sf::Vector2f(corners.a.x, corners.a.y));
      polygon.setPoint(1, sf::Vector2f(corners.b.x, corners.b.y));
      polygon.setPoint(2, sf::Vector2f(corners.c.x, corners.c.y));
      polygon.setPoint(3, sf::Vector2f(corners.d.x, corners.d.y));
      polygon.setFillColor(dmaze.colorizer(dmaze.grid, cell.row, cell.col));
      window.draw(polygon);
    }
  }

  // Draw rest of maze
  for (const auto &cell : dmaze.grid.positions()) {
    auto corners = cell_polygon(cell);
    if (!dmaze.grid.connected_cell_east(cell)) {
      sf::Vertex line[] = {
          sf::Vertex(sf::Vector2f(corners.b.x, corners.b.y), line_color),
          sf::Vertex(sf::Vector2f(corners.c.x, corners.c.y), line_color)};

      window.draw(line, 2, sf::Lines);
    }
    if (!dmaze.grid.connected_cell_south(cell)) {
      sf::Vertex line[] = {
          sf::Vertex(sf::Vector2f(corners.d.x, corners.d.y), line_color),
          sf::Vertex(sf::Vector2f(corners.c.x, corners.c.y), line_color)};

      window.draw(line, 2, sf::Lines);
    }
    // if(dmaze.grid.masked_at(cell))
    // {
    //   sf::CircleShape shape((std::min(cell_width, cell_height) / 2) * 0.8);
    //   shape.setPosition(x1 + cell_width * 0.1, y1 + cell_height * 0.1);
    //   shape.setFillColor(sf::Color::Red);
    //   window.draw(shape);
    // }
  }
  // Draw start/end
  {
    auto cell = dmaze.path.front();
    auto corners = cell_polygon(cell);
    draw_circle_centered(window, {corners.i.x, corners.i.y},
                         .8 * std::min(cell_width, cell_height) / 2,
                         sf::Color::Cyan);
  }
  {
    auto cell = dmaze.path.back();
    auto corners = cell_polygon(cell);
    draw_circle_centered(window, {corners.i.x, corners.i.y},
                         .8 * std::min(cell_width, cell_height) / 2,
                         sf::Color::Magenta);
  }

  if (dmaze.show_solution) {
    draw_path(dmaze.grid, window, dmaze.path, center_of_cell);
  }
  draw_path(dmaze.grid, window, dmaze.player_path, center_of_cell,
            sf::Color::Blue);

  // Labels
  static sf::Font font;
  static std::once_flag load_font_flag;
  std::call_once(load_font_flag, []() {
    if (!font.loadFromFile("/Library/Fonts/Roboto-Thin.ttf")) {
      throw std::runtime_error("Cannot find font");
    }
  });
  {
    sf::Text text;
    text.setFont(font);
    text.setString(dmaze.method_name);
    text.setCharacterSize(22);  // in pixels, not points!
    text.setFillColor(sf::Color::Yellow);
    window.draw(text);
  }

  if (dmaze.path.back() == dmaze.player_path.back()) {
    sf::Text text;
    text.setFont(font);
    text.setString("WINNER");
    text.setCharacterSize(44);  // in pixels, not points!
    text.setFillColor(sf::Color::Yellow);
    sf::FloatRect textRect = text.getLocalBounds();
    text.setOrigin(textRect.left + textRect.width / 2.0f,
                   textRect.top + textRect.height / 2.0f);
    text.setPosition(sf::Vector2f(800 / 2.0f, 600 / 2.0f));
    window.draw(text);
  }
};

void draw_polar_maze(sf::RenderWindow &window, const DrawableMaze &dmaze) {
  auto window_size = window.getSize();
  const float cell_width =
      800.0 / (2 * dmaze.grid.height_ + 2);  // yes use height
  const float cell_height = 600.0 / (2 * dmaze.grid.height_ + 2);
  const auto cell_size = std::min(cell_width, cell_height);
  const auto line_color = sf::Color::Green;
  const auto center_x = 400;
  const auto center_y = 300;
  constexpr double pi = M_PI;
  const auto theta = 2 * pi / dmaze.grid.width_;

  const auto cell_x_to_draw_x = [&](const auto cell) {
    auto &[row, col] = cell;
    const auto inner_radius = row * cell_size;
    const auto theta_ccw = col * theta;
    const float ax = center_x + (inner_radius * std::cos(theta_ccw));
    return ax;
  };
  const auto cell_y_to_draw_y = [&](const auto cell) {
    auto &[row, col] = cell;
    const auto inner_radius = row * cell_size;
    const auto theta_ccw = col * theta;
    const float ay = center_y + (inner_radius * std::sin(theta_ccw));
    return ay;
  };
  const auto center_of_cell = [&](const auto row, const auto col) {
    //
    return std::make_pair(
        cell_x_to_draw_x(std::make_pair(row + 0.5f, col + 0.5f)),
        cell_y_to_draw_y(std::make_pair(row + 0.5f, col + 0.5f)));
  };
  auto cell_polygon = [&](auto cell) -> CellPoly {
    const auto inner_radius = cell.row * cell_size;
    const auto outer_radius = (cell.row + 1) * cell_size;
    const auto theta_ccw = cell.col * theta;
    const auto theta_cw = (cell.col + 1) * theta;

    const float ax = center_x + (inner_radius * std::cos(theta_ccw));
    const float ay = center_y + (inner_radius * std::sin(theta_ccw));
    const float bx = center_x + (outer_radius * std::cos(theta_ccw));
    const float by = center_y + (outer_radius * std::sin(theta_ccw));
    const float cx = center_x + (inner_radius * std::cos(theta_cw));
    const float cy = center_y + (inner_radius * std::sin(theta_cw));
    const float dx = center_x + (outer_radius * std::cos(theta_cw));
    const float dy = center_y + (outer_radius * std::sin(theta_cw));

    return {
        .a = CellCorner{ax, ay},
        .b = CellCorner{cx, cy},
        .c = CellCorner{dx, dy},
        .d = CellCorner{bx, by},
        .i = CellCorner{cell_x_to_draw_x(
                            std::make_pair(cell.row + 0.5f, cell.col + 0.5f)),
                        cell_y_to_draw_y(
                            std::make_pair(cell.row + 0.5f, cell.col + 0.5f))},
    };
  };

  // Draw outer boundry
  {
    auto rad = cell_size * (dmaze.grid.height_);
    sf::CircleShape shape(rad);
    shape.setFillColor(sf::Color::Black);
    shape.setOutlineColor(line_color);
    shape.setOutlineThickness(1);
    shape.setPosition(center_x - rad, center_y - rad);
    window.draw(shape);
  }

  return draw_maze_common(window, dmaze, cell_polygon, center_of_cell,
                          cell_width, cell_height);
}

void draw_maze(sf::RenderWindow &window, const DrawableMaze &dmaze) {
  auto window_size = window.getSize();
  const float cell_width = 800.0 / (dmaze.grid.width_ + 2);
  const float cell_height = 600.0 / (dmaze.grid.height_ + 2);
  const auto line_color = sf::Color::Green;

  const auto cell_x_to_draw_x = [&](const auto row) {
    return cell_height * (row + 1);
  };
  const auto cell_y_to_draw_y = [&](const auto col) {
    return cell_width * (col + 1);
  };
  const auto center_of_cell = [&](const auto row, const auto col) {
    return std::make_pair(cell_y_to_draw_y(col + 0.5f),
                          cell_x_to_draw_x(row + 0.5f));
  };
  auto cell_polygon = [&](auto cell) -> CellPoly {
    const auto &[row, col] = cell;

    return {
        .a = CellCorner{cell_y_to_draw_y(col), cell_x_to_draw_x(row)},
        .b = CellCorner{cell_y_to_draw_y(col + 1), cell_x_to_draw_x(row)},
        .c = CellCorner{cell_y_to_draw_y(col + 1), cell_x_to_draw_x(row + 1)},
        .d = CellCorner{cell_y_to_draw_y(col), cell_x_to_draw_x(row + 1)},
        .i = CellCorner{cell_y_to_draw_y(col + 0.5f),
                        cell_x_to_draw_x(row + 0.5f)},
    };
  };

  // Draw top border
  {
    float x1 = cell_width * (0 + 1);

    float y1 = cell_height * (0 + 1);
    float y2 = cell_height * (dmaze.grid.height_ + 1);
    sf::Vertex line[] = {sf::Vertex(sf::Vector2f(x1, y1), line_color),
                         sf::Vertex(sf::Vector2f(x1, y2), line_color)};

    window.draw(line, 2, sf::Lines);
  }

  // Draw left border
  {
    float x1 = cell_width * (0 + 1);
    float x2 = cell_width * (dmaze.grid.width_ + 1);

    float y1 = cell_height * (0 + 1);
    sf::Vertex line[] = {sf::Vertex(sf::Vector2f(x1, y1), line_color),
                         sf::Vertex(sf::Vector2f(x2, y1), line_color)};

    window.draw(line, 2, sf::Lines);
  }

  return draw_maze_common(window, dmaze, cell_polygon, center_of_cell,
                          cell_width, cell_height);
}

void gui_main(size_t width, size_t height, size_t method_idx, GridMask mask) {
  std::unique_ptr<DrawableMaze> dmaze;
  auto regen_maze = [&dmaze, &method_idx, &width, &height, &mask]() {
    const auto &method = GeneratorRegistry::GetMazeGeneratorByIndex(method_idx);
    // dmaze = std::make_unique<DrawableMaze>(width, height, method);
    dmaze = std::make_unique<DrawableMaze>(mask, method);
  };

  regen_maze();

  // create the window
  sf::RenderWindow window(sf::VideoMode(800, 600), "Maze window");
  auto player_move = [&](auto next) {
    if (next) {
      if (dmaze->player_path.size() > 1 &&
          next == dmaze->player_path.rbegin()[1]) {
        dmaze->player_path.pop_back();
      } else {
        dmaze->player_path.push_back(*next);
      }
    }
  };

  // run the program as long as the window is open
  while (window.isOpen()) {
    // check all the window's events that were triggered since the last
    // iteration of the loop
    sf::Event event;
    bool need_regen = false;
    while (window.pollEvent(event)) {
      // "close requested" event: we close the window
      switch (event.type) {
        case sf::Event::Closed:
          window.close();
          break;

        case sf::Event::TextEntered:
          switch (event.text.unicode) {
            case '-':
              width--;
              need_regen = true;
              break;
            case '+':
              width++;
              need_regen = true;
              break;
            case '[':
              height--;
              need_regen = true;
              break;
            case ']':
              height++;
              need_regen = true;
              break;

            case 'j':
            case 'J':
              method_idx =
                  (method_idx + 1) % GeneratorRegistry::GetMazeGeneratorCount();
              need_regen = true;
              break;
            case 'k':
            case 'K':
              method_idx = (method_idx +
                            GeneratorRegistry::GetMazeGeneratorCount() - 1) %
                           GeneratorRegistry::GetMazeGeneratorCount();
              need_regen = true;
              break;
            case 'r':
              dmaze->polar_maze = !dmaze->polar_maze;
              break;
            case 'p':
              fmt::print("{}\n", dmaze->grid);
              break;
            case 's':
              dmaze->show_solution = !dmaze->show_solution;
              break;
            case 'q':
              return;
              break;
          }
        case sf::Event::KeyPressed:
          switch (event.key.code) {
            case sf::Keyboard::Space:
              need_regen = true;
              break;
            case sf::Keyboard::Left:
              player_move(
                  dmaze->grid.connected_cell_west(dmaze->player_path.back()));
              break;
            case sf::Keyboard::Right:
              player_move(
                  dmaze->grid.connected_cell_east(dmaze->player_path.back()));
              break;
            case sf::Keyboard::Down:
              player_move(
                  dmaze->grid.connected_cell_south(dmaze->player_path.back()));
              break;
            case sf::Keyboard::Up:
              player_move(
                  dmaze->grid.connected_cell_north(dmaze->player_path.back()));
              break;
            default:
              break;
          }
          break;
        default:
          break;
      }
    }
    if (need_regen) regen_maze();

    // clear the window with black color
    window.clear(sf::Color::Black);

    // draw everything here...
    if (dmaze->polar_maze) {
      draw_polar_maze(window, *dmaze);
    } else {
      draw_maze(window, *dmaze);
    }

    // end the current frame
    window.display();
  }
}

static void gui_usage() {
  std::string description =
      "========================================\n"
      "Solve the maze!\n"
      "Start at the cyan marker. Use the arrow keys to find a path to "
      "the magenta mark.\n"
      "\n"
      "Controls:\n"
      "arrows      move player path in direction of arrow.\n"
      "Space       generate a new maze.\n"
      "'j' / 'k'   cycle through various maze generators (and generate new "
      "maze).\n"
      "'-' / '+'   shrink/grow horizontally by one (and generate new maze).\n"
      "'[' / ']'   shrink/grow vertically by one (and generate new maze).\n"
      "'s'         toggle revealing/hiding the solution.\n"
      "========================================\n";
  fmt::print(description);
}

static auto read_mask(std::string fn, size_t width, size_t height, bool invert)
    -> GridMask {
  png::image<png::gray_pixel> image(fn);
  width = width == 0 ? image.get_width() : width;
  height = height == 0 ? image.get_height() : height;
  image.resize(width, height);
  std::vector<uint8_t> mask(width * height);
  auto pix_to_mask = [invert]() {
    return invert ? [](uint8_t p) -> uint8_t { return p > 128; }
                  : [](uint8_t p) -> uint8_t { return p < 128; };
  }();
  auto d = stdex::mdspan<
      uint8_t, stdex::extents<stdex::dynamic_extent, stdex::dynamic_extent>>(
      mask.data(), height, width);
  for (int i = 0; i < height; i++)
    for (int j = 0; j < width; j++) {
      d(i, j) = pix_to_mask(image.get_pixel(j, i));
    }
  fmt::print("{}", mask);
  return GridMask{width, height, std::move(mask)};
}

int main(int argc, char **argv) {
  cxxopts::Options options("maze_gui", "Play with mazes!");
  // clang-format off
  options.add_options()
    ("w,width", "width of maze to generate", cxxopts::value<size_t>()->default_value("10"))
    ("v,height", "width of maze to generate", cxxopts::value<size_t>()->default_value("10"))
    ("m,mask", "File name", cxxopts::value<std::string>())
    ("i,invert", "Invert the mask",  cxxopts::value<bool>()->default_value("false"))
    ("d,method", "Name of generation method",  cxxopts::value<std::string>()->default_value("B"))
    ("h,help", "Print usage")
    ;
  // clang-format on

  auto result = options.parse(argc, argv);
  if (result.count("help")) {
    fmt::print("{}\n", options.help());
    return 0;
  }
  size_t width = result["width"].as<size_t>();
  size_t height = result["height"].as<size_t>();

  // Calculate polar grid widths and set new width to max of that
  // auto w = calculate_variable_widths(height);
  // width = w.back();

  gui_usage();

  auto msk = result.count("mask")
                 ? read_mask(result["mask"].as<std::string>(), width, height,
                             result["invert"].as<bool>())
                 : GridMask{width, height, {}};
  auto method_name = result["method"].as<std::string>();
  size_t method_idx =
      GeneratorRegistry::GetMazeGeneratorIndexByShortName(method_name[0]);

  auto &m = GeneratorRegistry::AllMethods();
  jt::maze::ensure_registry();

  msk.mask.resize(width * height);
  // Mask out the polar part east of width
  // for (auto p : ranges::views::enumerate(w)) {
  //   auto &[r, c] = p;
  //   stdex::mdspan<uint8_t,
  //                 stdex::extents<stdex::dynamic_extent,
  //                 stdex::dynamic_extent>>
  //       m{msk.mask.data(), height, width};
  //   // auto m = dmaze.grid.mask_as_mdspan();
  //   for (auto cc : ranges::views::iota(int(c), int(width))) {
  //     fmt::print("{} {}\n", r, cc);
  //     m(r, cc) = 1;
  //   }
  // }

  gui_main(width, height, method_idx, msk);

  return 0;
}
