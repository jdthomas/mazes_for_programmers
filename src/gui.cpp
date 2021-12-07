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
void draw_path(const Grid &grid, sf::RenderWindow &window,
               const std::vector<CellCoordinate> &path,
               const sf::Color line_color = sf::Color::Red) {
  const float cell_width = 800.0 / (grid.width_ + 2);
  const float cell_height = 600.0 / (grid.height_ + 2);

  const auto center_of_cell = [cell_width, cell_height](auto row, auto col) {
    // Col/row correctly swapped here, i guess i have always been drwing these
    // sideways :P
    return std::pair{(1.5 + col) * cell_width, (1.5 + row) * cell_height};
  };

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
}

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
  std::vector<CellCoordinate> player_path{};
};

void draw_maze(sf::RenderWindow &window, const DrawableMaze &dmaze) {
  auto window_size = window.getSize();
  const float cell_width = 800.0 / (dmaze.grid.width_ + 2);
  const float cell_height = 600.0 / (dmaze.grid.height_ + 2);
  const auto line_color = sf::Color::Green;

  // Fill the cells??
  for (const auto &cell : dmaze.grid.positions()) {
    const auto &[row, col] = cell;
    float x1 = cell_width * (col + 1);
    float x2 = cell_width * (col + 2);

    float y1 = cell_height * (row + 1);
    float y2 = cell_height * (row + 2);

    // Fill?
    if (dmaze.show_solution) {
      sf::RectangleShape cell(sf::Vector2f(cell_width, cell_height));
      cell.setPosition(x1, y1);
      cell.setFillColor(dmaze.colorizer(dmaze.grid, row, col));
      window.draw(cell);
    }
  }

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
  // Draw rest of maze
  for (const auto &cell : dmaze.grid.positions()) {
    const auto &[row, col] = cell;
    float x1 = cell_width * (col + 1);
    float x2 = cell_width * (col + 2);

    float y1 = cell_height * (row + 1);
    float y2 = cell_height * (row + 2);

    // Right wall
    if (dmaze.grid(cell).right != Wall::Open) {
      sf::Vertex line[] = {sf::Vertex(sf::Vector2f(x2, y1), line_color),
                           sf::Vertex(sf::Vector2f(x2, y2), line_color)};

      window.draw(line, 2, sf::Lines);
    }

    // Bottom wall
    if (dmaze.grid(cell).down != Wall::Open) {
      sf::Vertex line[] = {sf::Vertex(sf::Vector2f(x1, y2), line_color),
                           sf::Vertex(sf::Vector2f(x2, y2), line_color)};

      window.draw(line, 2, sf::Lines);
    }
    // Draw start/end
    if (cell == dmaze.path.front()) {
      sf::CircleShape shape((std::min(cell_width, cell_height) / 2) * 0.8);
      shape.setPosition(x1 + cell_width * 0.1, y1 + cell_height * 0.1);
      shape.setFillColor(sf::Color::Cyan);
      window.draw(shape);
    }
    if (cell == dmaze.path.back()) {
      sf::CircleShape shape((std::min(cell_width, cell_height) / 2) * 0.8);
      shape.setPosition(x1 + cell_width * 0.1, y1 + cell_height * 0.1);
      shape.setFillColor(sf::Color::Magenta);
      window.draw(shape);
    }
    // if(dmaze.grid.masked_at(cell))
    // {
    //   sf::CircleShape shape((std::min(cell_width, cell_height) / 2) * 0.8);
    //   shape.setPosition(x1 + cell_width * 0.1, y1 + cell_height * 0.1);
    //   shape.setFillColor(sf::Color::Red);
    //   window.draw(shape);
    // }
  }

  if (dmaze.show_solution) {
    draw_path(dmaze.grid, window, dmaze.path);
  }
  draw_path(dmaze.grid, window, dmaze.player_path, sf::Color::Blue);

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
    draw_maze(window, *dmaze);

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

  gui_main(width, height, method_idx, msk);

  return 0;
}
