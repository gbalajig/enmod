#include "enmod/Grid.h"
#include "enmod/Policy.h" 
#include <stdexcept>
#include <sstream>
#include <cmath>

Grid::Grid(const json& config) : grid_config(config) {
    try {
        grid_name = config.at("name");
        rows = config.at("rows");
        cols = config.at("cols");
        grid_map.assign(rows, std::vector<CellType>(cols, CellType::EMPTY));
        for (const auto& wall_pos : config.at("walls")) {
            grid_map[wall_pos.at("row")][wall_pos.at("col")] = CellType::WALL;
        }
        for (const auto& smoke_cfg : config.at("smoke")) {
            Position p = {smoke_cfg.at("row"), smoke_cfg.at("col")};
            grid_map[p.row][p.col] = CellType::SMOKE;
            smoke_intensities[p] = smoke_cfg.value("intensity", "light");
        }
        for (const auto& exit_p : config.at("exits")) {
            Position p = {exit_p.at("row"), exit_p.at("col")};
            grid_map[p.row][p.col] = CellType::EXIT;
            exit_pos.push_back(p);
        }
        start_pos = {config.at("start").at("row"), config.at("start").at("col")};
        grid_map[start_pos.row][start_pos.col] = CellType::START;
    } catch (const json::exception& e) {
        throw std::runtime_error("Failed to parse grid config: " + std::string(e.what()));
    }
}
int Grid::getRows() const { return rows; }
int Grid::getCols() const { return cols; }
const std::string& Grid::getName() const { return grid_name; }
Position Grid::getStartPosition() const { return start_pos; }
const std::vector<Position>& Grid::getExitPositions() const { return exit_pos; }
const json& Grid::getConfig() const { return grid_config; }
bool Grid::isValid(int r, int c) const { return r >= 0 && r < rows && c >= 0 && c < cols; }
bool Grid::isWalkable(int r, int c) const { if (!isValid(r, c)) return false; return grid_map[r][c] != CellType::WALL; }
bool Grid::isExit(int r, int c) const { if (!isValid(r,c)) return false; return grid_map[r][c] == CellType::EXIT; }
CellType Grid::getCellType(const Position& pos) const { if (!isValid(pos.row, pos.col)) return CellType::WALL; return grid_map[pos.row][pos.col]; }
std::string Grid::getSmokeIntensity(const Position& pos) const { if (smoke_intensities.count(pos)) { return smoke_intensities.at(pos); } return ""; }

void Grid::addHazard(const json& event_config) {
    Position pos = {event_config.at("position").at("row"), event_config.at("position").at("col")};
    if (isValid(pos.row, pos.col)) {
        grid_map[pos.row][pos.col] = CellType::FIRE;
        std::string size = event_config.value("size", "small");
        int radius = 1;
        if(size == "medium") radius = 2;
        if(size == "large") radius = 3;
        active_fires.push_back({pos, size, radius});
    }
}

Cost Grid::getMoveCost(const Position& pos) const {
    if (!isValid(pos.row, pos.col)) return {};
    for(const auto& fire : active_fires){
        int dist = std::abs(pos.row - fire.pos.row) + std::abs(pos.col - fire.pos.col);
        if(dist <= fire.radius){
            if(dist == 0) return {1000, 100, 1};
            if(dist == 1) return {50, 10, 1};
            if(dist == 2) return {25, 5, 1};
            if(dist == 3) return {10, 2, 1};
        }
    }
    if (grid_map[pos.row][pos.col] == CellType::SMOKE) {
        if (smoke_intensities.count(pos) && smoke_intensities.at(pos) == "heavy") return {25, 4, 1};
        return {5, 2, 1};
    }
    return {0, 1, 1};
}
Position Grid::getNextPosition(const Position& current, Direction dir) const {
    switch (dir) {
        case Direction::UP: return {current.row - 1, current.col};
        case Direction::DOWN: return {current.row + 1, current.col};
        case Direction::LEFT: return {current.row, current.col - 1};
        case Direction::RIGHT: return {current.row, current.col + 1};
        default: return current;
    }
}
std::string Grid::cellToHtml(int r, int c, const std::string& content) const {
    std::string class_name; std::string text = content;
    switch (grid_map[r][c]) {
        case CellType::WALL: class_name = "wall"; text = "W"; break;
        case CellType::START: class_name = "start"; text = "S"; break;
        case CellType::EXIT: class_name = "exit"; text = "E"; break;
        case CellType::SMOKE: class_name = "smoke"; text = "~"; break;
        case CellType::FIRE: class_name = "fire"; text = "F"; break;
        default: break;
    }
    if (content.find("path") != std::string::npos) { class_name += " path"; }
    return "<td class='grid-cell " + class_name + "'>" + text + "</td>";
}
std::string Grid::toHtmlString() const {
     std::stringstream ss;
    ss << "<table class='grid-table'><tbody>";
    for(int i = 0; i < rows; ++i){
        ss << "<tr>";
        for(int j = 0; j < cols; ++j){ ss << cellToHtml(i,j); }
        ss << "</tr>";
    }
    ss << "</tbody></table>";
    return ss.str();
}
std::string Grid::toHtmlStringWithCost(const std::vector<std::vector<Cost>>& cost_map) const {
    std::stringstream ss;
    ss << "<table class='grid-table'><tbody>";
    for(int r = 0; r < rows; ++r){
        ss << "<tr>";
        for(int c = 0; c < cols; ++c){
             std::stringstream content_ss;
            if(cost_map[r][c].distance != MAX_COST) {
                content_ss << "S:" << cost_map[r][c].smoke << "<br>T:" << cost_map[r][c].time << "<br>D:" << cost_map[r][c].distance;
            }
            ss << cellToHtml(r,c, content_ss.str());
        }
        ss << "</tr>";
    }
    ss << "</tbody></table>";
    return ss.str();
}
std::string Grid::toHtmlStringWithPath(const std::vector<Position>& path) const {
     std::stringstream ss;
    ss << "<table class='grid-table'><tbody>";
    for(int r = 0; r < rows; ++r){
        ss << "<tr>";
        for(int c = 0; c < cols; ++c){
            bool is_path = false;
            for(const auto& p : path){ if(p.row == r && p.col == c){ is_path = true; break; } }
            ss << cellToHtml(r,c, is_path ? "path" : "");
        }
        ss << "</tr>";
    }
    ss << "</tbody></table>";
    return ss.str();
}
std::string Grid::toHtmlStringWithPolicy(const Policy& policy) const {
    std::stringstream ss;
    ss << "<table class='grid-table'><tbody>";
    for(int r = 0; r < rows; ++r){
        ss << "<tr>";
        for(int c = 0; c < cols; ++c){
            char dir_char = ' ';
            switch(policy.getDirection({r,c})){
                case Direction::UP: dir_char = '^'; break;
                case Direction::DOWN: dir_char = 'v'; break;
                case Direction::LEFT: dir_char = '<'; break;
                case Direction::RIGHT: dir_char = '>'; break;
                case Direction::STAY: dir_char = 'O'; break;
                default: break;
            }
            ss << cellToHtml(r,c, std::string(1, dir_char));
        }
        ss << "</tr>";
    }
    ss << "</tbody></table>";
    return ss.str();
}
std::string Grid::toHtmlStringWithAgent(const Position& agent_pos) const {
    std::stringstream ss;
    ss << "<table class='grid-table'><tbody>";
    for(int r = 0; r < rows; ++r){
        ss << "<tr>";
        for(int c = 0; c < cols; ++c){
            std::string content = "";
            if (r == agent_pos.row && c == agent_pos.col) { content = "A"; }
            ss << cellToHtml(r,c, content);
        }
        ss << "</tr>";
    }
    ss << "</tbody></table>";
    return ss.str();
}
void Grid::setCellUnwalkable(const Position& pos) {
    if (isValid(pos.row, pos.col)) {
        // Ensure we don't block an exit
        if (grid_map[pos.row][pos.col] != CellType::EXIT) {
            grid_map[pos.row][pos.col] = CellType::WALL;
        }
    }
}
