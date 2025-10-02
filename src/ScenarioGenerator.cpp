#include "enmod/ScenarioGenerator.h"
#include "enmod/Types.h"
#include "enmod/json.hpp"
#include <random>
#include <chrono>
#include <algorithm>
#include <cmath>

using json = nlohmann::json;

json ScenarioGenerator::generate(int size, const std::string& name) {
    json config;
    config["name"] = name;
    config["rows"] = size;
    config["cols"] = size;

    std::mt19937 rng(static_cast<unsigned int>(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::uniform_int_distribution<int> dist(0, size - 1);
    
    Position start_pos = {1, 1};
    config["start"] = {{"row", start_pos.row}, {"col", start_pos.col}};

    config["exits"] = json::array();
    Position primary_exit = {size - 2, size - 2};
    config["exits"].push_back({{"row", primary_exit.row}, {"col", primary_exit.col}});
    if (size > 10) config["exits"].push_back({{"row", 0}, {"col", size / 2}});
    if (size > 20) config["exits"].push_back({{"row", size / 2}, {"col", 0}});

    config["walls"] = json::array();
    int num_walls = (size * size) / 5;
    for (int i = 0; i < num_walls; ++i) {
        int r = dist(rng);
        int c = dist(rng);
        Position p = {r, c};
        bool is_start = (p == start_pos);
        bool is_exit = std::any_of(config["exits"].begin(), config["exits"].end(), 
            [&p](const json& exit_j){ return exit_j["row"] == p.row && exit_j["col"] == p.col; });
        if (!is_start && !is_exit) {
            config["walls"].push_back({{"row", r}, {"col", c}});
        }
    }

    config["smoke"] = json::array();
    int num_smoke = (size * size) / 20;
     for (int i = 0; i < num_smoke; ++i) {
        int r = dist(rng);
        int c = dist(rng);
        Position p = {r, c};
        bool is_start = (p == start_pos);
        bool is_exit = std::any_of(config["exits"].begin(), config["exits"].end(), 
            [&p](const json& exit_j){ return exit_j["row"] == p.row && exit_j["col"] == p.col; });
        if (!is_start && !is_exit) {
            std::string intensity = (rng() % 5 == 0) ? "heavy" : "light";
            config["smoke"].push_back({{"row", r}, {"col", c}, {"intensity", intensity}});
        }
    }

    config["dynamic_events"] = json::array();
    int path_dist1 = std::abs(start_pos.row - primary_exit.row) + std::abs(start_pos.col - primary_exit.col);
    Position fire_pos1 = {start_pos.row + (primary_exit.row - start_pos.row) / 2, start_pos.col + (primary_exit.col - start_pos.col) / 2};
    
    config["dynamic_events"].push_back({
        {"type", "fire"},
        {"time_step", path_dist1 / 2},
        {"position", {{"row", fire_pos1.row}, {"col", fire_pos1.col}}},
        {"size", "medium"}
    });

    if (size > 10) {
        Position second_exit = {0, size / 2};
        Position fire_pos2 = {second_exit.row + 2, second_exit.col};
        config["dynamic_events"].push_back({
            {"type", "fire"},
            {"time_step", (path_dist1 / 2) + 5},
            {"position", {{"row", fire_pos2.row}, {"col", fire_pos2.col}}},
            {"size", "small"}
        });
    }

    return config;
}

