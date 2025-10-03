#include "enmod/MultiAgentCPSController.h"
#include "enmod/Logger.h"
#include <fstream>
#include <iostream>
#include <filesystem>
#include <string>

MultiAgentCPSController::MultiAgentCPSController(const json& initial_config, const std::string& report_path, int num_agents)
    : master_grid(initial_config),
      report_generator(report_path + "/multi_agent_report.html"),
      report_path(report_path) {

    std::filesystem::create_directory(report_path + "/agent_io");

    // Initialize agents safely within the grid boundaries
    Position start_pos = master_grid.getStartPosition();
    for (int i = 0; i < num_agents; ++i) {
        Position agent_pos = {start_pos.row + i, start_pos.col};

        // If the calculated position is invalid or blocked, find a new one.
        while (!master_grid.isWalkable(agent_pos.row, agent_pos.col)) {
            agent_pos.col++; // Try the next column
            if (!master_grid.isValid(agent_pos.row, agent_pos.col)) {
                agent_pos.col--; // Move back if we went out of bounds
                agent_pos.row++; // Try the next row
            }
        }

        if (!master_grid.isValid(agent_pos.row, agent_pos.col)) {
            Logger::log(LogLevel::ERROR, "Could not place agent " + std::to_string(i) + " within grid bounds.");
            continue;
        }

        agents.push_back({"agent_" + std::to_string(i), agent_pos});
    }

    solver = std::make_unique<HybridDPRLSolver>(master_grid);
}

void MultiAgentCPSController::write_input_file(int timestep, const Agent& agent) {
    json input_data;
    input_data["agent_id"] = agent.id;
    input_data["timestep"] = timestep;
    input_data["current_position"] = {
        {"row", agent.position.row},
        {"col", agent.position.col}
    };
    input_data["environment_update"] = master_grid.getConfig();

    std::ofstream o(report_path + "/agent_io/" + agent.id + "_input_t" + std::to_string(timestep) + ".json");
    o << std::setw(4) << input_data << std::endl;
}

Direction MultiAgentCPSController::read_output_file(int timestep, const Agent& agent) {
    std::ifstream i(report_path + "/agent_io/" + agent.id + "_output_t" + std::to_string(timestep) + ".json");
    json output_data;
    i >> output_data;

    std::string move = output_data.at("next_move");
    if (move == "UP") return Direction::UP;
    if (move == "DOWN") return Direction::DOWN;
    if (move == "LEFT") return Direction::LEFT;
    if (move == "RIGHT") return Direction::RIGHT;
    return Direction::STAY;
}

void MultiAgentCPSController::run_simulation() {
    std::cout << "\n===== Starting Real-Time Multi-Agent CPS Simulation for " << master_grid.getName() << " =====\n";

    for (int t = 0; t < 2 * (master_grid.getRows() * master_grid.getCols()); ++t) {
        std::cout << "Timestep " << t << std::endl;

        for (const auto& event_cfg : master_grid.getConfig().value("dynamic_events", json::array())) {
            if (event_cfg.value("time_step", -1) == t) {
                master_grid.addHazard(event_cfg);
            }
        }

        std::vector<Position> agent_positions;
        for (const auto& agent : agents) {
            agent_positions.push_back(agent.position);
        }
        report_generator.add_timestep(t, master_grid, agent_positions);

        bool all_exited = true;
        for (auto& agent : agents) {
            if (master_grid.isExit(agent.position.row, agent.position.col)) {
                continue;
            }
            all_exited = false;

            write_input_file(t, agent);

            Grid agent_grid = master_grid;
            for (const auto& other_agent : agents) {
                if (agent.id != other_agent.id) {
                    agent_grid.setCellUnwalkable(other_agent.position);
                }
            }
            Direction next_move = solver->getNextMove(agent.position, agent_grid);

            json output_data;
            output_data["agent_id"] = agent.id;
            std::string move_str = "STAY";
            if (next_move == Direction::UP) move_str = "UP";
            else if (next_move == Direction::DOWN) move_str = "DOWN";
            else if (next_move == Direction::LEFT) move_str = "LEFT";
            else if (next_move == Direction::RIGHT) move_str = "RIGHT";
            output_data["next_move"] = move_str;

            std::ofstream o(report_path + "/agent_io/" + agent.id + "_output_t" + std::to_string(t) + ".json");
            o << std::setw(4) << output_data << std::endl;

            Direction received_move = read_output_file(t, agent);
            agent.position = master_grid.getNextPosition(agent.position, received_move);
        }

        if (all_exited) {
            std::cout << "SUCCESS: All agents reached the exit." << std::endl;
            Logger::log(LogLevel::INFO, "SUCCESS: All agents reached the exit.");
            break;
        }
    }

    report_generator.finalize_report();
    std::cout << "\nMulti-agent simulation for " << master_grid.getName() << " complete. Report generated at "
              << report_path << "/multi_agent_report.html\n";
}