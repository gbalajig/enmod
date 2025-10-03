#include "enmod/MultiAgentCPSController.h"
#include "enmod/Logger.h"
#include <fstream>
#include <iostream>
#include <filesystem>

MultiAgentCPSController::MultiAgentCPSController(const json& initial_config, const std::string& report_path, int num_agents) 
    : master_grid(initial_config), 
      report_generator(report_path + "/multi_agent_report.html"),
      report_path(report_path) {
    
    // Create a subdirectory for agent I/O files
    std::filesystem::create_directory(report_path + "/agent_io");

    // Initialize agents at different starting positions
    for (int i = 0; i < num_agents; ++i) {
        agents.push_back({"agent_" + std::to_string(i), {i + 1, 1}});
    }

    // The solver is shared among all agents
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
    std::cout << "\n===== Starting Real-Time Multi-Agent CPS Simulation =====\n";
    
    for (int t = 0; t < 2 * (master_grid.getRows() * master_grid.getCols()); ++t) {
        std::cout << "Timestep " << t << std::endl;

        // Update master grid with dynamic events
        for (const auto& event_cfg : master_grid.getConfig().value("dynamic_events", json::array())) {
            if (event_cfg.value("time_step", -1) == t) {
                master_grid.addHazard(event_cfg);
            }
        }
        
        std::vector<Position> agent_positions;
        for(const auto& agent : agents) {
            agent_positions.push_back(agent.position);
        }
        report_generator.add_timestep(t, master_grid, agent_positions);

        bool all_exited = true;
        for (auto& agent : agents) {
            if (master_grid.isExit(agent.position.row, agent.position.col)) {
                continue;
            }
            all_exited = false;

            // 1. Simulate agent sending data
            write_input_file(t, agent);

            // 2. Server processes the data and decides next move
            Grid agent_grid = master_grid;
            for (const auto& other_agent : agents) {
                if (agent.id != other_agent.id) {
                    agent_grid.addHazard({{"position", {{"row", other_agent.position.row}, {"col", other_agent.position.col}}}, {"type", "wall"}});
                }
            }
            Direction next_move = solver->getNextMove(agent.position, agent_grid);

            // 4. Server sends command back to agent
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

            // 5. Simulate agent receiving and executing the move
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
    std::cout << "\nMulti-agent simulation complete. Report generated at " << report_path << "/multi_agent_report.html\n";
}