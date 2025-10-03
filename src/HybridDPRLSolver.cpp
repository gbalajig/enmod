#include "enmod/HybridDPRLSolver.h"
#include "enmod/Logger.h"
#include <algorithm>
#include <cmath>

HybridDPRLSolver::HybridDPRLSolver(const Grid& grid_ref) 
    : Solver(grid_ref, "HybridDPRLSim"), current_mode(EvacuationMode::NORMAL) {
    // Pre-train the RL agent
    rl_solver = std::make_unique<QLearningSolver>(grid_ref);
    rl_solver->train(5000); // Pre-train with 5000 episodes
}

void HybridDPRLSolver::assessThreatAndSetMode(const Position& current_pos, const Grid& current_grid) {
    const auto& events = current_grid.getConfig().value("dynamic_events", json::array());
    current_mode = EvacuationMode::NORMAL; 

    for (const auto& event : events) {
        if (event.value("type", "") == "fire") {
            Position fire_pos = {event.at("position").at("row"), event.at("position").at("col")};
            if (current_grid.getCellType(fire_pos) == CellType::FIRE) {
                int radius = 1;
                if(event.value("size", "small") == "medium") radius = 2;
                if(event.value("size", "small") == "large") radius = 3;

                int dist = std::abs(current_pos.row - fire_pos.row) + std::abs(current_pos.col - fire_pos.col);
                if (dist <= 1) { current_mode = EvacuationMode::PANIC; return; }
                if (dist <= radius) { current_mode = EvacuationMode::ALERT; }
            }
        }
    }

    int dr[] = {-1, 1, 0, 0};
    int dc[] = {0, 0, -1, 1};
    for(int i = 0; i < 4; ++i) {
        Position neighbor = {current_pos.row + dr[i], current_pos.col + dc[i]};
        if(current_grid.getSmokeIntensity(neighbor) == "heavy"){
             if (current_mode != EvacuationMode::PANIC) current_mode = EvacuationMode::ALERT;
        }
    }
}


Direction HybridDPRLSolver::getNextMove(const Position& current_pos, const Grid& current_grid) {
    assessThreatAndSetMode(current_pos, current_grid);
    Cost::current_mode = current_mode;

    if (current_mode == EvacuationMode::PANIC) {
        return rl_solver->chooseAction(current_pos);
    } 
    else {
        BIDP step_planner(current_grid);
        step_planner.run();
        const auto& cost_map = step_planner.getCostMap();
        
        Cost best_neighbor_cost = cost_map[current_pos.row][current_pos.col];
        Direction best_direction = Direction::STAY;
        
        int dr[] = {-1, 1, 0, 0};
        int dc[] = {0, 0, -1, 1};
        Direction dirs[] = {Direction::UP, Direction::DOWN, Direction::LEFT, Direction::RIGHT};

        for (int i = 0; i < 4; ++i) {
            Position neighbor = {current_pos.row + dr[i], current_pos.col + dc[i]};
            if (current_grid.isWalkable(neighbor.row, neighbor.col)) {
                if (cost_map[neighbor.row][neighbor.col] < best_neighbor_cost) {
                    best_neighbor_cost = cost_map[neighbor.row][neighbor.col];
                    best_direction = dirs[i];
                }
            }
        }
        return best_direction;
    }
}

void HybridDPRLSolver::run() {
    // The original run loop is now effectively handled by the CPSController.
    // This function can be kept for compatibility with the old test harness.
    // For a real CPS, this function would not be used.
    Grid dynamic_grid = grid;
    Position current_pos = dynamic_grid.getStartPosition();
    total_cost = {0, 0, 0};
    
    for (int t = 0; t < 2 * (grid.getRows() * grid.getCols()); ++t) {
        Direction next_move = getNextMove(current_pos, dynamic_grid);
        total_cost = total_cost + dynamic_grid.getMoveCost(current_pos);
        current_pos = dynamic_grid.getNextPosition(current_pos, next_move);

        if (dynamic_grid.isExit(current_pos.row, current_pos.col)) {
            break;
        }
    }
}

Cost HybridDPRLSolver::getEvacuationCost() const { return total_cost; }

void HybridDPRLSolver::generateReport(std::ofstream& report_file) const {
    report_file << "<h2>Simulation History (Hybrid DP-RL Solver)</h2>\n";
    for (const auto& step : history) {
        std::string mode_str;
        switch(step.mode){
            case EvacuationMode::NORMAL: mode_str = "NORMAL"; break;
            case EvacuationMode::ALERT: mode_str = "ALERT"; break;
            case EvacuationMode::PANIC: mode_str = "PANIC"; break;
        }
        report_file << "<h3>Time Step: " << step.time_step << " (Mode: " << mode_str << ")</h3>\n";
        report_file << "<p><strong>Agent Position:</strong> (" << step.agent_pos.row << ", " << step.agent_pos.col << ")</p>\n";
        report_file << "<p><strong>Action Taken:</strong> " << step.action << "</p>\n";
        report_file << "<p><strong>Cumulative Cost:</strong> " << step.current_total_cost << "</p>\n";
        report_file << step.grid_state.toHtmlStringWithAgent(step.agent_pos);
    }
}