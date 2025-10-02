#include "enmod/InterlacedSolver.h"
#include "enmod/BIDP.h"
#include "enmod/Logger.h"
#include <algorithm>
#include <cmath>

InterlacedSolver::InterlacedSolver(const Grid& grid_ref) 
    : Solver(grid_ref, "InterlacedSim (BIDP)"), current_mode(EvacuationMode::NORMAL) {}

void InterlacedSolver::assessThreatAndSetMode(const Position& current_pos, const Grid& current_grid) {
    // Logic is unchanged, but now used here
    const auto& events = current_grid.getConfig().value("dynamic_events", json::array());
    current_mode = EvacuationMode::NORMAL; // Reset mode each step

    for (const auto& event : events) {
        if (event.value("type", "") == "fire") {
            Position fire_pos = {event.at("position").at("row"), event.at("position").at("col")};
            if (current_grid.getCellType(fire_pos) == CellType::FIRE) {
                int radius = event.value("impact_radius", 1);
                int dist = std::abs(current_pos.row - fire_pos.row) + std::abs(current_pos.col - fire_pos.col);
                if (dist <= 1) {
                    current_mode = EvacuationMode::PANIC;
                    return;
                }
                if (dist <= radius) {
                    current_mode = EvacuationMode::ALERT;
                }
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


void InterlacedSolver::run() {
    Grid dynamic_grid = grid;
    Position current_pos = dynamic_grid.getStartPosition();
    total_cost = {0, 0, 0};
    history.clear();

    const auto& events = dynamic_grid.getConfig().value("dynamic_events", json::array());

    for (int t = 0; t < 2 * (dynamic_grid.getRows() * dynamic_grid.getCols()); ++t) {
        
        for (const auto& event_cfg : events) {
            if (event_cfg.value("time_step", -1) == t) {
                Position hazard_pos = {event_cfg.at("position").at("row"), event_cfg.at("position").at("col")};
                dynamic_grid.addHazard(hazard_pos, CellType::FIRE);
            }
        }

        assessThreatAndSetMode(current_pos, dynamic_grid);
        Cost::current_mode = current_mode;
        
        history.push_back({t, dynamic_grid, current_pos, "Planning...", total_cost, current_mode});

        if (dynamic_grid.isExit(current_pos.row, current_pos.col)) {
            history.back().action = "SUCCESS: Reached Exit.";
            break;
        }

        // UPGRADED LOGIC: Re-plan at every step using BIDP
        BIDP step_planner(dynamic_grid);
        step_planner.run();
        const auto& cost_map = step_planner.getCostMap();
        
        // Find best move from current position by checking neighbors' costs on the new map
        Cost best_neighbor_cost;
        Position best_next_move = current_pos;
        std::string action = "STAY";
        
        int dr[] = {-1, 1, 0, 0};
        int dc[] = {0, 0, -1, 1};
        std::string actions[] = {"UP", "DOWN", "LEFT", "RIGHT"};

        for (int i = 0; i < 4; ++i) {
            Position neighbor = {current_pos.row + dr[i], current_pos.col + dc[i]};
            if (dynamic_grid.isWalkable(neighbor.row, neighbor.col)) {
                if (cost_map[neighbor.row][neighbor.col] < best_neighbor_cost) {
                    best_neighbor_cost = cost_map[neighbor.row][neighbor.col];
                    best_next_move = neighbor;
                    action = actions[i];
                }
            }
        }
        
        if(best_next_move == current_pos && step_planner.getEvacuationCost().distance == MAX_COST){
            history.back().action = "FAILURE: No path found.";
            total_cost = {};
            break;
        }

        history.back().action = action;
        total_cost = total_cost + dynamic_grid.getMoveCost(current_pos);
        current_pos = best_next_move;
    }
     if(history.back().action.find("SUCCESS") == std::string::npos && history.back().action.find("FAILURE") == std::string::npos){
         history.back().action = "FAILURE: Timed out.";
         total_cost = {};
     }
     
     Cost::current_mode = EvacuationMode::NORMAL;
}

Cost InterlacedSolver::getEvacuationCost() const {
    return total_cost;
}

void InterlacedSolver::generateReport(std::ofstream& report_file) const {
    report_file << "<h2>Simulation History (Turn-by-Turn using BIDP Planner)</h2>\n";
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

