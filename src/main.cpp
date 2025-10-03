#include "enmod/Logger.h"
#include "enmod/ScenarioGenerator.h"
#include "enmod/Grid.h"
#include "enmod/Solver.h"
#include "enmod/HtmlReportGenerator.h"
// Static DP Solvers
#include "enmod/BIDP.h"
#include "enmod/FIDP.h"
#include "enmod/API.h"
// Dynamic DP Solvers
#include "enmod/DynamicBIDPSolver.h" 
#include "enmod/DynamicAPISolver.h"
#include "enmod/DynamicFIDPSolver.h"
#include "enmod/DynamicAVISolver.h"
// RL Solvers (Static and Dynamic headers)
#include "enmod/QLearningSolver.h"
#include "enmod/SARSASolver.h"
#include "enmod/ActorCriticSolver.h"
#include "enmod/DynamicQLearningSolver.h"
#include "enmod/DynamicSARSASolver.h"
#include "enmod/DynamicActorCriticSolver.h"
// EnMod-DP Solvers
#include "enmod/HybridDPRLSolver.h"
#include "enmod/AdaptiveCostSolver.h"
#include "enmod/InterlacedSolver.h"
#include "enmod/HierarchicalSolver.h"
#include "enmod/PolicyBlendingSolver.h"
// CPS simulator
#include "enmod/CPSController.h"
#include "enmod/MultiAgentCPSController.h"
#include <iostream>
#include <vector>
#include <string>
#include <iomanip>
#include <memory>
#include <filesystem>
#include <map>
#include <chrono>
#include <sstream>

#ifdef _MSC_VER
#pragma warning(disable : 4996) 
#endif

void runScenario(const json& config, const std::string& report_path, std::vector<Result>& results) {
    Grid grid(config);
    std::cout << "\n===== Running Scenario: " << grid.getName() << " (" << grid.getRows() << "x" << grid.getCols() << ") =====\n";
    std::string scenario_report_path = report_path + "/" + grid.getName();
    std::filesystem::create_directory(scenario_report_path);
    HtmlReportGenerator::generateInitialGridReport(grid, scenario_report_path);

    std::vector<std::unique_ptr<Solver>> solvers;
    
    // --- Static Planners ---
    solvers.push_back(std::make_unique<BIDP>(grid));
    solvers.push_back(std::make_unique<FIDP>(grid));
    solvers.push_back(std::make_unique<API>(grid));
    solvers.push_back(std::make_unique<QLearningSolver>(grid));
    solvers.push_back(std::make_unique<SARSASolver>(grid));
    solvers.push_back(std::make_unique<ActorCriticSolver>(grid));

    // --- Dynamic Simulators ---
    solvers.push_back(std::make_unique<DynamicBIDPSolver>(grid));
    solvers.push_back(std::make_unique<DynamicFIDPSolver>(grid));
    solvers.push_back(std::make_unique<DynamicAVISolver>(grid));
    solvers.push_back(std::make_unique<DynamicAPISolver>(grid));
    solvers.push_back(std::make_unique<DynamicQLearningSolver>(grid));
    solvers.push_back(std::make_unique<DynamicSARSASolver>(grid));
    solvers.push_back(std::make_unique<DynamicActorCriticSolver>(grid));

    // --- EnMod-DP Hybrid Approaches ---
    solvers.push_back(std::make_unique<HybridDPRLSolver>(grid));
    solvers.push_back(std::make_unique<AdaptiveCostSolver>(grid));
    solvers.push_back(std::make_unique<InterlacedSolver>(grid));
    solvers.push_back(std::make_unique<HierarchicalSolver>(grid));
    solvers.push_back(std::make_unique<PolicyBlendingSolver>(grid));

    for (const auto& solver : solvers) {
        std::cout << "  - Running " << solver->getName() << "... ";
        auto start_time = std::chrono::high_resolution_clock::now();
        solver->run();
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> execution_time = end_time - start_time;
        std::cout << "Done (" << execution_time.count() << " ms).\n";
        
        Cost final_cost = solver->getEvacuationCost();
        double weighted_cost = (final_cost.distance == MAX_COST) ? std::numeric_limits<double>::infinity() : (final_cost.smoke * 1000) + (final_cost.time * 10) + (final_cost.distance * 1);
        results.push_back({grid.getName(), solver->getName(), final_cost, weighted_cost, execution_time.count()});
        HtmlReportGenerator::generateSolverReport(*solver, scenario_report_path);
    }
}

int main() {
    try {
        std::filesystem::create_directory("logs");
        std::filesystem::create_directory("reports");
        Logger::init("logs/enmod_simulation.log");
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");
        std::string report_root_path = "reports/run_" + ss.str();
        std::filesystem::create_directory(report_root_path);
        std::cout << "Log file created at: logs/enmod_simulation.log\n";
        std::cout << "Reports will be generated in: " << report_root_path << "\n";

        // --- Run the new real-time Multi-Agent CPS simulation ---
        json cps_scenario = ScenarioGenerator::generate(20, "20x20_MultiAgent_CPS");
        MultiAgentCPSController cps_controller(cps_scenario, report_root_path, 5); // 5 agents
        cps_controller.run_simulation();

        Logger::log(LogLevel::INFO, "Application Finished Successfully");
        Logger::close();
    } catch (const std::exception& e) {
        std::cerr << "A critical error occurred: " << e.what() << std::endl;
        Logger::log(LogLevel::ERROR, "A critical error occurred: " + std::string(e.what()));
        return 1;
    }
    return 0;
}