#include "enmod/Logger.h"
#include "enmod/ScenarioGenerator.h"
#include "enmod/Grid.h"
#include "enmod/Solver.h"
#include "enmod/HtmlReportGenerator.h"
// DP Solvers
#include "enmod/DynamicBIDPSolver.h" 
#include "enmod/DynamicAPISolver.h"
#include "enmod/DynamicFIDPSolver.h"
#include "enmod/DynamicAVISolver.h"
// RL Solvers
#include "enmod/DynamicQLearningSolver.h"
#include "enmod/DynamicSARSASolver.h"
#include "enmod/DynamicActorCriticSolver.h"
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
    solvers.push_back(std::make_unique<DynamicBIDPSolver>(grid));
    solvers.push_back(std::make_unique<DynamicFIDPSolver>(grid));
    solvers.push_back(std::make_unique<DynamicAVISolver>(grid));
    solvers.push_back(std::make_unique<DynamicAPISolver>(grid));
    solvers.push_back(std::make_unique<DynamicQLearningSolver>(grid));
    solvers.push_back(std::make_unique<DynamicSARSASolver>(grid));
    solvers.push_back(std::make_unique<DynamicActorCriticSolver>(grid));

    for (const auto& solver : solvers) {
        std::cout << "  - Running " << solver->getName() << "... ";
        solver->run();
        std::cout << "Done.\n";
        
        Cost final_cost = solver->getEvacuationCost();
        double weighted_cost = (final_cost.distance == MAX_COST) ? std::numeric_limits<double>::infinity() : (final_cost.smoke * 1000) + (final_cost.time * 10) + (final_cost.distance * 1);
        results.push_back({grid.getName(), solver->getName(), final_cost, weighted_cost});
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

        std::vector<json> scenarios;
        scenarios.push_back(ScenarioGenerator::generate(5, "5x5"));
        scenarios.push_back(ScenarioGenerator::generate(10, "10x10"));
        scenarios.push_back(ScenarioGenerator::generate(15, "15x15"));

        std::vector<Result> all_results;
        for (const auto& config : scenarios) {
            runScenario(config, report_root_path, all_results);
        }
        
        HtmlReportGenerator::generateSummaryReport(all_results, report_root_path);
        std::cout << "\nSimulation complete. Final summary written to " << report_root_path << "/_Summary_Report.html\n";

        Logger::log(LogLevel::INFO, "Application Finished Successfully");
        Logger::close();
    } catch (const std::exception& e) {
        std::cerr << "A critical error occurred: " << e.what() << std::endl;
        Logger::log(LogLevel::ERROR, "A critical error occurred: " + std::string(e.what()));
        Logger::close();
        return 1;
    }
    return 0;
}

