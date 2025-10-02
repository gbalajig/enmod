#ifndef ENMOD_INTERLACED_SOLVER_H
#define ENMOD_INTERLACED_SOLVER_H

#include "Solver.h"
#include <vector>
#include <string>

struct StepReport {
    int time_step;
    Grid grid_state;
    Position agent_pos;
    std::string action;
    Cost current_total_cost;
};

class InterlacedSolver : public Solver {
public:
    InterlacedSolver(const Grid& grid_ref);
    void run() override;
    Cost getEvacuationCost() const override;
    void generateReport(std::ofstream& report_file) const override;

private:
    std::vector<StepReport> history;
    Cost total_cost;
};

#endif // ENMOD_INTERLACED_SOLVER_H

