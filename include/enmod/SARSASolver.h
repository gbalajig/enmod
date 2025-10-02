#ifndef ENMOD_SARSA_SOLVER_H
#define ENMOD_SARSA_SOLVER_H

#include "RLSolver.h"

class SARSASolver : public RLSolver {
public:
    SARSASolver(const Grid& grid_ref);
    void run() override;
    Cost getEvacuationCost() const override;
    void generateReport(std::ofstream& report_file) const override;
    
    void train(int episodes) override;
    Direction chooseAction(const Position& state) override;
};

#endif // ENMOD_SARSA_SOLVER_H
