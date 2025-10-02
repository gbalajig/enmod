#ifndef ENMOD_Q_LEARNING_SOLVER_H
#define ENMOD_Q_LEARNING_SOLVER_H

#include "RLSolver.h"

class QLearningSolver : public RLSolver {
public:
    QLearningSolver(const Grid& grid_ref);
    void run() override; // This will be the training step
    Cost getEvacuationCost() const override;
    void generateReport(std::ofstream& report_file) const override;
    
    void train(int episodes) override;
    Direction chooseAction(const Position& state) override;
};

#endif // ENMOD_Q_LEARNING_SOLVER_H
