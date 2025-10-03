#ifndef ENMOD_CPS_CONTROLLER_H
#define ENMOD_CPS_CONTROLLER_H

#include "Grid.h"
#include "HybridDPRLSolver.h"
#include <string>
#include <memory>

class CPSController {
public:
    CPSController(const json& initial_config);
    void run_simulation();

private:
    Grid master_grid;
    Position agent_position;
    std::unique_ptr<HybridDPRLSolver> solver;
    std::string agent_id = "agent_01";

    void write_input_file(int timestep);
    Direction read_output_file();
};

#endif // ENMOD_CPS_CONTROLLER_H