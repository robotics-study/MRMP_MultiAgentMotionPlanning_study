// db-CBS demo — assembly only (see demos/demo_common.hpp, run_velocity).
#include "demo_common.hpp"
#include "mrmp/kinodynamic/db_cbs.hpp"

int main(int argc, char** argv) {
  try {
    demo::Args args = demo::parse_args(argc, argv);
    auto params = mrmp::core::ParamSet::from_yaml(args.params);
    mrmp::kinodynamic::DbCbs planner(params);
    return demo::run_velocity(args, params, planner);
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
}
