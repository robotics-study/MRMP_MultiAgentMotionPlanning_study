// dRRT* demo — assembly only (see demos/demo_common.hpp). Continuous planner: the
// scenario's world-coord Points stay raw and each disc's radius rides along;
// planning_started declares coords=world plus the radii.
#include "demo_common.hpp"
#include "mrmp/sampling/drrt_star.hpp"

int main(int argc, char** argv) {
  try {
    demo::Args args = demo::parse_args(argc, argv);
    auto params = mrmp::core::ParamSet::from_yaml(args.params);
    mrmp::sampling::DrrtStar planner(params);
    return demo::run_continuous(args, params, planner);
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
}
