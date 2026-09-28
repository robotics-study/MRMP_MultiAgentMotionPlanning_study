// Joint-space A* demo — assembly only (see demos/demo_common.hpp).
#include "demo_common.hpp"
#include "mrmp/search/joint_astar.hpp"

int main(int argc, char** argv) {
  try {
    demo::Args args = demo::parse_args(argc, argv);
    auto params = mrmp::core::ParamSet::from_yaml(args.params);
    mrmp::search::JointAStar planner(params);
    return demo::run(args, params, planner);
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
}
