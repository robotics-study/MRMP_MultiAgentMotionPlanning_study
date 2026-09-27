#include <filesystem>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "mrmp/core/params.hpp"
#include "mrmp/core/trace.hpp"
#include "mrmp/core/types.hpp"
#include "mrmp/core/yaml.hpp"
#include "mrmp/maps/loader.hpp"
#include "mrmp/maps/occupancy_grid.hpp"
#include "mrmp/maps/pgm.hpp"
#include "test_util.hpp"

using namespace mrmp;

// --- YAML parser on real repo files ------------------------------------------

TEST(Yaml, ParsesRealMapAndScenarioFlowSequences) {
  core::YamlNode m = core::parse_yaml_file(test::repo_path("maps/grid/maze01.yaml"));
  EXPECT_EQ(m.at("type").as_string(), "occupancy_grid");
  EXPECT_DOUBLE_EQ(m.at("resolution").as_double(), 0.5);
  EXPECT_DOUBLE_EQ(m.at("origin").seq.at(0).as_double(), 0.0);

  core::YamlNode s = core::parse_yaml_file(test::repo_path("maps/scenarios/maze01_two.yaml"));
  // agents is a block sequence of maps; each start/goal is a flow sequence.
  const core::YamlNode& agents = s.at("agents");
  ASSERT_TRUE(agents.is_seq());
  ASSERT_EQ(agents.seq.size(), 2u);
  EXPECT_DOUBLE_EQ(agents.seq[0].at("start").seq.at(0).as_double(), 0.75);
  EXPECT_DOUBLE_EQ(agents.seq[1].at("goal").seq.at(0).as_double(), 0.75);
}

TEST(Yaml, EmptyFlowSequenceIsEmpty) {
  std::string p = test::write_temp("p.yaml", "algorithm: x\ncategory: mapf\nparams: []\n");
  core::YamlNode root = core::parse_yaml_file(p);
  EXPECT_TRUE(root.at("params").is_seq());
  EXPECT_TRUE(root.at("params").seq.empty());
}

// --- ParamSet validation ------------------------------------------------------

TEST(Params, LoadsDeclaredDefaults) {
  std::string p = test::write_temp(
      "ok.yaml",
      "algorithm: x\ncategory: mapf\nparams:\n"
      "  - name: weight\n    type: float\n    default: 1.0\n    description: w\n"
      "  - name: n\n    type: int\n    default: 3\n    min: 0\n    max: 10\n    description: n\n");
  auto ps = core::ParamSet::from_yaml(p);
  EXPECT_EQ(ps.algorithm(), "x");
  EXPECT_EQ(ps.category(), "mapf");
  EXPECT_DOUBLE_EQ(ps.get_float("weight"), 1.0);
  EXPECT_EQ(ps.get_int("n"), 3);
}

TEST(Params, UnknownCategoryThrows) {
  std::string p = test::write_temp("bad.yaml", "algorithm: x\ncategory: global_planning\nparams: []\n");
  EXPECT_THROW(core::ParamSet::from_yaml(p), std::runtime_error);
}

TEST(Params, OutOfRangeDefaultThrows) {
  std::string p = test::write_temp("bad.yaml",
                                   "algorithm: x\ncategory: mapf\nparams:\n"
                                   "  - name: weight\n    type: float\n"
                                   "    default: 9.0\n    min: 1.0\n    max: 5.0\n"
                                   "    description: w\n");
  EXPECT_THROW(core::ParamSet::from_yaml(p), std::runtime_error);
}

TEST(Params, WrongTypeAccessThrows) {
  std::string p = test::write_temp("ok.yaml",
                                   "algorithm: x\ncategory: mapf\nparams:\n"
                                   "  - name: n\n    type: int\n    default: 3\n    description: n\n");
  auto ps = core::ParamSet::from_yaml(p);
  EXPECT_EQ(ps.get_int("n"), 3);
  EXPECT_THROW(ps.get_float("n"), std::runtime_error);  // declared int
}

// --- Grid geometry + MAPF move set --------------------------------------------

TEST(Grid, WorldCellRoundTrip) {
  auto g = test::make_grid({"....", "....", "....", "...."});
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      core::Cell cell{r, c};
      core::Point w = g.cell_to_world(cell);
      core::Cell back = g.world_to_cell(w.x, w.y);
      EXPECT_EQ(back.row, r);
      EXPECT_EQ(back.col, c);
    }
  }
}

TEST(Grid, OccupancyThresholding) {
  // occ = 1 - pixel/255: black(0)->occupied, white(255)->free, mid(128)->unknown->blocked.
  maps::PgmImage img;
  img.width = 3;
  img.height = 1;
  img.maxval = 255;
  img.pixels = {0, 128, 255};
  auto g = maps::OccupancyGrid2D::from_image(img, 0.5, 0.0, 0.0, 0.65, 0.196);
  EXPECT_FALSE(g.is_free(0, 0));  // occupied
  EXPECT_FALSE(g.is_free(0, 1));  // unknown treated as blocked
  EXPECT_TRUE(g.is_free(0, 2));   // free
}

TEST(Grid, NeighborsFixedOrderWithWait) {
  // Interior cell: up/down/left/right in fixed order, then the wait self-loop —
  // all cost 1.0 (one time step per action). Order is a cross-language contract.
  auto g = test::make_grid({"...", "...", "..."});
  auto nbrs = g.neighbors(core::Cell{1, 1});
  ASSERT_EQ(nbrs.size(), 5u);
  // Whole condition in parentheses: the braced Cell initializer's comma would
  // otherwise split the EXPECT macro's arguments.
  EXPECT_TRUE((nbrs[0].first == core::Cell{0, 1}) && nbrs[0].second == 1.0);
  EXPECT_TRUE((nbrs[1].first == core::Cell{2, 1}) && nbrs[1].second == 1.0);
  EXPECT_TRUE((nbrs[2].first == core::Cell{1, 0}) && nbrs[2].second == 1.0);
  EXPECT_TRUE((nbrs[3].first == core::Cell{1, 2}) && nbrs[3].second == 1.0);
  EXPECT_TRUE((nbrs[4].first == core::Cell{1, 1}) && nbrs[4].second == 1.0);  // wait
}

TEST(Grid, WaitAlwaysAvailableEvenWhenBoxedIn) {
  // A fully boxed-in cell still has the wait action — an agent can always stand
  // still; infeasibility comes from no passable moves, never from no actions.
  auto g = test::make_grid({".#.", "#.#", ".#."});
  auto nbrs = g.neighbors(core::Cell{1, 1});
  ASSERT_EQ(nbrs.size(), 1u);
  EXPECT_TRUE((nbrs[0].first == core::Cell{1, 1}));
}

TEST(Grid, HeuristicIsManhattan) {
  auto g = test::make_grid({"...", "...", "..."});
  // Diagonal distance is Manhattan (2), not Euclidean — diagonals are not moves.
  EXPECT_DOUBLE_EQ(g.heuristic(core::Cell{0, 0}, core::Cell{1, 1}), 2.0);
}

TEST(Grid, Capabilities) {
  auto g = test::make_grid({"..", ".."});
  EXPECT_TRUE(g.supports(core::Capability::DISCRETE_SPACE));
}

// --- Trace --------------------------------------------------------------------

TEST(Trace, ExactWireFormatMatchesPython) {
  // Byte-exact contract with the Python recorder (python/mrmp/core/trace.py emits
  // the same JSON modulo float formatting: json writes 5.0 where write_num writes
  // 5 — parsed values are equal). Params arrive unsorted here; the recorder must
  // emit them sorted, exactly like Python's dict(sorted(...)).
  std::ostringstream os;
  core::TraceRecorder rec(os);
  rec.planning_started("prioritized_astar", "maps/grid/maze01.yaml",
                       {{"zeta", core::ParamValue(2.0)}, {"alpha", core::ParamValue(1)}});
  rec.node_expanded(core::Cell{3, 4}, 5.0, 0, 5);
  rec.path_found(std::vector<core::Cell>{{3, 4}, {3, 4}}, 0);
  rec.conflict_found(core::ConflictKind::Vertex, core::Cell{2, 2}, 1, {0, 1});
  rec.constraint_added(1, core::ConflictKind::Vertex, core::Cell{2, 2}, 1);
  rec.planning_finished(true, {{"expanded_nodes", 3.0}, {"sum_of_costs", 4.0}});

  const std::vector<std::string> expected = {
      R"({"seq":0,"event":"planning_started","algorithm":"prioritized_astar","map":"maps/grid/maze01.yaml","params":{"alpha":1,"zeta":2}})",
      R"({"seq":1,"event":"node_expanded","state":[3,4],"cost":5,"agent":0,"t":5})",
      R"({"seq":2,"event":"path_found","path":[[3,4],[3,4]],"agent":0})",
      R"({"seq":3,"event":"conflict_found","kind":"vertex","cell":[2,2],"t":1,"agents":[0,1]})",
      R"({"seq":4,"event":"constraint_added","agent":1,"kind":"vertex","cell":[2,2],"t":1})",
      R"({"seq":5,"event":"planning_finished","success":true,"metrics":{"expanded_nodes":3,"sum_of_costs":4}})",
  };
  std::istringstream in(os.str());
  std::string line;
  size_t i = 0;
  while (std::getline(in, line)) {
    ASSERT_LT(i, expected.size()) << "extra event line: " << line;
    // An MRMP trace carries no wall-clock time — only seq + the discrete step t.
    EXPECT_EQ(line, expected[i]);
    ++i;
  }
  EXPECT_EQ(i, expected.size());
}

TEST(Trace, CellStateSerializesAsIntegerArrayWithAgentAndStep) {
  std::ostringstream os;
  core::TraceRecorder rec(os);
  rec.node_expanded(core::Cell{3, 7}, std::nullopt, 1, 2);
  EXPECT_NE(os.str().find("\"state\":[3,7]"), std::string::npos);
  EXPECT_NE(os.str().find("\"agent\":1"), std::string::npos);
  EXPECT_NE(os.str().find("\"t\":2"), std::string::npos);
}

TEST(Trace, JointStateIsFlattenedIntList) {
  // Joint-space search expands one node per joint state: every agent's cell
  // flattened to [r0,c0,r1,c1,...], no agent field (the state carries all).
  std::ostringstream os;
  core::TraceRecorder rec(os);
  rec.node_expanded(core::flatten({core::Cell{3, 4}, core::Cell{2, 2}}));
  EXPECT_NE(os.str().find("\"state\":[3,4,2,2]"), std::string::npos);
}

TEST(Trace, EdgeConflictCarriesBothCells) {
  std::ostringstream os;
  core::TraceRecorder rec(os);
  rec.conflict_found(core::ConflictKind::Edge, core::Cell{2, 2}, 3, {1, 0}, core::Cell{2, 3});
  const std::string s = os.str();
  EXPECT_NE(s.find("\"kind\":\"edge\""), std::string::npos);
  EXPECT_NE(s.find("\"cell\":[2,2]"), std::string::npos);
  EXPECT_NE(s.find("\"t\":3"), std::string::npos);
  EXPECT_NE(s.find("\"agents\":[1,0]"), std::string::npos);
  EXPECT_NE(s.find("\"to\":[2,3]"), std::string::npos);
}

// --- PGM reader ---------------------------------------------------------------

TEST(Pgm, P2AsciiWithCommentAndBlockLayout) {
  std::string p = test::write_temp("a.pgm", "P2\n# a comment\n3 2\n255\n0 255 0\n255 0 255\n");
  maps::PgmImage img = maps::load_pgm(p);
  EXPECT_EQ(img.width, 3);
  EXPECT_EQ(img.height, 2);
  EXPECT_EQ(img.pixels, (std::vector<int>{0, 255, 0, 255, 0, 255}));
}

TEST(Pgm, P5BinaryMatchesAsciiValues) {
  std::string content = "P5\n3 2\n255\n";
  for (int v : {0, 255, 128, 64, 200, 10}) content += static_cast<char>(v);
  std::string p = test::write_temp("b.pgm", content);
  maps::PgmImage img = maps::load_pgm(p);
  EXPECT_EQ(img.pixels, (std::vector<int>{0, 255, 128, 64, 200, 10}));
}

TEST(Pgm, P5SixteenBitBigEndian) {
  // Values above 255 need two big-endian bytes per sample (Netpbm spec).
  std::vector<int> values{0, 65535, 300, 1000};
  std::string content = "P5\n2 2\n65535\n";
  for (int v : values) {
    content += static_cast<char>((v >> 8) & 0xFF);
    content += static_cast<char>(v & 0xFF);
  }
  std::string p = test::write_temp("c.pgm", content);
  maps::PgmImage img = maps::load_pgm(p);
  EXPECT_EQ(img.pixels, values);
}

TEST(Pgm, TruncatedAsciiThrows) {
  std::string p = test::write_temp("d.pgm", "P2\n3 3\n255\n0 255 0\n");  // 3 of 9 values
  EXPECT_THROW(maps::load_pgm(p), std::runtime_error);
}

TEST(Pgm, UnsupportedMagicThrows) {
  std::string p = test::write_temp("e.pgm", "P3\n1 1\n255\n0 0 0\n");  // P3 = color PPM
  EXPECT_THROW(maps::load_pgm(p), std::runtime_error);
}

// --- Map / scenario loaders ---------------------------------------------------

TEST(Loader, LoadsRealOccupancyGrid) {
  auto map = maps::load_map(test::repo_path("maps/grid/maze01.yaml"));
  ASSERT_NE(map, nullptr);
  EXPECT_TRUE(map->supports(core::Capability::DISCRETE_SPACE));
}

TEST(Loader, UnsupportedMapTypeThrows) {
  std::string p = test::write_temp("g.yaml", "type: graph\nnodes: []\nedges: []\n");
  EXPECT_THROW(maps::load_map(p), std::runtime_error);
}

TEST(Loader, ScenarioResolvesMapPathAndAgents) {
  maps::Scenario sc = maps::load_scenario(test::repo_path("maps/scenarios/maze01_two.yaml"));
  std::filesystem::path expected =
      std::filesystem::weakly_canonical(test::repo_path("maps/grid/maze01.yaml"));
  EXPECT_EQ(std::filesystem::path(sc.map_path), expected);
  ASSERT_EQ(sc.agents.size(), 2u);
  // World coords arrive untouched — the demo driver converts to cells.
  EXPECT_DOUBLE_EQ(sc.agents[0].start.x, 0.75);
  EXPECT_DOUBLE_EQ(sc.agents[0].goal.y, 7.25);
}

TEST(Loader, SingleRobotScenarioRejected) {
  // This repo is multi-agent only: a start/goal scenario without `agents:` is the
  // sibling single-robot format, not something load_scenario may silently accept.
  std::string p = test::write_temp("s.yaml", "map: ../grid/maze01.yaml\nstart: [0.75, 1.25]\n");
  EXPECT_THROW(maps::load_scenario(p), std::runtime_error);
}
