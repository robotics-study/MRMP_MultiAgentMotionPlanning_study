// 웹 라이브 TS 엔진과 저장소 python demo 의 정밀 대조.
// public/data 의 py trace 에서 (파라미터, 에이전트 작업)을 읽어 같은 입력으로 TS 엔진을
// 돌리고 결과를 비교한다. MRMP 는 모든 계획이 결정론적이라(고정 이웃 순서·tie-break)
// expanded_nodes 와 sum_of_costs 모두 exact 일 수 있다 — 허용 오차가 필요한 알고리즘만
// metricKeys.tol 를 둔다. 실행: node scripts/check-engine-parity.mjs (esbuild 로 번들).
import {execFileSync} from "node:child_process";
import {gunzipSync} from "node:zlib";
import {mkdtempSync, readFileSync, rmSync} from "node:fs";
import {tmpdir} from "node:os";
import {dirname, join} from "node:path";
import {fileURLToPath} from "node:url";
import {createRequire} from "node:module";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const outDir = mkdtempSync(join(tmpdir(), "parity-"));
const bundle = join(outDir, "engines.cjs");
execFileSync(join(root, "node_modules", ".bin", "esbuild"), [
    join(root, "scripts", "parity-entry.ts"),
    "--bundle", "--format=cjs", "--platform=node", `--outfile=${bundle}`,
    "--loader:.ts=ts",
], {stdio: "pipe"});
const engines = createRequire(import.meta.url)(bundle);

const loadMap = (name) => engines.parseGridMap(
    JSON.parse(readFileSync(join(root, "public", "data", "maps", `${name}.json`), "utf-8")));
const loadTrace = (algo, name) =>
    gunzipSync(readFileSync(join(root, "public", "data", "traces", algo, `${name}.py.jsonl.gz`)))
        .toString("utf-8").trim().split("\n").map((l) => JSON.parse(l));

const finalOf = (events) => events[events.length - 1];

// (map, agents, params) → TS 엔진 실행 (events 반환). 알고리즘이 추가되면 여기에 러너를
// 등록한다 — agents 는 시나리오 yaml 의 cell 좌표값(데모가 world→cell 변환한 결과).
const RUNNERS = {
    prioritized_astar: (map, agents, params) => engines.runPrioritizedAStar(map, agents, params),
    joint_astar: (map, agents, params) => engines.runJointAStar(map, agents, params),
    cbs: (map, agents, params) => engines.runCbs(map, agents, params),
    ma_rrt_star: (map, agents, params) => engines.runMaRrtStar(map, agents, params),
};

// algo × scenario 조합. trace 파일은 시나리오 이름으로 키를 잡는다 (한 맵에 여러
// 시나리오가 살 수 있어서). agents: [[start, goal], ...] (cell [row, col]).
// metricKeys 가 없으면 sum_of_costs(expand 시 exact)와 expanded_nodes 를 비교한다.
const CHECKS = [
    {
        algo: "prioritized_astar",
        scenarios: [
            {map: "maze01", name: "maze01_two", agents: [[[17, 1], [5, 16]], [[17, 16], [5, 1]]]},
            {map: "open01", name: "open01_cross", agents: [[[10, 1], [10, 17]], [[1, 9], [18, 9]]]},
            {map: "open01", name: "open01_swap", agents: [[[10, 2], [10, 16]], [[10, 16], [10, 2]]]},
        ],
    },
    {
        algo: "joint_astar",
        scenarios: [
            {map: "maze01", name: "maze01_two", agents: [[[17, 1], [5, 16]], [[17, 16], [5, 1]]]},
            {map: "open01", name: "open01_cross", agents: [[[10, 1], [10, 17]], [[1, 9], [18, 9]]]},
            {map: "open01", name: "open01_swap", agents: [[[10, 2], [10, 16]], [[10, 16], [10, 2]]]},
        ],
    },
    {
        algo: "cbs",
        scenarios: [
            {map: "maze01", name: "maze01_two", agents: [[[17, 1], [5, 16]], [[17, 16], [5, 1]]]},
            {map: "open01", name: "open01_cross", agents: [[[10, 1], [10, 17]], [[1, 9], [18, 9]]]},
            {map: "open01", name: "open01_swap", agents: [[[10, 2], [10, 16]], [[10, 16], [10, 2]]]},
        ],
    },
    {
        algo: "ma_rrt_star",
        scenarios: [
            {map: "maze01", name: "maze01_two", agents: [[[17, 1], [5, 16]], [[17, 16], [5, 1]]]},
            {map: "open01", name: "open01_cross", agents: [[[10, 1], [10, 17]], [[1, 9], [18, 9]]]},
            {map: "open01", name: "open01_swap", agents: [[[10, 2], [10, 16]], [[10, 16], [10, 2]]]},
        ],
    },
];

let failures = 0;
for (const check of CHECKS) {
    for (const scenario of check.scenarios) {
        let events;
        try {
            events = loadTrace(check.algo, scenario.name);
        } catch {
            continue;   // 해당 시나리오의 trace 미탑재 — 검사 대상 아님
        }
        const expected = finalOf(events);
        const started = events[0];
        const got = finalOf(RUNNERS[check.algo](loadMap(scenario.map), scenario.agents,
            started.params ?? {}));

        const problems = [];
        if (Boolean(got.success) !== Boolean(expected.success)) {
            problems.push(`success ${got.success} != ${expected.success}`);
        }
        for (const key of check.metricKeys ?? [
            {key: "sum_of_costs", tol: 0}, {key: "expanded_nodes", tol: 0},
        ]) {
            const a = got.metrics?.[key.key];
            const b = expected.metrics?.[key.key];
            if (a === undefined || b === undefined || Math.abs(a - b) > key.tol) {
                problems.push(`${key.key} ${a} != ${b}`);
            }
        }
        const tag = `${check.algo} × ${scenario.name}`;
        if (problems.length) {
            failures++;
            console.log(`FAIL ${tag}: ${problems.join("; ")}`);
        } else {
            console.log(`ok   ${tag}`);
        }
    }
}
rmSync(outDir, {recursive: true, force: true});
if (failures) {
    console.error(`\n${failures} parity failure(s)`);
    process.exit(1);
}
console.log(`${CHECKS.length ? "\nall engines match the repository demos" : "no engine checks registered yet — nothing to compare"}`);
