import {Layer, Line, Rect, Stage, Text} from "react-konva";
import CanvasFigure from "../../CanvasFigure";
import {useCanvasColors} from "../../../libs/useTheme";
import {useTr} from "../../../libs/i18n";

// CBS의 개념 그림: high-level constraint tree. 뿌리는 각 agent를 따로 계획한 해다.
// 충돌이 있으면 그 충돌 하나를 두 갈래로 나눠, 한쪽은 agent i를, 다른쪽은 agent j를
// 그 (칸, 시각)에서 금지하는 제약을 더한다. 실제로 충돌한 agent만, 충돌한 지점에서만
// 결합하므로 트리는 충돌이 남은 가지에서만 자란다 (Sharon et al., 2015).
const W = 360;
const H = 300;

interface NodeSpec {
    x: number;
    y: number;
    w: number;
    h: number;
    title: string;
    sub: string;
    solved?: boolean;
}

const Scene = () => {
    const colors = useCanvasColors()
    const t = useTr()

    const root: NodeSpec = {
        x: W / 2 - 95, y: 14, w: 190, h: 46,
        title: t("root: each agent alone", "root: agent 각자 계획"),
        sub: t("conflict: 1 & 2 at (v, t=3)", "충돌: 1·2가 (v, t=3)"),
    }
    const left: NodeSpec = {
        x: 20, y: 130, w: 150, h: 52,
        title: t("agent 1 ⊘ (v, t=3)", "agent 1 ⊘ (v, t=3)"),
        sub: t("replanned · cost 12 · clear", "재계획 · 비용 12 · 충돌 없음"),
        solved: true,
    }
    const right: NodeSpec = {
        x: W - 170, y: 130, w: 150, h: 52,
        title: t("agent 2 ⊘ (v, t=3)", "agent 2 ⊘ (v, t=3)"),
        sub: t("replanned · cost 14 · new conflict", "재계획 · 비용 14 · 새 충돌"),
    }
    const rightChild: NodeSpec = {
        x: W - 170, y: 232, w: 150, h: 46,
        title: t("+ agent 2 ⊘ (u, t=5)", "+ agent 2 ⊘ (u, t=5)"),
        sub: t("cost 15 · continue…", "비용 15 · 계속…"),
    }

    const bottom = (n: NodeSpec): [number, number] => [n.x + n.w / 2, n.y + n.h]
    const top = (n: NodeSpec): [number, number] => [n.x + n.w / 2, n.y]

    const drawNode = (n: NodeSpec, key: string) => {
        const accent = n.solved ? colors.accent2 : colors.accent
        return (
            <>
                <Rect key={`${key}-r`} x={n.x} y={n.y} width={n.w} height={n.h}
                      cornerRadius={7} fill={colors.surface}
                      stroke={accent} strokeWidth={n.solved ? 2.4 : 1.4}/>
                <Text key={`${key}-t`} x={n.x + 8} y={n.y + 8} width={n.w - 16}
                      text={n.title} fontSize={12} fill={colors.text} fontStyle="bold"
                      align="center"/>
                <Text key={`${key}-s`} x={n.x + 8} y={n.y + 27} width={n.w - 16}
                      text={n.sub} fontSize={10.5} align="center"
                      fill={n.solved ? colors.accent2 : colors.muted}/>
            </>
        )
    }

    const edge = (from: NodeSpec, to: NodeSpec, label: string, key: string) => {
        const [fx, fy] = bottom(from)
        const [tx, ty] = top(to)
        return (
            <>
                <Line key={`${key}-l`} points={[fx, fy, tx, ty]}
                      stroke={colors.muted} strokeWidth={1.4}/>
                <Text key={`${key}-lb`} x={(fx + tx) / 2 - 44} y={(fy + ty) / 2 - 8}
                      width={88} align="center" text={label}
                      fontSize={10.5} fill={colors.muted}/>
            </>
        )
    }

    return (
        <div className="flex flex-col items-center gap-2">
            <Stage width={W} height={H}
                   className="bg-surface border border-border rounded-lg overflow-hidden">
                <Layer>
                    {edge(root, left, t("constrain 1", "1을 제약"), "e-l")}
                    {edge(root, right, t("constrain 2", "2를 제약"), "e-r")}
                    {edge(right, rightChild, t("next conflict", "다음 충돌"), "e-rc")}
                    {drawNode(root, "root")}
                    {drawNode(left, "left")}
                    {drawNode(right, "right")}
                    {drawNode(rightChild, "rc")}
                    <Text x={left.x - 4} y={left.y + left.h + 6} width={left.w + 8} align="center"
                          text={t("← solution (lowest cost, conflict-free)",
                              "← 해 (최소 비용, 충돌 없음)")}
                          fontSize={10.5} fill={colors.accent2} fontStyle="bold"/>
                </Layer>
            </Stage>
            <div className="text-xs text-muted text-center" style={{maxWidth: W}}>
                {t(
                    "one conflict splits into two constraints; the tree is searched best-first by cost and grows only where conflicts remain",
                    "충돌 하나가 제약 둘로 갈라지고, 트리는 비용 기준 best-first로 탐색되며 충돌이 남은 곳에서만 자란다",
                )}
            </div>
        </div>
    )
}

const MultiConstraintTree = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "CBS's high-level constraint tree: the root plans every agent independently; each conflict branches into two nodes, one forbidding agent i and the other agent j from the contested cell-time. Only conflicting agents ever get coupled.",
            "CBS의 high-level constraint tree. root는 모든 agent를 독립으로 계획하고, 충돌마다 두 갈래로 분기해 한쪽은 agent i를, 다른쪽은 agent j를 문제의 칸·시각에서 금지한다. 실제로 충돌한 agent만 결합된다.",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<Scene/>}
    >
        <Scene/>
    </CanvasFigure>
}

export default MultiConstraintTree
