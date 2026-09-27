import {ReactNode} from "react";
import {InlineMath} from "./Tex";

// 수식 바로 아래에 붙는 항 설명. 모든 기호를 그 자리에서 정의해, 다른 페이지나
// 앞 섹션으로 돌아가지 않고 수식만 보고 읽히게 한다 (이전 페이지에서 정의한
// 기호도 다시 적는다).
interface TermsProps {
    items: Array<[string, ReactNode]>;
}

const Terms = ({items}: TermsProps) => (
    <div className="mx-auto w-fit max-w-full -mt-1 mb-5 px-4 py-2.5 rounded-lg bg-surface border border-border
                    grid grid-cols-[auto_1fr] gap-x-4 gap-y-1 text-[0.875rem] leading-relaxed">
        {items.map(([sym, desc], i) => (
            <div key={i} className="contents">
                <span className="justify-self-end whitespace-nowrap text-[var(--accent)]">
                    <InlineMath math={sym}/>
                </span>
                <span className="text-muted">{desc}</span>
            </div>
        ))}
    </div>
)

export default Terms
