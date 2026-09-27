import CodeHighlight from "./CodeHighlight";

// 알고리즘 절차를 산문 번호 목록 대신 수도코드 블록으로 보여 준다.
// 수도코드는 언어 중립이라 번역하지 않는다. 키워드/주석 색은 python 토크나이저를 재사용한다.
const Pseudocode = ({code}: {code: string}) => (
    <pre className="border border-border rounded-xl overflow-x-auto text-[.82rem] leading-relaxed !mb-5">
        <code><CodeHighlight code={code} lang="python"/></code>
    </pre>
)

export default Pseudocode
