import {ReactNode, useEffect, useState} from "react";
import cn from "../libs/cn";
import CodeHighlight, {CodeLang} from "./CodeHighlight";

// 저장소의 실제 소스를 언어 토글로 보여 주는 코드 패널. 코드 문자열은 vite의 ?raw
// import로 저장소 파일에서 직접 가져오므로 문서와 구현이 어긋나지 않는다.
export interface CodeFile {
    name: string;    // 표시용 파일 경로 (repo 상대)
    code: string;
    href: string;    // GitHub 원본 링크
}

export interface CodeTab {
    label: string;   // "python" | "c++"
    lang: CodeLang;
    files: CodeFile[];
}

const CopyIcon = () => (
    <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor"
         strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
        <rect x="9" y="9" width="11" height="11" rx="2"/>
        <path d="M5 15V5a2 2 0 0 1 2-2h10"/>
    </svg>
)

const CheckIcon = () => (
    <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor"
         strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
        <path d="M20 6 9 17l-5-5"/>
    </svg>
)

const ExtIcon = () => (
    <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor"
         strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true">
        <path d="M7 17 17 7M8 7h9v9"/>
    </svg>
)

const CopyButton = ({code}: {code: string}) => {
    const [copied, setCopied] = useState(false)
    useEffect(() => {
        if (!copied) return
        const id = window.setTimeout(() => setCopied(false), 1600)
        return () => window.clearTimeout(id)
    }, [copied])
    return (
        <button type="button"
                className="inline-flex items-center gap-1 hover:text-[var(--text)]"
                aria-label={copied ? "Copied" : "Copy code"}
                onClick={async () => {
                    try {
                        await navigator.clipboard.writeText(code)
                        setCopied(true)
                    } catch {
                        // 클립보드 접근 불가(비보안 컨텍스트 등)면 조용히 무시한다.
                    }
                }}>
            {copied ? <CheckIcon/> : <CopyIcon/>}
        </button>
    )
}

const CodeTabs = ({tabs, caption}: {tabs: CodeTab[]; caption?: ReactNode}) => {
    const [active, setActive] = useState(0)
    const tab = tabs[active]
    return (
        <div className="my-5">
            <div className="flex items-center gap-1.5 mb-2 text-xs">
                {tabs.map((item, i) => (
                    <button key={item.label} type="button" onClick={() => setActive(i)}
                            className={cn(
                                "px-2.5 py-1 rounded border font-mono",
                                i === active
                                    ? "border-[var(--accent)] text-[var(--accent)] font-semibold"
                                    : "border-border text-muted hover:bg-surface",
                            )}>
                        {item.label}
                    </button>
                ))}
            </div>
            {tab.files.map((file) => (
                <div key={file.name} className="border border-border rounded-xl overflow-hidden mb-3">
                    <div className="flex items-center gap-2 px-3 py-1.5 text-xs text-muted bg-surface border-b border-border">
                        <span className="font-mono">{file.name}</span>
                        <span className="ml-auto flex items-center gap-2.5">
                            <CopyButton code={file.code}/>
                            <a href={file.href} target="_blank" rel="noopener noreferrer"
                               className="inline-flex items-center gap-1 hover:text-[var(--text)]">
                                GitHub<ExtIcon/>
                            </a>
                        </span>
                    </div>
                    <pre className="!my-0 !rounded-none max-h-[420px] overflow-auto text-[.8rem] leading-relaxed">
                        <code><CodeHighlight code={file.code} lang={tab.lang}/></code>
                    </pre>
                </div>
            ))}
            {caption && <div className="text-xs text-muted text-center">{caption}</div>}
        </div>
    )
}

export default CodeTabs
