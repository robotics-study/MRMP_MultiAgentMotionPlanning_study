import {ReactNode, useState} from "react";
import cn from "../libs/cn";
import Modal from "./Modal";

interface CanvasFigureProps {
    label: string;
    // 인라인 썸네일로 렌더할 캔버스.
    children: ReactNode;
    // 모달용 확대 캔버스. 생략하면 children을 큰 컨테이너에 그대로 재사용한다.
    modal?: ReactNode;
    // 외곽 래퍼(레이아웃 폭 등).
    className?: string;
    // 모달 내부 캔버스 컨테이너 크기.
    bodyClassName?: string;
    // 고정 픽셀 크기 캔버스(예: Konva)는 래퍼를 콘텐츠에 밀착시켜 zoom 버튼이 캔버스 모서리에 붙게 한다.
    tight?: boolean;
}

// 모달 확대 캔버스의 픽셀 크기. 모달 헤더/패딩과 캔버스 아래 컨트롤·레전드 몫을 빼고
// 뷰포트에 맞춰 최대한 크게 잡는다. 모달 인스턴스는 열릴 때 fresh mount 되므로 호출 시점 값으로 충분하다.
export const modalCanvasSize = (aspect = 1) => {
    // 모바일은 모달이 풀스크린 + 얇은 패딩이라 여백 몫이 훨씬 작다.
    const chrome = window.innerWidth <= 640 ? 36 : 120;
    const maxW = Math.min(window.innerWidth - chrome, 1300);
    const maxH = window.innerHeight * 0.94 - 190;
    const width = Math.max(300, Math.round(Math.min(maxW, maxH * aspect)));
    return {width, height: Math.round(width / aspect)};
};

// 고정 픽셀 크기 Konva figure를 모달에서 크게 채우기 위한 배율. 네이티브 (w, h)를
// 뷰포트에 맞춘 modalCanvasSize 폭에 대응시켜 배율을 구한다. Scene은 이 값을 Stage
// 크기와 Layer scaleX/scaleY에 곱해 벡터 그대로 선명하게 확대한다(래스터 CSS 확대와
// 달리 흐려지지 않는다). cap으로 과확대를 막는다.
export const modalScale = (w: number, h: number, cap = 2.6) =>
    Math.min(cap, modalCanvasSize(w / h).width / w);

const ZoomIcon = () => (
    <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor"
         strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
        <circle cx="11" cy="11" r="7"/>
        <path d="m21 21-4.3-4.3M11 8v6M8 11h6"/>
    </svg>
);

const CanvasFigure = ({label, children, modal, className, bodyClassName, tight}: CanvasFigureProps) => {
    const [open, setOpen] = useState(false);
    return (
        <div className={cn("flex flex-col items-center justify-center p-3 gap-1", className)}>
            <div className={cn("relative group", tight ? "w-fit max-w-full" : "w-full")}>
                {children}
                <button type="button" className="canvas-zoom-btn" onClick={() => setOpen(true)}
                        aria-label={`Expand ${label}`}>
                    <ZoomIcon/>
                </button>
            </div>
            <span className="text-xs text-muted">{label}</span>
            <Modal open={open} onClose={() => setOpen(false)} title={label}>
                <div className={cn("mx-auto", bodyClassName ?? "w-[min(86vw,78vh,1000px)] aspect-square")}>
                    {/* 열렸을 때만 확대 인스턴스를 마운트한다 (불필요한 WebGL 컨텍스트 방지). */}
                    {open ? (modal ?? children) : null}
                </div>
            </Modal>
        </div>
    );
};

export default CanvasFigure;
