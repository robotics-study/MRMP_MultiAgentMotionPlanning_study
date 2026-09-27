import {useEffect, useMemo, useState} from "react";

export type Theme = "light" | "dark";

// 캔버스(Konva/Babylon)는 CSS 변수를 못 읽으므로 현재 테마를 JS로 직접 판정한다.
// 우선순위: 명시적 data-theme 속성 > 시스템 prefers-color-scheme.
function resolveTheme(): Theme {
    const attr = document.documentElement.getAttribute("data-theme");
    if (attr === "light" || attr === "dark") return attr;
    return window.matchMedia?.("(prefers-color-scheme: dark)").matches ? "dark" : "light";
}

export function useTheme(): Theme {
    const [theme, setTheme] = useState<Theme>(resolveTheme);
    useEffect(() => {
        const update = () => setTheme(resolveTheme());
        const media = window.matchMedia("(prefers-color-scheme: dark)");
        media.addEventListener("change", update);
        // 향후 테마 토글이 data-theme를 바꿀 때도 즉시 반영한다.
        const observer = new MutationObserver(update);
        observer.observe(document.documentElement, {attributes: true, attributeFilter: ["data-theme"]});
        return () => {
            media.removeEventListener("change", update);
            observer.disconnect();
        };
    }, []);
    return theme;
}

export interface CanvasColors {
    text: string;
    muted: string;
    border: string;
    surface: string;
    bg: string;
    accent: string;
    accent2: string;
}

// 캔버스에서 쓸 실제 색을 CSS 변수(단일 진실원본)에서 읽어온다. 테마가 바뀌면 재계산된다.
export function useCanvasColors(): CanvasColors {
    const theme = useTheme();
    return useMemo(() => {
        const style = getComputedStyle(document.documentElement);
        const read = (name: string, fallback: string) => style.getPropertyValue(name).trim() || fallback;
        return {
            text: read("--text", "#0f172a"),
            muted: read("--muted", "#5b6b82"),
            border: read("--border", "#e2e8f0"),
            surface: read("--surface", "#f8fafc"),
            bg: read("--bg", "#ffffff"),
            accent: read("--accent", "#6366f1"),
            accent2: read("--accent-2", "#06b6d4"),
        };
    }, [theme]);
}
