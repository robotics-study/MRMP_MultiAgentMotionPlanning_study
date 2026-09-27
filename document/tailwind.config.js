/** @type {import('tailwindcss').Config} */
module.exports = {
    content: ["./index.html", "./src/**/*.{js,ts,jsx,tsx}"], // Tailwind가 파일을 스캔할 경로
    theme: {
        extend: {
            // 시맨틱 색 토큰 → CSS 변수. 테마 전환 시 값이 바뀌므로 클래스는 그대로 둔다.
            colors: {
                bg: "var(--bg)",
                surface: "var(--surface)",
                "surface-2": "var(--surface-2)",
                text: "var(--text)",
                muted: "var(--muted)",
                border: "var(--border)",
                accent: "var(--accent)",
                accent2: "var(--accent-2)",
            },
            fontFamily: {
                sans: ["Inter", "system-ui", "-apple-system", "Segoe UI", "Roboto",
                    "Apple SD Gothic Neo", "Noto Sans KR", "Malgun Gothic", "sans-serif"],
            },
            boxShadow: {
                card: "var(--shadow)",
            },
        },
    },
    plugins: [],
};
