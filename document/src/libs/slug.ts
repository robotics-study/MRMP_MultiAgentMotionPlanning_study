// 제목 텍스트 → URL 앵커(id) 변환. TOC가 헤딩에 부여하는 id와 검색 인덱스의 앵커가
// 같은 규칙을 공유해야 검색 결과 딥링크(#anchor)가 실제 헤딩으로 스크롤된다.
export function slugify(text: string): string {
    return text
        .trim()
        .toLowerCase()
        // 영숫자·비-ASCII 문자(한글 등)·공백·하이픈만 남긴다. ASCII 문장부호/기호는 제거.
        // (\p{L} 정규식은 es5 타깃에서 못 써 유니코드 범위로 대체.)
        .replace(/[^0-9a-zÀ-￿\s-]/g, "")
        .replace(/[\s_]+/g, "-")             // 공백·언더스코어 → 하이픈
        .replace(/-+/g, "-")                  // 연속 하이픈 축약
        .replace(/^-|-$/g, "")                // 양끝 하이픈 제거
}
