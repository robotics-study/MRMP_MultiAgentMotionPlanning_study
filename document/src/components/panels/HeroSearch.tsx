import {lazy, Suspense} from "react";
import {useTr} from "../../libs/i18n";

// 히어로의 라이브 피겨는 lazy import로 분리한다 — 첫 페인트를 막지 않도록 캔버스 코드를
// 초기 번들 밖으로 뺀다. 정적 GIF 대신 "이 사이트의 페이지가 이런 식으로 동작한다"를 첫
// 화면에서 바로 만지게 한다.
const SpaceTimeConflict = lazy(() => import("./intro/SpaceTimeConflict"));

const HeroSearch = () => {
    const t = useTr()
    return (
        <div className="hero-3d">
            <div className="px-4 py-7">
                <Suspense fallback={
                    <div style={{minHeight: 380}}
                         className="flex items-center justify-center text-sm text-muted">
                        {t("loading live demo…", "라이브 데모 로딩 중…")}
                    </div>
                }>
                    <SpaceTimeConflict/>
                </Suspense>
            </div>
        </div>
    )
}

export default HeroSearch
