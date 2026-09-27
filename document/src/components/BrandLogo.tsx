// robotics-study.github.io와 동일한 관절(joint) 모티프 로고 — indigo→cyan 그라디언트.
// Topbar와 랜딩이 공유한다. 같은 페이지에 두 개가 동시에 렌더될 수 있어
// 그라디언트 id를 인스턴스마다 달리 받는다.
const BrandLogo = ({size = 26, gradId = "navLogo"}: { size?: number; gradId?: string }) => (
    <svg className="logo" width={size} height={size} viewBox="0 0 24 24" fill="none" aria-hidden="true">
        <defs>
            <linearGradient id={gradId} x1="0" y1="1" x2="1" y2="0">
                <stop stopColor="#6366f1"/>
                <stop offset="1" stopColor="#06b6d4"/>
            </linearGradient>
        </defs>
        <path d="M4 19V11h8V5h8" stroke={`url(#${gradId})`} strokeWidth="2.2" strokeLinecap="round"
              strokeLinejoin="round"/>
        <circle cx="4" cy="19" r="2.6" fill={`url(#${gradId})`}/>
        <circle cx="20" cy="5" r="2.6" fill={`url(#${gradId})`}/>
    </svg>
)

export default BrandLogo
