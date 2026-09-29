import {T} from "../libs/i18n";

const LICENSE_URL = "https://github.com/robotics-study/mrmp_introduction/blob/main/LICENSE"

const Footer = () => (
    <footer className="site-footer">
        <T
            en={<p>
                © 2026 robotics-study ·{" "}
                <a href={LICENSE_URL} target="_blank" rel="noopener noreferrer">MIT License</a>
                {" "}· Unofficial study notes
            </p>}
            ko={<p>
                © 2026 robotics-study ·{" "}
                <a href={LICENSE_URL} target="_blank" rel="noopener noreferrer">MIT License</a>
                {" "}· 비공식 학습 노트
            </p>}
        />
    </footer>
)

export default Footer
