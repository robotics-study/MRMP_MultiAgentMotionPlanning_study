import {ReactNode, useEffect} from "react";
import {createPortal} from "react-dom";

interface ModalProps {
    open: boolean;
    onClose: () => void;
    title?: string;
    children: ReactNode;
}

const Modal = ({open, onClose, title, children}: ModalProps) => {
    useEffect(() => {
        if (!open) return;
        const onKey = (e: KeyboardEvent) => {
            if (e.key === "Escape") onClose();
        };
        document.addEventListener("keydown", onKey);
        // 모달 열림 동안 배경 스크롤을 잠근다.
        const prevOverflow = document.body.style.overflow;
        document.body.style.overflow = "hidden";
        return () => {
            document.removeEventListener("keydown", onKey);
            document.body.style.overflow = prevOverflow;
        };
    }, [open, onClose]);

    if (!open) return null;
    return createPortal(
        <div className="canvas-modal-backdrop" onClick={onClose}>
            <div className="canvas-modal" role="dialog" aria-modal="true" aria-label={title}
                 onClick={(e) => e.stopPropagation()}>
                <div className="canvas-modal-head">
                    <span className="canvas-modal-title">{title}</span>
                    <button type="button" className="canvas-modal-close" onClick={onClose} aria-label="Close">
                        <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor"
                             strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                            <path d="M18 6 6 18M6 6l12 12"/>
                        </svg>
                    </button>
                </div>
                <div className="canvas-modal-body">{children}</div>
            </div>
        </div>,
        document.body,
    );
};

export default Modal;
