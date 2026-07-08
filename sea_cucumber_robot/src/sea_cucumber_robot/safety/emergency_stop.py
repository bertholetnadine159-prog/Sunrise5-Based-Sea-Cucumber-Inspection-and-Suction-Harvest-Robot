from __future__ import annotations

from pathlib import Path


class EmergencyStop:
    def __init__(self, flag_file: str | Path) -> None:
        self.flag_file = Path(flag_file)
        self._latched_reason: str | None = None

    @property
    def latched(self) -> bool:
        return self._latched_reason is not None or self.flag_file.exists()

    @property
    def reason(self) -> str:
        if self._latched_reason:
            return self._latched_reason
        if self.flag_file.exists():
            return f"Emergency flag file exists: {self.flag_file}"
        return ""

    def trigger(self, reason: str) -> None:
        self._latched_reason = reason
        self.flag_file.parent.mkdir(parents=True, exist_ok=True)
        self.flag_file.write_text(reason + "\n", encoding="utf-8")

    def clear_manual(self) -> None:
        if self.flag_file.exists():
            self.flag_file.unlink()
        self._latched_reason = None
