# -*- coding: utf-8 -*-
from __future__ import annotations

try:
    from PyQt5 import sip as _qt_sip
except Exception:
    _qt_sip = None


def is_qt_object_alive(obj: object | None) -> bool:
    if obj is None:
        return False
    if _qt_sip is not None:
        try:
            if _qt_sip.isdeleted(obj):
                return False
        except Exception:
            pass
    return True


def set_text_if_changed(widget: object | None, text: object) -> bool:
    if not is_qt_object_alive(widget):
        return False
    value = str(text)
    try:
        current = widget.text() if hasattr(widget, "text") else None
        if current != value and hasattr(widget, "setText"):
            widget.setText(value)
        return True
    except RuntimeError:
        return False
    except Exception:
        return False


def set_stylesheet_if_changed(widget: object | None, stylesheet: str) -> bool:
    if not is_qt_object_alive(widget):
        return False
    try:
        current = widget.styleSheet() if hasattr(widget, "styleSheet") else None
        if current != stylesheet and hasattr(widget, "setStyleSheet"):
            widget.setStyleSheet(stylesheet)
        return True
    except RuntimeError:
        return False
    except Exception:
        return False


def set_enabled_if_changed(widget: object | None, enabled: bool) -> bool:
    if not is_qt_object_alive(widget):
        return False
    value = bool(enabled)
    try:
        current = widget.isEnabled() if hasattr(widget, "isEnabled") else None
        if current != value and hasattr(widget, "setEnabled"):
            widget.setEnabled(value)
        return True
    except RuntimeError:
        return False
    except Exception:
        return False


def set_visible_if_changed(widget: object | None, visible: bool) -> bool:
    if not is_qt_object_alive(widget):
        return False
    value = bool(visible)
    try:
        current = widget.isVisible() if hasattr(widget, "isVisible") else None
        if current != value and hasattr(widget, "setVisible"):
            widget.setVisible(value)
        return True
    except RuntimeError:
        return False
    except Exception:
        return False


def set_value_if_changed(widget: object | None, value: object) -> bool:
    if not is_qt_object_alive(widget):
        return False
    try:
        current = widget.value() if hasattr(widget, "value") else None
        if current != value and hasattr(widget, "setValue"):
            widget.setValue(value)
        return True
    except RuntimeError:
        return False
    except Exception:
        return False


def set_format_if_changed(widget: object | None, text: object) -> bool:
    if not is_qt_object_alive(widget):
        return False
    value = str(text)
    try:
        current = widget.format() if hasattr(widget, "format") else None
        if current != value and hasattr(widget, "setFormat"):
            widget.setFormat(value)
        return True
    except RuntimeError:
        return False
    except Exception:
        return False


def set_tooltip_if_changed(widget: object | None, text: object) -> bool:
    if not is_qt_object_alive(widget):
        return False
    value = str(text)
    try:
        current = widget.toolTip() if hasattr(widget, "toolTip") else None
        if current != value and hasattr(widget, "setToolTip"):
            widget.setToolTip(value)
        return True
    except RuntimeError:
        return False
    except Exception:
        return False


def set_property_if_changed(widget: object | None, name: str, value: object) -> bool:
    if not is_qt_object_alive(widget):
        return False
    try:
        current = widget.property(name) if hasattr(widget, "property") else None
        if current == value or not hasattr(widget, "setProperty"):
            return False
        widget.setProperty(name, value)
        return True
    except RuntimeError:
        return False
    except Exception:
        return False


def set_range_if_changed(widget: object | None, minimum: int, maximum: int) -> bool:
    if not is_qt_object_alive(widget):
        return False
    try:
        current = (
            widget.minimum() if hasattr(widget, "minimum") else None,
            widget.maximum() if hasattr(widget, "maximum") else None,
        )
        desired = (int(minimum), int(maximum))
        if current != desired and hasattr(widget, "setRange"):
            widget.setRange(*desired)
        return True
    except RuntimeError:
        return False
    except Exception:
        return False
