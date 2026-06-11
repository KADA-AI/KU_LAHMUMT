"""Wrapper around the ver2 monitoring summary tab for reuse in the legacy GUI."""

from modules.monitoring_ver2.gui.tabs.MonitoringTab import MonitoringTab as _Ver2MonitoringTab


class Ver2MonitoringTab(_Ver2MonitoringTab):
    """Thin wrapper to keep import paths local to the monitoring module."""

    pass
