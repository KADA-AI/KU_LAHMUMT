from __future__ import annotations

import unittest
from unittest.mock import patch

import run
from app.ui.main_window import MainWindow


class _Process:
    pid = 43210
    returncode = None

    def poll(self):
        return None


class _DbPathLine:
    def text(self):
        return r"C:\temp\Scenario_auto\SBC3"


class _Window:
    def __init__(self, existing=None):
        self.btn_mission_status_monitor = None
        self._role_processes = {}
        if existing is not None:
            self._role_processes["mission_status_monitor"] = existing
        self._mission_status_monitor_url = None
        self._db_path_line = _DbPathLine()
        self.logs = []
        self.killed = []

    def _log_simulation(self, *, message):
        self.logs.append(message)

    def _kill_process_tree(self, process):
        self.killed.append(process)

    def _is_tcp_port_available(self, _host, _port):
        return True

    def _is_mission_status_monitor_service(self, _host, _port):
        return False

    def _kill_port_occupant(self, _port, _name):
        raise AssertionError("unexpected port cleanup")

    def _check_mission_status_monitor_process(self, _process):
        return None


class MissionStatusDashboardLaunchTests(unittest.TestCase):
    def test_auto_start_does_not_toggle_off_running_monitor(self):
        process = _Process()
        window = _Window(process)

        MainWindow._launch_mission_status_monitor(window, auto_start=True)

        self.assertIs(window._role_processes["mission_status_monitor"], process)
        self.assertEqual(window.killed, [])
        self.assertIn("auto-start skipped", window.logs[-1])

    def test_manual_button_behavior_still_stops_running_monitor(self):
        process = _Process()
        window = _Window(process)

        MainWindow._launch_mission_status_monitor(window)

        self.assertNotIn("mission_status_monitor", window._role_processes)
        self.assertEqual(window.killed, [process])

    def test_launch_passes_selected_db_and_tracks_process(self):
        process = _Process()
        window = _Window()
        with patch("app.ui.main_window.subprocess.Popen", return_value=process) as popen, patch(
            "app.ui.main_window.QTimer.singleShot"
        ):
            MainWindow._launch_mission_status_monitor(window, auto_start=True)

        env = popen.call_args.kwargs["env"]
        self.assertEqual(env["KU_MISSION_DB_ROOT"], _DbPathLine().text())
        self.assertEqual(env["KU_LAUNCHED_BY_DASHBOARD"], "1")
        self.assertIs(window._role_processes["mission_status_monitor"], process)
        self.assertEqual(window._mission_status_monitor_url, "http://127.0.0.1:8300/")

    def test_auto_start_failure_logs_without_blocking_dashboard(self):
        window = _Window()
        with patch("app.ui.main_window.subprocess.Popen", side_effect=OSError("launch failed")), patch(
            "app.ui.main_window.QMessageBox.critical"
        ) as critical:
            MainWindow._launch_mission_status_monitor(window, auto_start=True)

        critical.assert_not_called()
        self.assertIn("launch failed", window.logs[-1])

    def test_launch_all_schedules_monitor_after_existing_modules(self):
        scheduled = []
        auto_callback = lambda: None

        class _Orchestrator:
            _last_launch_all_requested_at = 0.0
            _auto_launch_mission_status_monitor = staticmethod(auto_callback)

            @staticmethod
            def _ensure_launch_ready(**_kwargs):
                return True

            @staticmethod
            def _safe_log(_message):
                return None

            @staticmethod
            def _launch_gui(*_args, **_kwargs):
                return None

            @staticmethod
            def _set_mode_text_all(_mode):
                return None

            @staticmethod
            def _refresh_service_status_panel():
                return None

        orchestrator = _Orchestrator()
        with patch.object(run.time, "monotonic", return_value=100.0), patch.dict(
            run.os.environ, {"KU_NFUSION_LAUNCH_STAGGER_MS": "25"}
        ), patch.object(run.QTimer, "singleShot", side_effect=lambda delay, callback: scheduled.append((delay, callback))):
            run.DashboardOrchestrator._launch_all_guis(orchestrator)

        self.assertIn((100, auto_callback), scheduled)


if __name__ == "__main__":
    unittest.main()
