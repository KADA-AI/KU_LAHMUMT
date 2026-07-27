from __future__ import annotations

import unittest

from modules.mission_status_monitoring.service import _gsd_requirement_satisfied


class GsdSatisfactionTests(unittest.TestCase):
    def test_smaller_gsd_satisfies_maximum_limit(self) -> None:
        self.assertTrue(_gsd_requirement_satisfied(5.41, 8.38))

    def test_equal_gsd_satisfies_maximum_limit(self) -> None:
        self.assertTrue(_gsd_requirement_satisfied(8.38, 8.38))

    def test_larger_gsd_fails_maximum_limit(self) -> None:
        self.assertFalse(_gsd_requirement_satisfied(8.39, 8.38))

    def test_missing_value_is_not_evaluated(self) -> None:
        self.assertIsNone(_gsd_requirement_satisfied(None, 8.38))


if __name__ == "__main__":
    unittest.main()
