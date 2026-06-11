# -*- coding: utf-8 -*-
from __future__ import annotations

import json
import os
import subprocess
from pathlib import Path
from typing import Iterable


def kill_python_processes_for_scripts(
    root: Path | str,
    script_names: Iterable[str],
    *,
    exclude_pids: Iterable[int] | None = None,
    exclude_parent_pids: Iterable[int] | None = None,
    only_orphaned: bool = False,
    timeout_s: float = 8.0,
) -> list[int]:
    """Kill stale Python processes running managed DSS scripts under root."""
    scripts = sorted({str(name).strip() for name in (script_names or []) if str(name).strip()})
    if os.name != "nt" or not scripts:
        return []

    try:
        root_text = str(Path(root).resolve())
    except Exception:
        root_text = str(root)

    exclude_pid_set = {int(os.getpid())}
    for pid in exclude_pids or []:
        try:
            exclude_pid_set.add(int(pid))
        except Exception:
            pass

    exclude_parent_set: set[int] = set()
    for pid in exclude_parent_pids or []:
        try:
            exclude_parent_set.add(int(pid))
        except Exception:
            pass

    env = os.environ.copy()
    env["KU_CLEAN_ROOT"] = root_text
    env["KU_CLEAN_SCRIPTS"] = json.dumps(scripts, ensure_ascii=True)
    env["KU_CLEAN_EXCLUDE_PIDS"] = json.dumps(sorted(exclude_pid_set), ensure_ascii=True)
    env["KU_CLEAN_EXCLUDE_PARENT_PIDS"] = json.dumps(sorted(exclude_parent_set), ensure_ascii=True)
    env["KU_CLEAN_ONLY_ORPHANED"] = "1" if only_orphaned else "0"

    command = r"""
$root = ($env:KU_CLEAN_ROOT).ToLower().Replace('/','\')
$scripts = @()
foreach ($item in (ConvertFrom-Json $env:KU_CLEAN_SCRIPTS)) {
  $scripts += [string]$item
}
$exclude = @{}
foreach ($item in (ConvertFrom-Json $env:KU_CLEAN_EXCLUDE_PIDS)) {
  try { $exclude[[int]$item] = $true } catch {}
}
$excludeParents = @{}
foreach ($item in (ConvertFrom-Json $env:KU_CLEAN_EXCLUDE_PARENT_PIDS)) {
  try { $excludeParents[[int]$item] = $true } catch {}
}
$onlyOrphaned = $env:KU_CLEAN_ONLY_ORPHANED -eq '1'
Get-CimInstance Win32_Process | Where-Object {
  $_.CommandLine -and $_.Name -match '^(python|pythonw)\.exe$'
} | ForEach-Object {
  $pidValue = [int]$_.ProcessId
  if ($exclude.ContainsKey($pidValue)) { return }
  if ($excludeParents.ContainsKey([int]$_.ParentProcessId)) { return }
  $cmd = $_.CommandLine.ToLower().Replace('/','\')
  if (-not $cmd.Contains($root)) { return }
  $matched = $false
  foreach ($script in $scripts) {
    $scriptText = ([string]$script).ToLower().Replace('/','\')
    if ($cmd.Contains("\" + $scriptText) -or $cmd.Contains($scriptText)) {
      $matched = $true
      break
    }
  }
  if (-not $matched) { return }
  if ($onlyOrphaned) {
    $parent = Get-CimInstance Win32_Process -Filter "ProcessId=$([int]$_.ParentProcessId)" -ErrorAction SilentlyContinue
    if ($null -ne $parent) { return }
  }
  try {
    Stop-Process -Id $pidValue -Force -ErrorAction Stop
    [Console]::Out.WriteLine($pidValue)
  } catch {}
}
"""
    try:
        result = subprocess.run(
            ["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass", "-Command", command],
            capture_output=True,
            text=True,
            timeout=max(1.0, float(timeout_s)),
            env=env,
        )
    except Exception:
        return []

    killed: list[int] = []
    for line in (result.stdout or "").splitlines():
        try:
            pid = int(line.strip())
        except Exception:
            continue
        if pid > 0 and pid not in killed:
            killed.append(pid)
    return killed
