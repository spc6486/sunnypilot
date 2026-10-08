"""
Lexus IS branch: boot-time recovery for micd, soundd and qcomgpsd, and an engagement gate on soundd's audio stream.

Two settings in the Lexus IS feature file (features.py):
  boot_recovery    - restart micd / soundd / qcomgpsd after a crash, log why their audio stream failed to open,
                     and retry opening the modem diag port before giving up.
  audio_ready_gate - no engagement until soundd has its audio stream open.
With both off, every hook below returns before doing anything, and the processes behave as upstream.

Why (logs of routes 386, 380, 37a, 37e; KB SUNNYPILOT_BOOT_PROCESS_NOT_RUNNING_2026-10-07.md):
- micd and soundd open their stream through @retry(attempts=10, delay=3), about 31 s from process start. On AGNOS 19.7
  onroad starts 20-46 s after boot, and the audio device was ready 36.5-53.7 s after boot. On the three earliest starts
  the budget ran out and both processes exited with "get_stream failed after retry".
- The manager starts a PythonProcess only while proc is None, so a crashed process stays dead until the next offroad
  transition. selfdrived then shows "Process Not Running" (NO_ENTRY and SOFT_DISABLE).
- qcomgpsd failed twice opening /dev/ttyUSB0 with EACCES right after the modem's AT port answered, the same way.
- While micd/soundd are still retrying they count as running, so openpilot can be engaged with no audible alerts.
"""
import functools
import os
import time

from openpilot.common.swaglog import cloudlog

RESTART_PROCS = frozenset({"micd", "soundd", "qcomgpsd"})
RESTART_MAX = 3            # restarts per process while it should run (reset when the manager stops it)
RESTART_DELAY_S = 5.0      # wait after a crash is first seen before restarting

SERIAL_OPEN_ATTEMPTS = 10  # qcomgpsd: tries to open the modem diag port
SERIAL_OPEN_DELAY_S = 1.0

# holds soundd's pid once its stream is open (per OPENPILOT_PREFIX, like the other shared-memory names)
SOUNDD_READY_PATH = "/dev/shm/lexus_is_soundd_ready" + os.environ.get("OPENPILOT_PREFIX", "")


def _feature(key: str) -> bool:
  # The single binding to the Lexus IS feature file. features.load() reads the file without filling features' per-process
  # cache: the manager calls this before it forks its children, and a filled cache would be inherited by every child,
  # so an edit of the file would no longer take effect at a process's next start.
  from openpilot.sunnypilot.lexus_is import features
  return bool(features.load()[key])


def boot_recovery_enabled() -> bool:
  return _feature("boot_recovery")


def audio_ready_gate_enabled() -> bool:
  # never in process replay or simulation: no soundd marker exists there, and the gate would refuse every engagement
  if "REPLAY" in os.environ or "SIMULATION" in os.environ:
    return False
  return _feature("audio_ready_gate")


class CrashRestarter:
  """Restarts selected manager processes after a crash, a bounded number of times."""

  def __init__(self, enabled_fn=boot_recovery_enabled, clock=time.monotonic):
    self._enabled_fn = enabled_fn
    self._clock = clock
    self._enabled = None
    self.attempts: dict[str, int] = {}
    self.dead_since: dict[str, float] = {}
    self.gave_up: set[str] = set()

  def enabled(self) -> bool:
    if self._enabled is None:
      self._enabled = bool(self._enabled_fn())
    return self._enabled

  def forget(self, name: str) -> None:
    # the manager stopped this process (should_run false): a new run gets a fresh budget
    if not self.enabled():
      return
    self.attempts.pop(name, None)
    self.dead_since.pop(name, None)
    self.gave_up.discard(name)

  def update(self, running) -> None:
    # called with the processes that should run, before ensure_running starts them
    if not self.enabled():
      return
    now = self._clock()
    for p in running:
      if p.name not in RESTART_PROCS or p.proc is None or p.shutting_down:
        continue
      exitcode = p.proc.exitcode
      if exitcode is None or exitcode == 0:
        self.dead_since.pop(p.name, None)
        continue
      n = self.attempts.get(p.name, 0)
      if n >= RESTART_MAX:
        if p.name not in self.gave_up:
          self.gave_up.add(p.name)
          cloudlog.event("lexus_is_restart_gave_up", process=p.name, exitcode=exitcode, attempts=n, error=True)
        continue
      t0 = self.dead_since.setdefault(p.name, now)
      if now - t0 < RESTART_DELAY_S:
        continue
      self.attempts[p.name] = n + 1
      self.dead_since.pop(p.name, None)
      cloudlog.event("lexus_is_restart", process=p.name, exitcode=exitcode, attempt=n + 1, error=True)
      p.proc = None   # ensure_running's start() launches a new one


RESTARTER = CrashRestarter()


def log_failures(name: str):
  """Decorator under @retry: log each failed attempt with its exception (upstream only prints a generic line)."""
  def decorator(func):
    @functools.wraps(func)
    def wrapper(*args, **kwargs):
      try:
        return func(*args, **kwargs)
      except Exception:
        if boot_recovery_enabled():
          cloudlog.exception(f"{name}: {func.__name__} failed")
        raise
    return wrapper
  return decorator


def retry_open(name: str, attempts: int = SERIAL_OPEN_ATTEMPTS, delay: float = SERIAL_OPEN_DELAY_S,
               sleep=time.sleep):
  """Decorator: with boot_recovery on, retry an OSError from opening a device; the last error is raised as before."""
  def decorator(func):
    @functools.wraps(func)
    def wrapper(*args, **kwargs):
      if not boot_recovery_enabled():
        return func(*args, **kwargs)
      for i in range(attempts):
        try:
          return func(*args, **kwargs)
        except OSError:
          cloudlog.exception(f"{name}: {func.__name__} failed (attempt {i + 1} of {attempts})")
          if i == attempts - 1:
            raise
          sleep(delay)
    return wrapper
  return decorator


def mark_soundd_ready(path: str = SOUNDD_READY_PATH) -> None:
  # soundd, once its output stream is open
  if not audio_ready_gate_enabled():
    return
  tmp = f"{path}.{os.getpid()}"
  try:
    with open(tmp, "w") as f:
      f.write(str(os.getpid()))
    os.replace(tmp, path)
  except OSError:
    cloudlog.exception("soundd: could not write the ready marker")
    try:
      os.unlink(tmp)
    except OSError:
      pass


class AudioReadyGate:
  """selfdrived: True while soundd is running but has not opened its stream (its pid is not in the marker).

  selfdrived turns a block into EventName.carNotReady, whose only alert type is NO_ENTRY and which nothing else
  consumes. Not selfdriveInitializing: card.py treats that event as its init barrier and stops sending CAN while it is
  present."""

  def __init__(self, enabled_fn=audio_ready_gate_enabled, path: str = SOUNDD_READY_PATH):
    self._enabled_fn = enabled_fn
    self._enabled = None
    self._path = path
    self.blocking = False
    self.evaluated = False

  def enabled(self) -> bool:
    if self._enabled is None:
      self._enabled = bool(self._enabled_fn())
    return self._enabled

  def update(self, manager_state) -> bool:
    # re-evaluated on each managerState; soundd absent or not running is processNotRunning's case, not this gate's
    self.evaluated = True
    was_blocking = self.blocking
    self.blocking = self._evaluate(manager_state)
    if self.blocking != was_blocking:
      cloudlog.event("lexus_is_audio_ready_gate", blocking=self.blocking)
    return self.blocking

  def _evaluate(self, manager_state) -> bool:
    if not self.enabled():
      return False
    soundd = next((p for p in manager_state.processes if p.name == "soundd"), None)
    if soundd is None or not soundd.running or soundd.pid == 0:
      return False
    try:
      with open(self._path) as f:
        ready_pid = int(f.read().strip() or 0)
    except (OSError, ValueError):
      ready_pid = 0
    return ready_pid != soundd.pid
