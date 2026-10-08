"""Lexus IS branch: tests for boot_recovery (process restart, failure logging, port retry, audio-ready gate).

Runs without a device: the openpilot modules boot_recovery and process.py import are stubbed while they load (sys.modules
is restored afterwards), and the feature file is replaced by a dict. ensure_running is checked against the upstream copy of process.py with both settings off.
"""
import contextlib
import importlib.util
import os
import re
import sys
import tempfile
import types
import unittest
from pathlib import Path

HERE = Path(__file__).resolve()
OP = HERE.parents[3]                     # .../openpilot (the package directory)
BOOT = OP / "sunnypilot" / "lexus_is" / "boot_recovery.py"
PROCESS = OP / "system" / "manager" / "process.py"
UPSTREAM_PROCESS = os.environ.get("UPSTREAM_PROCESS_PY")   # optional: staging's process.py for the off-equivalence test

FLAGS = {"boot_recovery": False, "audio_ready_gate": False}
LOG: list[tuple] = []


class _Cloudlog:
  def event(self, event, *args, **kw):
    LOG.append(("event", event, kw))

  def exception(self, msg):
    LOG.append(("exception", msg, sys.exc_info()[0]))

  def info(self, msg):
    LOG.append(("info", msg))

  def warning(self, msg):
    LOG.append(("warning", msg))


_STUB_NAMES = ("openpilot", "openpilot.common", "openpilot.common.swaglog", "openpilot.sunnypilot",
               "openpilot.sunnypilot.lexus_is", "openpilot.sunnypilot.lexus_is.boot_recovery", "setproctitle",
               "openpilot.cereal", "opendbc", "opendbc.car", "opendbc.car.structs", "openpilot.cereal.messaging",
               "openpilot.system", "openpilot.system.sentry", "openpilot.common.basedir", "openpilot.common.params")


@contextlib.contextmanager
def _stubbed(boot_module=None):
  """Stub the modules boot_recovery and process.py import, only while loading them; sys.modules is restored after,
  so other tests in the same session import the real packages."""
  saved = {n: sys.modules.get(n) for n in _STUB_NAMES}

  def stub(name, **attrs):
    m = types.ModuleType(name)
    m.__dict__.update(attrs)
    sys.modules[name] = m
    return m

  try:
    for n in ("openpilot", "openpilot.common", "openpilot.sunnypilot", "openpilot.sunnypilot.lexus_is", "opendbc",
              "opendbc.car", "openpilot.cereal.messaging", "openpilot.system"):
      stub(n)
    stub("openpilot.common.swaglog", cloudlog=_Cloudlog())
    stub("setproctitle", setproctitle=lambda *_: None)
    stub("openpilot.cereal", log=types.SimpleNamespace(ManagerState=types.SimpleNamespace(ProcessState=None)))
    stub("opendbc.car.structs", car=types.SimpleNamespace(CarParams=object))
    stub("openpilot.system.sentry", capture_exception=lambda: None, set_tag=lambda *a: None)
    stub("openpilot.common.basedir", BASEDIR="/tmp")
    stub("openpilot.common.params", Params=object)
    if boot_module is not None:
      sys.modules["openpilot.sunnypilot.lexus_is.boot_recovery"] = boot_module
    yield
  finally:
    for n, m in saved.items():
      if m is None:
        sys.modules.pop(n, None)
      else:
        sys.modules[n] = m


def _load(name, path, boot_module=None):
  with _stubbed(boot_module):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
  return mod


br = _load("lexus_is_boot_recovery_under_test", BOOT)
REAL_FEATURE = br._feature
br._feature = lambda key: FLAGS[key]   # the feature file is replaced by FLAGS


class FakeProc:
  def __init__(self, exitcode=None, pid=100):
    self.exitcode = exitcode
    self.pid = pid

  def is_alive(self):
    return self.exitcode is None


class FakeManaged:
  """Stands in for a PythonProcess: start() launches only while proc is None, like upstream."""
  next_pid = 1000

  def __init__(self, name, should=True):
    self.name = name
    self.proc = None
    self.shutting_down = False
    self.enabled = True
    self.should = should
    self.starts = 0

  def should_run(self, started, params, CP):
    return self.should and started

  def start(self):
    if self.shutting_down:   # as upstream: a pending non-blocking stop is finished first
      self.stop()
    if self.proc is not None:
      return
    FakeManaged.next_pid += 1
    self.proc = FakeProc(None, FakeManaged.next_pid)
    self.starts += 1

  def stop(self, block=True, **kw):
    if self.proc is not None and self.proc.exitcode is not None:
      self.proc = None
      self.shutting_down = False


class Clock:
  def __init__(self):
    self.t = 0.0

  def __call__(self):
    return self.t


class TestCrashRestarter(unittest.TestCase):
  def setUp(self):
    LOG.clear()
    self.clock = Clock()

  def make(self, on):
    return br.CrashRestarter(enabled_fn=lambda: on, clock=self.clock)

  def test_off_does_nothing(self):
    r = self.make(False)
    p = FakeManaged("soundd")
    p.proc = FakeProc(exitcode=1)
    for _ in range(20):
      self.clock.t += 1.0
      r.update([p])
    self.assertIsNotNone(p.proc)
    self.assertEqual(LOG, [])

  def test_restart_after_delay_then_give_up(self):
    r = self.make(True)
    p = FakeManaged("micd")
    p.start()
    restarts = 0
    for _crash in range(5):
      p.proc.exitcode = 1                     # it crashes
      t_crash = self.clock.t
      for _ in range(int(br.RESTART_DELAY_S * 2) + 2):
        r.update([p])
        if p.proc is None:
          restarts += 1
          self.assertGreaterEqual(self.clock.t - t_crash, br.RESTART_DELAY_S)
          p.start()
          break
        self.clock.t += 0.5
    self.assertEqual(restarts, br.RESTART_MAX)
    self.assertEqual(sum(1 for e in LOG if e[1] == "lexus_is_restart"), br.RESTART_MAX)
    self.assertEqual(sum(1 for e in LOG if e[1] == "lexus_is_restart_gave_up"), 1)
    self.assertIsNotNone(p.proc)              # left dead after the budget: processNotRunning stays visible

  def test_waits_full_delay(self):
    r = self.make(True)
    p = FakeManaged("soundd")
    p.start()
    p.proc.exitcode = 1
    r.update([p])                             # first seen dead at t = 0
    self.clock.t = br.RESTART_DELAY_S - 0.01
    r.update([p])
    self.assertIsNotNone(p.proc)
    self.clock.t = br.RESTART_DELAY_S
    r.update([p])
    self.assertIsNone(p.proc)

  def test_clean_exit_shutting_down_and_other_names_left_alone(self):
    r = self.make(True)
    a = FakeManaged("soundd")
    a.proc = FakeProc(exitcode=0)
    b = FakeManaged("micd")
    b.proc = FakeProc(exitcode=1)
    b.shutting_down = True
    c = FakeManaged("modeld")
    c.proc = FakeProc(exitcode=1)
    for _ in range(30):
      self.clock.t += 1.0
      r.update([a, b, c])
    self.assertIsNotNone(a.proc)
    self.assertIsNotNone(b.proc)
    self.assertIsNotNone(c.proc)

  def test_forget_gives_a_fresh_budget(self):
    r = self.make(True)
    r.attempts["qcomgpsd"] = br.RESTART_MAX
    r.gave_up.add("qcomgpsd")
    r.forget("qcomgpsd")
    self.assertNotIn("qcomgpsd", r.attempts)
    self.assertNotIn("qcomgpsd", r.gave_up)


class TestEnsureRunning(unittest.TestCase):
  """ensure_running from the patched process.py, with FakeManaged processes."""

  def setUp(self):
    LOG.clear()
    self.clock = Clock()

  def run_sequence(self, process_mod, restarter):
    if hasattr(process_mod, "RESTARTER"):
      process_mod.RESTARTER = restarter
    p = FakeManaged("soundd")
    q = FakeManaged("modeld")
    trace = []
    for step in range(40):
      started = step < 30
      if step == 3:
        p.proc.exitcode = 1                   # soundd crashes onroad
        q.proc.exitcode = 1                   # so does a process outside the set
      process_mod.ensure_running([p, q], started, params=None, CP=None)
      trace.append((p.starts, p.proc is not None and p.proc.exitcode is None, q.starts))
      self.clock.t += 0.5
    return trace, p, q

  def test_patched_on_restarts_only_listed(self):
    mod = _load("patched_process_on", PROCESS, br)
    trace, p, q = self.run_sequence(mod, br.CrashRestarter(enabled_fn=lambda: True, clock=self.clock))
    self.assertEqual(p.starts, 2)             # first start + one restart after the crash
    self.assertEqual(q.starts, 1)
    restart_step = next(i for i, t in enumerate(trace) if t[0] == 2)
    self.assertGreaterEqual((restart_step - 3) * 0.5, br.RESTART_DELAY_S)

  def test_patched_off_equals_upstream(self):
    if not UPSTREAM_PROCESS:
      self.skipTest("UPSTREAM_PROCESS_PY not set")
    mod_off = _load("patched_process_off", PROCESS, br)
    trace_off, *_ = self.run_sequence(mod_off, br.CrashRestarter(enabled_fn=lambda: False, clock=self.clock))
    self.clock = Clock()
    mod_up = _load("upstream_process", UPSTREAM_PROCESS)
    trace_up, *_ = self.run_sequence(mod_up, None)
    self.assertEqual(trace_off, trace_up)


class TestLogFailures(unittest.TestCase):
  def setUp(self):
    LOG.clear()

  def _fn(self):
    @br.log_failures("micd")
    def get_stream():
      raise OSError("PortAudio: no device")
    return get_stream

  def test_on_logs_and_reraises(self):
    FLAGS["boot_recovery"] = True
    try:
      with self.assertRaises(OSError):
        self._fn()()
    finally:
      FLAGS["boot_recovery"] = False
    self.assertEqual(LOG, [("exception", "micd: get_stream failed", OSError)])

  def test_off_silent(self):
    with self.assertRaises(OSError):
      self._fn()()
    self.assertEqual(LOG, [])

  def test_success_passes_through(self):
    FLAGS["boot_recovery"] = True
    try:
      self.assertEqual(br.log_failures("x")(lambda: 7)(), 7)
    finally:
      FLAGS["boot_recovery"] = False


class TestRetryOpen(unittest.TestCase):
  def setUp(self):
    LOG.clear()
    self.sleeps = []
    self.calls = 0

  def opener(self, fail_times, exc=PermissionError):
    def open_serial():
      self.calls += 1
      if self.calls <= fail_times:
        raise exc(13, "Permission denied: '/dev/ttyUSB0'")
      return "serial"
    return br.retry_open("qcomgpsd", sleep=self.sleeps.append)(open_serial)

  def test_off_single_attempt(self):
    with self.assertRaises(PermissionError):
      self.opener(1)()
    self.assertEqual(self.calls, 1)
    self.assertEqual(self.sleeps, [])

  def test_on_recovers(self):
    FLAGS["boot_recovery"] = True
    try:
      self.assertEqual(self.opener(3)(), "serial")
    finally:
      FLAGS["boot_recovery"] = False
    self.assertEqual(self.calls, 4)
    self.assertEqual(self.sleeps, [br.SERIAL_OPEN_DELAY_S] * 3)

  def test_on_gives_up_with_last_error(self):
    FLAGS["boot_recovery"] = True
    try:
      with self.assertRaises(PermissionError):
        self.opener(99)()
    finally:
      FLAGS["boot_recovery"] = False
    self.assertEqual(self.calls, br.SERIAL_OPEN_ATTEMPTS)
    self.assertEqual(len(self.sleeps), br.SERIAL_OPEN_ATTEMPTS - 1)

  def test_on_other_errors_not_retried(self):
    FLAGS["boot_recovery"] = True
    try:
      with self.assertRaises(ValueError):
        self.opener(1, exc=lambda *a: ValueError("x"))()
    finally:
      FLAGS["boot_recovery"] = False
    self.assertEqual(self.calls, 1)


class TestAudioReadyGate(unittest.TestCase):
  def setUp(self):
    LOG.clear()
    self.dir = tempfile.TemporaryDirectory()
    self.path = os.path.join(self.dir.name, "ready")
    # the gate is off under REPLAY / SIMULATION; these tests model a car
    self.saved_env = {var: os.environ.pop(var, None) for var in ("REPLAY", "SIMULATION")}

  def tearDown(self):
    self.dir.cleanup()
    for var, value in self.saved_env.items():
      if value is not None:
        os.environ[var] = value

  @staticmethod
  def ms(running=True, pid=123, present=True):
    procs = [types.SimpleNamespace(name="soundd", running=running, pid=pid)] if present else []
    procs.append(types.SimpleNamespace(name="micd", running=True, pid=5))
    return types.SimpleNamespace(processes=procs)

  def test_off_never_blocks(self):
    g = br.AudioReadyGate(enabled_fn=lambda: False, path=self.path)
    self.assertFalse(g.update(self.ms()))
    FLAGS["audio_ready_gate"] = False
    br.mark_soundd_ready(self.path)
    self.assertFalse(os.path.exists(self.path))

  def test_on_blocks_until_marker_matches(self):
    g = br.AudioReadyGate(enabled_fn=lambda: True, path=self.path)
    self.assertTrue(g.update(self.ms(pid=os.getpid())))         # no marker yet
    FLAGS["audio_ready_gate"] = True
    try:
      br.mark_soundd_ready(self.path)                           # writes this process's pid
    finally:
      FLAGS["audio_ready_gate"] = False
    self.assertFalse(g.update(self.ms(pid=os.getpid())))
    self.assertTrue(g.update(self.ms(pid=os.getpid() + 1)))     # restarted soundd: stale marker blocks again
    self.assertFalse(g.update(self.ms(running=False)))          # not running: processNotRunning's case
    self.assertFalse(g.update(self.ms(present=False)))

  def test_state_changes_logged_once_and_evaluated_flag(self):
    g = br.AudioReadyGate(enabled_fn=lambda: True, path=self.path)
    self.assertFalse(g.evaluated)
    g.update(self.ms(pid=77))
    g.update(self.ms(pid=77))
    with open(self.path, "w") as f:
      f.write("77")
    g.update(self.ms(pid=77))
    self.assertTrue(g.evaluated)
    self.assertEqual([e[2]["blocking"] for e in LOG if e[1] == "lexus_is_audio_ready_gate"], [True, False])

  def test_marker_failure_cleans_up(self):
    os.mkdir(self.path)                                          # os.replace onto a directory fails
    FLAGS["audio_ready_gate"] = True
    try:
      br.mark_soundd_ready(self.path)
    finally:
      FLAGS["audio_ready_gate"] = False
    self.assertEqual(sorted(os.listdir(self.dir.name)), ["ready"])
    self.assertTrue(any(e[0] == "exception" for e in LOG))

  def test_on_garbage_marker_blocks(self):
    with open(self.path, "w") as f:
      f.write("not a pid")
    g = br.AudioReadyGate(enabled_fn=lambda: True, path=self.path)
    self.assertTrue(g.update(self.ms()))


class TestGateIntegration(unittest.TestCase):
  """The gate's event must block engagement only: card.py stops all CAN output while selfdriveInitializing is present."""

  def test_event_is_not_cards_init_barrier(self):
    selfdrived = (OP / "selfdrive" / "selfdrived" / "selfdrived.py").read_text()
    card = (OP / "selfdrive" / "car" / "card.py").read_text()
    events = (OP / "selfdrive" / "selfdrived" / "events.py").read_text()
    hook = selfdrived[selfdrived.index("# Lexus IS branch: no engagement"):]
    hook = hook[:hook.index("\n\n")]
    self.assertIn("self.events.add(EventName.carNotReady)", hook)
    self.assertNotIn("selfdriveInitializing", hook)
    self.assertIn("EventName.selfdriveInitializing", card)      # card's barrier is still that event ...
    self.assertNotIn("carNotReady", card)                        # ... and card does not look at carNotReady
    entry = events[events.index("EventName.carNotReady: {"):]
    entry = entry[:entry.index("},")]
    self.assertEqual(set(re.findall(r"ET\.([A-Z_]+)", entry)), {"NO_ENTRY"})

  def test_replay_and_simulation_disable_gate(self):
    saved = {var: os.environ.pop(var, None) for var in ("REPLAY", "SIMULATION")}
    FLAGS["audio_ready_gate"] = True
    try:
      for var in ("REPLAY", "SIMULATION"):
        os.environ[var] = "1"
        try:
          self.assertFalse(br.audio_ready_gate_enabled())
        finally:
          del os.environ[var]
      self.assertTrue(br.audio_ready_gate_enabled())
    finally:
      FLAGS["audio_ready_gate"] = False
      for var, value in saved.items():
        if value is not None:
          os.environ[var] = value

  def test_feature_reads_file_without_filling_the_cache(self):
    def no_cache(name):
      raise AssertionError("features.enabled() fills the per-process cache that forked children inherit")
    fake = types.ModuleType("openpilot.sunnypilot.lexus_is.features")
    fake.load = lambda: {"boot_recovery": False, "audio_ready_gate": True}
    fake.enabled = no_cache
    pkg = types.ModuleType("openpilot.sunnypilot.lexus_is")
    pkg.features = fake
    names = ("openpilot", "openpilot.sunnypilot", "openpilot.sunnypilot.lexus_is", "openpilot.sunnypilot.lexus_is.features")
    saved = {n: sys.modules.get(n) for n in names}
    try:
      for n in names[:2]:
        sys.modules[n] = types.ModuleType(n)
      sys.modules[names[2]] = pkg
      sys.modules[names[3]] = fake
      self.assertFalse(REAL_FEATURE("boot_recovery"))
      self.assertTrue(REAL_FEATURE("audio_ready_gate"))
    finally:
      for n, m in saved.items():
        if m is None:
          sys.modules.pop(n, None)
        else:
          sys.modules[n] = m


if __name__ == "__main__":
  unittest.main()
