#!/usr/bin/env python3
"""Robot Monitor: live single-line dashboard + UDP command sender."""
# python3 monitor.py

import argparse
import re
import select
import socket
import termios
import threading
import time
import shutil
from datetime import datetime
import sys


class RobotMonitor:
    LOG_PORT = 5556
    TELEMETRY_PORT = 1234
    DASHBOARD_LINES = 5

    def __init__(self, robot_ip="192.168.4.1", robot_port=5555, telemetry_port=1234, log_port=5556):
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.telemetry_port = telemetry_port
        self.log_port = log_port
        self.running = True

        self.prompt = "> "
        self.state_lock = threading.RLock()
        self.print_lock = threading.RLock()
        self.redraw_event = threading.Event()

        self.latest_log_time = "--:--:--.---"
        self.left_temp_c = None
        self.right_temp_c = None
        self.left_sg = None
        self.right_sg = None
        self.kp = 400.0
        self.ki = 0.0
        self.kd = 25.0
        self.target_pitch_deg = 0.0
        self.yaw_deg = 0.0
        self.pitch_deg = 0.0
        self.roll_deg = 0.0
        self.tilt_rate_dps = 0.0
        self.last_command = "idle"
        self.is_active = False

        self._last_action_key = None
        self._last_action_time = 0.0

        self.ANSI = {
            'reset': '\x1b[0m',
            'bold': '\x1b[1m',
            'dim': '\x1b[2m',
            'red': '\x1b[31m',
            'green': '\x1b[32m',
            'yellow': '\x1b[33m',
            'cyan': '\x1b[36m',
        }

    def _set_state(self, **kwargs):
        with self.state_lock:
            for key, value in kwargs.items():
                setattr(self, key, value)
        self.redraw_event.set()

    def _format_float(self, value, precision=1, empty="--"):
        if value is None:
            return empty
        return f"{value:.{precision}f}"

    def _fit_line(self, text):
        width = shutil.get_terminal_size(fallback=(100, 20)).columns
        if width <= 0 or len(text) <= width:
            return text
        if width <= 3:
            return text[:width]
        return text[: width - 3] + "..."

    def _format_dashboard(self):
        with self.state_lock:
            left_temp  = self._format_float(self.left_temp_c, precision=1)
            right_temp = self._format_float(self.right_temp_c, precision=1)
            left_sg    = self._format_float(self.left_sg, precision=0, empty="--")
            right_sg   = self._format_float(self.right_sg, precision=0, empty="--")
            kp           = self._format_float(self.kp, precision=2)
            ki           = self._format_float(self.ki, precision=2)
            kd           = self._format_float(self.kd, precision=2)
            target_pitch = self._format_float(self.target_pitch_deg, precision=2)
            yaw          = self._format_float(self.yaw_deg, precision=1)
            pitch        = self._format_float(self.pitch_deg, precision=1)
            roll         = self._format_float(self.roll_deg, precision=1)
            tilt_rate    = self._format_float(self.tilt_rate_dps, precision=1)
            is_active    = self.is_active

        width = shutil.get_terminal_size(fallback=(120, 24)).columns
        now    = datetime.now().strftime("%H:%M:%S")
        status = f"{self.ANSI['green']}ACTIVE{self.ANSI['reset']}" if is_active else f"{self.ANSI['dim']}IDLE{self.ANSI['reset']}"
        sep    = self.ANSI['dim'] + "─" * width + self.ANSI['reset']

        row1 = (f" Temps   L: {left_temp}°C   R: {right_temp}°C"
                f"   │   SG   L: {left_sg}   R: {right_sg}"
                f"   │   {status}   {now}")
        row2 = (f" PID     kp: {kp}   ki: {ki}   kd: {kd}   target: {target_pitch}°")
        row3 = (f" Pose    yaw: {yaw}°   pitch: {pitch}°   roll: {roll}°   rate: {tilt_rate} °/s")

        return [sep, row1, row2, row3, sep]

    def _redraw_dashboard_line(self):
        rows = self._format_dashboard()
        with self.print_lock:
            try:
                sys.stdout.write("\x1b7")  # save cursor
                for i, row in enumerate(rows):
                    sys.stdout.write(f"\x1b[{i + 1};1H\x1b[2K{row}")
                sys.stdout.write("\x1b8")  # restore cursor
                sys.stdout.flush()
            except Exception:
                pass

    def _update_from_log(self, message):
        now = datetime.now()
        self._set_state(latest_log_time=now.strftime("%H:%M:%S.%f")[:-3])

        tmc_match = re.search(
            r"TMC\s+[—-]\s+LEFT:\s+([0-9.]+)\s+C.*?SG=(\d+).*\|\s+RIGHT:\s+([0-9.]+)\s+C.*?SG=(\d+)",
            message,
        )
        if tmc_match:
            self._set_state(
                left_temp_c=float(tmc_match.group(1)),
                left_sg=float(tmc_match.group(2)),
                right_temp_c=float(tmc_match.group(3)),
                right_sg=float(tmc_match.group(4)),
            )
        else:
            temp_match = re.search(
                r"TMC\s+[—-]\s+LEFT:\s+([0-9.]+)\s+C.*\|\s+RIGHT:\s+([0-9.]+)\s+C",
                message,
            )
            if temp_match:
                self._set_state(
                    left_temp_c=float(temp_match.group(1)),
                    right_temp_c=float(temp_match.group(2)),
                )

        low = message.lower()
        if low.startswith("command:") or " start" in low or low == "start" or " stop" in low or low == "stop":
            words = low.split()
            key = next(
                (w for w in ("start", "stop", "forward", "backward", "left", "right") if w in words),
                low.strip()[:20],
            )

            now_t = time.time()
            show = False
            with self.state_lock:
                if key != self._last_action_key or now_t - self._last_action_time > 2.0:
                    self._last_action_key = key
                    self._last_action_time = now_t
                    show = True

            if show:
                ts = datetime.now().strftime("%H:%M:%S")
                with self.print_lock:
                    print(
                        f"  {self.ANSI['cyan']}▶{self.ANSI['reset']}"
                        f" {self.ANSI['bold']}{key.upper()}{self.ANSI['reset']}"
                        f"  received  [{ts}]"
                    )

        target_match = re.search(r"Target pitch set:\s+([0-9.+-]+)\s+deg", message)
        if target_match:
            self._set_state(target_pitch_deg=float(target_match.group(1)))

        pid_match = re.search(r"PID tuned:\s+kp=([0-9.+-]+)\s+ki=([0-9.+-]+)\s+kd=([0-9.+-]+)", message)
        if pid_match:
            self._set_state(
                kp=float(pid_match.group(1)),
                ki=float(pid_match.group(2)),
                kd=float(pid_match.group(3)),
                last_command=f"PID tuned {pid_match.group(1)}/{pid_match.group(2)}/{pid_match.group(3)}",
            )

    def _update_from_telemetry(self, payload):
        parts = [part.strip() for part in payload.split(",") if part.strip()]
        if len(parts) < 3:
            return

        try:
            yaw = float(parts[0])
            pitch = float(parts[1])
            roll = float(parts[2])
            tilt_rate = float(parts[3]) if len(parts) >= 4 else None
        except ValueError:
            return

        update = dict(yaw_deg=yaw, pitch_deg=pitch, roll_deg=roll)
        if tilt_rate is not None:
            update["tilt_rate_dps"] = tilt_rate
        self._set_state(**update)

    def _listener_loop(self, listen_port, handler, label):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.settimeout(0.2)

        try:
            sock.bind(("0.0.0.0", listen_port))
        except OSError as exc:
            self._set_state(last_command=f"{label} bind failed: {exc}")
            return

        while self.running:
            try:
                data, _addr = sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break

            message = data.decode("utf-8", errors="replace").strip()
            if message:
                handler(message)

        sock.close()

    def _log_listener(self):
        self._listener_loop(self.log_port, self._update_from_log, "log")

    def _telemetry_listener(self):
        self._listener_loop(self.telemetry_port, self._update_from_telemetry, "telemetry")

    def send_command(self, cmd):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.sendto(cmd.encode("utf-8"), (self.robot_ip, self.robot_port))
            sock.close()
        except Exception as exc:
            self._set_state(last_command=f"TX failed: {exc}")
            return False

        if cmd == "start":
            self._set_state(last_command="start", is_active=True)
        elif cmd == "stop":
            self._set_state(last_command="stop", is_active=False)
        else:
            self._set_state(last_command=cmd)
        return True

    def _dashboard_renderer(self):
        while self.running:
            self.redraw_event.wait(0.2)
            self.redraw_event.clear()
            self._redraw_dashboard_line()

    def _readline_simple(self, prompt):
        """Raw-mode line reader: handles DEL/^H as backspace, drains escape seqs."""
        if not sys.stdin.isatty():
            return input(prompt)

        with self.print_lock:
            sys.stdout.write(prompt)
            sys.stdout.flush()

        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        new = termios.tcgetattr(fd)
        # Disable canonical mode, echo, signal chars — but leave OPOST alone
        # so background print() calls still get the \n→\r\n translation.
        new[3] &= ~(termios.ECHO | termios.ICANON | termios.IEXTEN | termios.ISIG)
        new[6][termios.VMIN] = 1
        new[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSADRAIN, new)
        buf = []
        try:
            while True:
                ch = sys.stdin.read(1)
                if ch in ('\r', '\n'):
                    with self.print_lock:
                        sys.stdout.write('\n')
                        sys.stdout.flush()
                    return ''.join(buf)
                if ch == '\x03':
                    raise KeyboardInterrupt
                if ch == '\x04':
                    raise EOFError
                if ch in ('\x7f', '\x08'):
                    if buf:
                        buf.pop()
                        with self.print_lock:
                            sys.stdout.write('\x08 \x08')
                            sys.stdout.flush()
                elif ch == '\x1b':
                    # Drain escape sequence (arrow keys, forward-delete, etc.)
                    r, _, _ = select.select([sys.stdin], [], [], 0.05)
                    if r:
                        nxt = sys.stdin.read(1)
                        if nxt == '[':
                            while True:
                                r2, _, _ = select.select([sys.stdin], [], [], 0.05)
                                if not r2:
                                    break
                                c = sys.stdin.read(1)
                                if c.isalpha() or c == '~':
                                    break
                elif ord(ch) >= 32:
                    buf.append(ch)
                    with self.print_lock:
                        sys.stdout.write(ch)
                        sys.stdout.flush()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def interactive_shell(self):
        height = shutil.get_terminal_size(fallback=(120, 24)).lines
        prompt_row = self.DASHBOARD_LINES + 1
        # Clear screen and lock rows 1-DASHBOARD_LINES as a fixed header.
        # Scroll region confines all input/output to prompt_row..height so
        # dashboard writes to rows 1-5 never corrupt the input area.
        sys.stdout.write(
            "\x1b[2J\x1b[H"                          # clear screen
            f"\x1b[{prompt_row};{height}r"            # set scroll region
            f"\x1b[{prompt_row};1H"                   # move cursor into scroll region
        )
        sys.stdout.flush()
        self._redraw_dashboard_line()

        try:
            while self.running:
                try:
                    cmd = self._readline_simple(self.prompt).strip()
                except (KeyboardInterrupt, EOFError):
                    self.running = False
                    break

                if not cmd:
                    continue

                if cmd.lower() in ("quit", "exit"):
                    self.running = False
                    break

                if cmd.lower() == "help":
                    self.print_help()
                    continue

                self.send_command(cmd)
        finally:
            # Reset scroll region and leave terminal in a clean state
            sys.stdout.write("\x1b[r\x1b[2J\x1b[H")
            sys.stdout.flush()

        print("[*] Exiting...")

    def print_help(self):
        with self.print_lock:
            print(
            """
Available commands:
  start                    Arm balance control
  stop                     Disable motors (emergency stop)
  tune pid <kp> <ki> <kd>  Update PID gains live
  set target <deg>         Set balance point pitch angle
  calibrate                Capture current pitch as balance target
  forward <0-100>          Drive forward at speed %
  backward <0-100>         Drive backward at speed %
  left <0-100>             Turn left at speed %
  right <0-100>            Turn right at speed %
  watchdog_clear           Clear watchdog trip latch
  help                     Show this message
  quit                     Exit monitor
            """.rstrip()
        )

    def run(self):
        threads = [
            threading.Thread(target=self._log_listener, daemon=True),
            threading.Thread(target=self._telemetry_listener, daemon=True),
            threading.Thread(target=self._dashboard_renderer, daemon=True),
        ]
        for thread in threads:
            thread.start()

        time.sleep(0.3)
        self.redraw_event.set()
        self.interactive_shell()


def main():
    parser = argparse.ArgumentParser(description="Robot Monitor - live dashboard + command interface")
    parser.add_argument("-i", "--robot-ip", default="192.168.4.1", help="Robot IP address (default: 192.168.4.1)")
    parser.add_argument("-p", "--robot-port", type=int, default=5555, help="Robot command port (default: 5555)")
    parser.add_argument("--telemetry-port", type=int, default=1234, help="Telemetry UDP port (default: 1234)")
    parser.add_argument("--log-port", type=int, default=5556, help="Log UDP port (default: 5556)")

    args = parser.parse_args()

    monitor = RobotMonitor(
        robot_ip=args.robot_ip,
        robot_port=args.robot_port,
        telemetry_port=args.telemetry_port,
        log_port=args.log_port,
    )

    try:
        monitor.run()
    except KeyboardInterrupt:
        monitor.running = False


if __name__ == "__main__":
    main()
