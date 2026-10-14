#!/usr/bin/env python3
"""Run software-only checks with a deadline per command; never flash hardware."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys


def run(command, root, timeout):
    print("+ " + " ".join(command), flush=True)
    process = subprocess.Popen(command, cwd=root, start_new_session=True)
    try:
        return process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        print(f"TIMEOUT after {timeout}s; killing command process group", file=sys.stderr)
        os.killpg(process.pid, signal.SIGKILL)
        process.wait()
        return 124
    except KeyboardInterrupt:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait()
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=int, default=120, help="seconds per command (default: 120)")
    parser.add_argument("--offline", action="store_true", help="require cached Cargo dependencies")
    parser.add_argument("--build", action="store_true", help="also link release firmware (requires flip-link)")
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    root = Path(__file__).resolve().parent.parent
    common = ["--locked"] + (["--offline"] if args.offline else [])
    checks = [
        ["cargo", "fmt", "--all", "--", "--check"],
        ["cargo", "htest", *common],
        ["cargo", "check", "--lib", "--bins", "--release", *common],
        ["cargo", "clippy", "--lib", "--bins", "--release", *common, "--", "-D", "warnings"],
        ["cargo", "clippy", "-p", "foc", "--target", "x86_64-unknown-linux-gnu",
         "--all-targets", *common, "--", "-D", "warnings"],
    ]
    if args.build:
        checks.append(["cargo", "build", "--release", *common])
    for command in checks:
        status = run(command, root, args.timeout)
        if status:
            return status
    print("All requested software checks passed. Board acceptance is still required.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
