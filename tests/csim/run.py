#!/usr/bin/env python3
import argparse
import json
import re
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCENARIO_DIR = ROOT / "tests" / "csim" / "scenarios"
OUT_DIR = ROOT / "tests" / "csim" / "out"

TSIM_RE = re.compile(
    r"TSIM t (?P<t>-?\d+(?:\.\d+)?) mode (?P<mode>-?\d+) "
    r"hd (?P<hd>-?\d+(?:\.\d+)?) vd (?P<vd>-?\d+(?:\.\d+)?) "
    r"lat (?P<lat>-?\d+(?:\.\d+)?) lon (?P<lon>-?\d+(?:\.\d+)?) "
    r"alt (?P<alt>-?\d+(?:\.\d+)?) trk (?P<trk>-?\d+(?:\.\d+)?) "
    r"range (?P<range>-?\d+(?:\.\d+)?) xte (?P<xte>-?\d+(?:\.\d+)?)"
)


def run(cmd, *, cwd=ROOT, stdout=None):
    return subprocess.run(cmd, cwd=cwd, check=False, text=True, stdout=stdout)


def load_scenarios(names):
    if names:
        paths = []
        for name in names:
            path = Path(name)
            if not path.suffix:
                path = SCENARIO_DIR / f"{name}.json"
            elif not path.is_absolute():
                path = ROOT / path
            paths.append(path)
    else:
        paths = sorted(SCENARIO_DIR.glob("*.json"))

    scenarios = []
    for path in paths:
        with path.open() as f:
            scenario = json.load(f)
        scenario["_path"] = path
        scenario.setdefault("name", path.stem)
        scenarios.append(scenario)
    return scenarios


def parse_final_tsim(output):
    final = None
    for line in output.splitlines():
        match = TSIM_RE.search(line)
        if match:
            final = {key: float(value) for key, value in match.groupdict().items()}
            final["mode"] = int(final["mode"])
            final["line"] = line
    return final


def check_range(name, actual, expected):
    if isinstance(expected, list):
        lo, hi = expected
        return lo <= actual <= hi, f"{name}={actual} expected [{lo}, {hi}]"
    return actual == expected, f"{name}={actual} expected {expected}"


def check_scenario(scenario, output, final_tsim):
    failures = []

    for pattern in scenario.get("expect_output", []):
        if not re.search(pattern, output, re.MULTILINE):
            failures.append(f"missing output pattern: {pattern}")

    for pattern in scenario.get("reject_output", []):
        if re.search(pattern, output, re.MULTILINE):
            failures.append(f"unexpected output pattern: {pattern}")

    if final_tsim is None:
        failures.append("missing final TSIM line")
        return failures

    for key, expected in scenario.get("expect_final_tsim", {}).items():
        if key not in final_tsim:
            failures.append(f"unknown TSIM field: {key}")
            continue
        ok, message = check_range(key, final_tsim[key], expected)
        if not ok:
            failures.append(message)

    return failures


def run_scenario(scenario):
    name = scenario["name"]
    track_path = OUT_DIR / f"{name}.tracksim.txt"
    out_path = OUT_DIR / f"{name}.out"
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    track_path.write_text("".join(line.rstrip() + "\n" for line in scenario["track"]))
    cmd = [
        str(ROOT / "autotrim_csim"),
        "--seconds",
        str(scenario["seconds"]),
        "--tracksim",
        str(track_path),
    ]

    result = subprocess.run(cmd, cwd=ROOT, check=False, text=True, capture_output=True)
    output = result.stdout + result.stderr
    out_path.write_text(output)

    final_tsim = parse_final_tsim(output)
    failures = []
    if result.returncode != 0:
        failures.append(f"sim exited {result.returncode}")
    failures.extend(check_scenario(scenario, output, final_tsim))

    return {
        "name": name,
        "out_path": out_path,
        "final_tsim": final_tsim,
        "failures": failures,
    }


def main():
    parser = argparse.ArgumentParser(description="Run autotrim csim regression scenarios")
    parser.add_argument("scenario", nargs="*", help="scenario name or JSON path")
    parser.add_argument("--no-build", action="store_true", help="skip make BOARD=csim")
    args = parser.parse_args()

    scenarios = load_scenarios(args.scenario)
    if not scenarios:
        print(f"No scenarios found in {SCENARIO_DIR}", file=sys.stderr)
        return 2

    if not args.no_build:
        build = run(["make", "BOARD=csim", "-j2"])
        if build.returncode != 0:
            return build.returncode

    failed = False
    for scenario in scenarios:
        result = run_scenario(scenario)
        final = result["final_tsim"]
        print(f"== {result['name']} ==")
        if final:
            print(final["line"])
        print(f"output: {result['out_path']}")
        if result["failures"]:
            failed = True
            for failure in result["failures"]:
                print(f"FAIL: {failure}")
        else:
            print("PASS")

    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
