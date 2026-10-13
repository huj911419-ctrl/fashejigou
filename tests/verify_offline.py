#!/usr/bin/env python3
"""Run production-source host checks in a disposable build, never flash hardware.

Uses only Python's standard library, CMake/CTest and a host GCC/Clang toolchain.
Paths to local tools are supplied by the caller, not stored in project code.
"""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
import uuid
import xml.etree.ElementTree as ET


REPO = Path(__file__).resolve().parent.parent
BUILD_PREFIX = "robomaster-verify-"
OWNER_FILE = ".verification-owner"


def executable(value):
    resolved = shutil.which(value)
    if resolved is None:
        raise ValueError(f"Tool not found: {value}")
    return str(Path(resolved).resolve())


def fingerprint():
    """Record a digest of project/test sources, excluding generated outputs."""
    digest = hashlib.sha256()
    paths = []
    for name in ("application", "module", "bsp", "tests", "build_scripts"):
        paths.extend(path for path in (REPO / name).rglob("*")
                     if path.is_file() and (path.suffix in (".c", ".h", ".mk", ".py")
                                            or path.name == "CMakeLists.txt"))
    paths.extend(REPO.glob("Makefile*"))
    for path in sorted(paths):
        digest.update(path.relative_to(REPO).as_posix().encode("utf-8") + b"\0")
        digest.update(hashlib.sha256(path.read_bytes()).digest())
    return {"sha256": digest.hexdigest(), "files": len(paths),
            "scope": "application/module/bsp/tests/build_scripts C, H, mk, py and build manifests"}


def remove_owned_build(build, parent, token):
    """Delete only this run's uniquely named direct child of the temp directory."""
    resolved = build.resolve(strict=True)
    if (build.is_symlink() or resolved.parent != parent.resolve(strict=True)
            or not resolved.name.startswith(BUILD_PREFIX)
            or (resolved / OWNER_FILE).read_text(encoding="utf-8") != token):
        raise RuntimeError(f"Refusing to clean an unowned build directory: {build}")
    shutil.rmtree(resolved)


def run_step(name, command, report_dir, environment, timeout, results):
    log = report_dir / f"{name}.log"
    started = time.monotonic()
    print(f"[{name}] running", flush=True)
    result = {"name": name, "command": command, "log": log.name}
    results.append(result)
    # shell=False preserves spaces and treats all paths/arguments as data.
    with log.open("w", encoding="utf-8") as output:
        try:
            process = subprocess.run(command, cwd=REPO, env=environment, stdout=output,
                                     stderr=subprocess.STDOUT, timeout=timeout, check=False)
            result["returncode"] = process.returncode
        except (OSError, subprocess.TimeoutExpired, KeyboardInterrupt) as error:
            result["returncode"] = -1
            result["error"] = str(error) or "Interrupted by user"
            output.write(f"\n{error}\n")
    result["seconds"] = round(time.monotonic() - started, 3)
    if result["returncode"] != 0:
        raise RuntimeError(f"{name} failed; see {log}")
    print(f"[{name}] passed ({result['seconds']} s)", flush=True)


def write_report(report_dir, report):
    (report_dir / "report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    lines = ["# 离线验证记录", "", f"- 结果：**{report['result']}**",
             f"- UTC 时间：{report['started_utc']}", f"- 项目：`{REPO}`",
             f"- 源码摘要：`{report['sources']['sha256']}`",
             f"- 测试项：{len(report['tests'])}",
             "- 随机种子：1、20260929、4294967295",
             f"- 临时构建已删除：{report['build_removed']}"]
    if not report["build_removed"]:
        lines.append(f"- 保留现场：`{report['build_directory']}`")
    if report.get("error"):
        lines.append(f"- 错误：{report['error']}")
    lines += ["", "| 阶段 | 返回码 | 用时（秒） | 日志 |",
              "| --- | --- | --- | --- |"]
    for step in report["steps"]:
        lines.append(f"| {step['name']} | {step['returncode']} | {step['seconds']} | "
                     f"[{step['log']}]({step['log']}) |")
    lines += ["", "| 测试 | 状态 |", "| --- | --- |"]
    lines.extend(f"| {test['name']} | {test['status']} |" for test in report["tests"])
    lines += ["", "验证的是真实源码在主机及模拟外设契约下的行为，包括故障和边界输入。",
              "尚未证明：电机实际响应、PID 参数适合性、弹丸初速与一致性、",
              "实板 IRQ/CAN/UART 时序、电机内部失联保护、IMU/机械误差。",
              "本入口不编译或烧录 ARM 固件；固件构建记录应另行保存。", ""]
    (report_dir / "report.md").write_text("\n".join(lines), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", help="Host GCC/Clang executable; otherwise CMake chooses")
    parser.add_argument("--make", help="Optional CMAKE_MAKE_PROGRAM executable")
    parser.add_argument("--generator", help="For example: MinGW Makefiles or Ninja")
    parser.add_argument("--cmake", default="cmake")
    parser.add_argument("--ctest", help="Defaults to CTest beside the selected CMake")
    parser.add_argument("--output", type=Path, default=REPO / ".scratch" / "offline-verification")
    parser.add_argument("--jobs", type=int, default=4)
    parser.add_argument("--keep-build", action="store_true", help="Retain temporary build even on success")
    args = parser.parse_args()
    if args.jobs < 1:
        parser.error("--jobs must be positive")
    try:
        cmake = executable(args.cmake)
        ctest = executable(args.ctest or str(Path(cmake).with_name(
            "ctest.exe" if os.name == "nt" else "ctest")))
        compiler = executable(args.compiler) if args.compiler else None
        make = executable(args.make) if args.make else None
    except ValueError as error:
        parser.error(str(error))
    if compiler and "arm-none-eabi" in Path(compiler).name:
        parser.error("Use a host compiler; ARM firmware executables cannot run on this computer")

    environment = os.environ.copy()
    tool_dirs = [str(Path(tool).parent) for tool in (compiler, make) if tool]
    environment["PATH"] = os.pathsep.join(tool_dirs + [environment.get("PATH", "")])
    timestamp = datetime.now(timezone.utc)
    report_dir = args.output.resolve() / (timestamp.strftime("%Y%m%dT%H%M%SZ-") + uuid.uuid4().hex[:8])
    report_dir.mkdir(parents=True, exist_ok=False)
    temp_parent = Path(tempfile.gettempdir()).resolve()
    build = Path(tempfile.mkdtemp(prefix=BUILD_PREFIX, dir=temp_parent))
    token = uuid.uuid4().hex
    (build / OWNER_FILE).write_text(token, encoding="utf-8")
    report = {"result": "FAIL", "started_utc": timestamp.isoformat(), "sources": fingerprint(),
              "build_directory": str(build), "build_removed": False, "steps": [], "tests": []}
    command = [cmake, "-S", str(REPO / "tests"), "-B", str(build), "-DCMAKE_BUILD_TYPE=Debug"]
    if args.generator:
        command += ["-G", args.generator]
    if compiler:
        command.append(f"-DCMAKE_C_COMPILER={Path(compiler).as_posix()}")
    if make:
        command.append(f"-DCMAKE_MAKE_PROGRAM={Path(make).as_posix()}")
    try:
        run_step("configure", command, report_dir, environment, 120, report["steps"])
        run_step("build", [cmake, "--build", str(build), "--parallel", str(args.jobs)],
                 report_dir, environment, 180, report["steps"])
        run_step("tests", [ctest, "--test-dir", str(build), "--output-on-failure", "--timeout", "30",
                           "--output-junit", str(report_dir / "tests.xml")],
                 report_dir, environment, 180, report["steps"])
        report["result"] = "PASS"
    except (RuntimeError, KeyboardInterrupt) as error:
        report["error"] = str(error) or "Interrupted by user"
    finally:
        junit = report_dir / "tests.xml"
        if junit.exists():
            try:
                for case in ET.parse(junit).iter("testcase"):
                    status = "passed"
                    if case.find("failure") is not None or case.find("error") is not None:
                        status = "failed"
                    elif case.find("skipped") is not None or case.get("status") == "notrun":
                        status = "skipped"
                    report["tests"].append({"name": case.attrib["name"], "status": status})
            except (ET.ParseError, KeyError) as error:
                report["result"] = "FAIL"
                report["error"] = f"Invalid CTest report: {error}"
        if report["result"] == "PASS" and not report["tests"]:
            report["result"] = "FAIL"
            report["error"] = "No test results were recorded"
        if report["result"] == "PASS" and any(test["status"] != "passed" for test in report["tests"]):
            report["result"] = "FAIL"
            report["error"] = "Some tests failed or were skipped"
        if report["result"] == "PASS" and not args.keep_build:
            try:
                remove_owned_build(build, temp_parent, token)
                report["build_removed"] = True
            except (OSError, RuntimeError) as error:
                report["result"] = "FAIL"
                report["error"] = f"Temporary build cleanup failed: {error}"
        write_report(report_dir, report)
    print(f"{report['result']}: {report_dir / 'report.md'}", flush=True)
    return 0 if report["result"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
