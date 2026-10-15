#!/usr/bin/env python3
"""
Check that the prebuild hook really picks the '#pragma arduino ...' lines out of
a sketch and writes them where the build reads them from.

    python3 avr/extras/check_hooks.py --fqbn TinyCore:avr:x5:chip=85

The hook exists in two versions, a shell one and a batch one, and they only ever
run on their own platform. A failure is silent by nature: the options file stays
empty, the flags are quietly dropped and the build succeeds all the same. So
this builds a sketch whose pragmas are known and looks at what came out.

Which pragmas a core understands differs from core to core, so the names are
read out of platform.txt instead of being written down here.

Needs arduino-cli with the core installed. Standard library apart from that.
"""

import argparse
import os
import re
import shutil
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from check_core import working_directory     # pylint: disable=wrong-import-position

TOKEN = re.compile(r'"[^"]*"|\S+')
NAME = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")

SKETCH = """{pragmas}
void setup() {{ }}
void loop() {{ }}
"""


def pragma_names(root: str) -> list:
    """The pragma names the prebuild hook is told to look for, out of platform.txt.

    A hook line ends in the names, one bare word each; everything before them is
    a path or a {...} reference and is therefore quoted. So collecting the bare
    words from the right and stopping at the first quoted one gives the names. A
    core that passed its paths unquoted would need a different rule here.
    """
    names = []
    with open(os.path.join(root, "platform.txt"), encoding="utf-8",
              errors="replace") as src:
        for line in src:
            line = line.strip()
            if not line.startswith("recipe.hooks.prebuild.") or "=" not in line:
                continue
            trailing = []
            for token in reversed(TOKEN.findall(line.split("=", 1)[1])):
                if not NAME.match(token):
                    break
                trailing.append(token)
            for name in reversed(trailing):
                if name not in names:
                    names.append(name)
    return names


def build(fqbn: str, folder: str, sketch_source: str) -> str:
    """Compile the probe sketch and return the build directory."""
    sketch = os.path.join(folder, "with blank", "hook_probe")
    os.makedirs(sketch, exist_ok=True)
    with open(os.path.join(sketch, "hook_probe.ino"), "w", encoding="utf-8") as out:
        out.write(sketch_source)
    build_path = os.path.join(folder, "with blank", "build path")
    try:
        done = subprocess.run(["arduino-cli", "compile", "--clean", "-b", fqbn,
                               "--build-path", build_path, sketch],
                              capture_output=True, text=True, check=False, timeout=300)
    except subprocess.TimeoutExpired:
        print("the probe sketch is still building after five minutes, so something "
              "is stuck rather than slow")
        sys.exit(1)
    if done.returncode != 0:
        print("the probe sketch did not compile:")
        print((done.stderr or done.stdout).strip())
        sys.exit(1)
    return build_path


def main() -> int:
    """Build the probe and compare the options files with what the sketch said."""
    parser = argparse.ArgumentParser(description="check the prebuild hook")
    parser.add_argument("--fqbn", required=True, help="the board to build for")
    parser.add_argument("--root", help="the core's folder (default: the one above this)")
    parser.add_argument("--work-dir", metavar="DIR",
                        help="where to build, a temporary directory by default")
    args = parser.parse_args()

    if shutil.which("arduino-cli") is None:
        print("arduino-cli is not on PATH")
        return 1

    root = args.root or os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    expected = {name: "-DPROBE_" + name.upper() for name in pragma_names(root)}
    if not expected:
        print("the prebuild hook looks for no pragmas at all, so there is nothing "
              "to check here")
        return 0
    pragmas = "".join(f"#pragma arduino {name} {flag}\n"
                      for name, flag in expected.items())
    print(f"the hook is told to look for: {', '.join(expected)}")

    with working_directory(args.work_dir) as folder:
        build_path = build(args.fqbn, folder, SKETCH.format(pragmas=pragmas))
        wrong = []
        for flag, want in sorted(expected.items()):
            path = os.path.join(build_path, f"options.{flag}")
            if not os.path.exists(path):
                print(f"  MISSING  options.{flag}")
                wrong.append(flag)
                continue
            with open(path, encoding="utf-8", errors="replace") as src:
                content = src.read().strip()
            if want in content:
                print(f"  ok       options.{flag}: {content!r}")
            else:
                print(f"  WRONG    options.{flag}: {content!r}, expected {want!r}")
                wrong.append(flag)

    if wrong:
        print(f"the hook did not deliver {', '.join(wrong)}")
        return 1
    print("the prebuild hook works on this platform")
    return 0


if __name__ == "__main__":
    sys.exit(main())
