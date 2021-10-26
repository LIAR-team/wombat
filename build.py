#!/usr/bin/python3

import argparse
from enum import Enum
import os
import pathlib
import subprocess

THIS_DIR = os.path.abspath(os.path.dirname(os.path.realpath(__file__)))
WORKSPACE = "_ws"
WORKSPACE_DIR = os.path.join(THIS_DIR, WORKSPACE)

class Workflows(Enum):
  BUILD=1
  TEST=2

def parse_args():
  parser = argparse.ArgumentParser(description="The Wombat build script")

  parser.add_argument("--test",       help="Build and run all unit-tests", action="store_true")
  parser.add_argument("--test-pkg",   help="Build and run unit-tests for selected packages", nargs="+", metavar="<packages>")
  parser.add_argument("--test-name",  help="Build and run unit-tests that match this regex", metavar="<regex>")

  return parser.parse_args()

def execute(command, cwd=THIS_DIR, dry_run=False):
  print('+ ' + command)
  actual_command=command if not dry_run else '/bin/true'
  p = subprocess.Popen(actual_command, shell=True, cwd=cwd, stdout=None, stderr=None)
  return p.wait()

def setup_workspace():
  # Create workspace directory
  ws_wombat_dir = os.path.join(WORKSPACE_DIR, "src", "wombat")
  pathlib.Path(ws_wombat_dir).mkdir(parents=True, exist_ok=True)

  def is_valid_pkg(name):
    # Validate directories that are not hidden and are not the workspace itself
    return os.path.isdir(os.path.join(THIS_DIR, name)) and not name.startswith(".") and not name == WORKSPACE

  # Symlink packages into workspace
  packages_dirs = [os.path.join(THIS_DIR, name) for name in os.listdir(THIS_DIR) if is_valid_pkg(name)]
  for pkg in packages_dirs:
    pkg_name = os.path.basename(pkg)
    pkg_link_dest = os.path.join(ws_wombat_dir, pkg_name)
    if not os.path.exists(pkg_link_dest):
      if os.path.islink(pkg_link_dest):
        raise OSError(f"Found broken symbolic link: {pkg_link_dest} clean the workspace before proceeding")
      os.symlink(pkg, pkg_link_dest)

def build(args):
  # TODO: eventually do something with args
  setup_workspace()
  execute("colcon build --event-handlers console_direct+", WORKSPACE_DIR)

def test(args):
  
  build(args)

  test_cmd = "colcon test --event-handlers console_direct+"
  if args.test_pkg:
    test_pkg_str = " ".join(args.test_pkg)
    test_cmd += f" --packages-select {test_pkg_str}"
  
  if args.test_name:
    ctest_args = f"-R {args.test_name}"
    test_cmd += f" --ctest-args {ctest_args} --output-on-failure"

  execute(test_cmd, WORKSPACE_DIR)

def select_workflow(args):
  workflow = Workflows.BUILD
  if args.test or args.test_pkg or args.test_name:
    workflow = Workflows.TEST
  return workflow

def run_workflow(workflow):
  if workflow == Workflows.BUILD:
    build(args)
  elif workflow == Workflows.TEST:
    test(args)
  else:
    raise KeyError("Reached unhandled workflow!")

if __name__ == "__main__":
  args = parse_args()
  workflow = select_workflow(args)
  run_workflow(workflow)
