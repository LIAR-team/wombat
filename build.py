#!/usr/bin/python3

import argparse
import os
import pathlib
import subprocess

THIS_DIR = os.path.abspath(os.path.dirname(os.path.realpath(__file__)))
WORKSPACE = "_ws"
WORKSPACE_DIR = os.path.join(THIS_DIR, WORKSPACE)

def parse_args():
  parser = argparse.ArgumentParser(description="The Wombat build script")
  parser.parse_args()

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
      os.symlink(pkg, pkg_link_dest)

def build(args):
  # TODO: eventually do something with args
  execute("colcon build", WORKSPACE_DIR)

if __name__ == "__main__":
  args = parse_args()
  setup_workspace()
  build(args)
