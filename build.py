#!/usr/bin/python3

import argparse
import distro
from enum import Enum
import os
import platform
import shutil
import subprocess

###
# Constants
###

WOMBAT_DIR = os.path.abspath(os.path.dirname(os.path.realpath(__file__)))
WORKSPACE = "_ws"
WORKSPACE_DIR = os.path.join(WOMBAT_DIR, WORKSPACE)

# https://stackoverflow.com/questions/287871/how-to-print-colored-text-in-terminal-in-python
class bcolors:
  BLUE = '\033[34m'
  GREEN = '\033[32m'
  LIGHT_MAGENTA = '\033[95m'
  LIGHT_BLUE = '\033[94m'
  LIGHT_GREEN = '\033[92m'
  LIGHT_YELLOW = '\033[93m'
  LIGHT_RED = '\033[91m'
  ENDC = '\033[0m'
  BOLD = '\033[1m'
  DIM = '\033[2m'
  ITALIC = '\033[3m'
  UNDERLINE = '\033[4m'
  INVERT = '\033[7m'

  @staticmethod
  def header(txt):
    return bcolors.BOLD +  bcolors.UNDERLINE + txt + bcolors.ENDC
  
  @staticmethod
  def warning(txt):
    return bcolors.BOLD + bcolors.LIGHT_YELLOW + txt + bcolors.ENDC

###
# Command line arguments
###

def parse_args():
  parser = argparse.ArgumentParser(
    formatter_class=argparse.RawDescriptionHelpFormatter,
    description=bcolors.header("The wombat Build Script.") +
'''
Treat this script as an equivalent of `colcon`, which is meant to simplify the most standard build operations.
In addition to the optional arguments mentioned below, it supports all regular colcon arguments.
Run `colcon --help` to see all the colcon supported arguments.

'''
+ bcolors.header("CheatSheet") +
'''
Build All                 ./build.py
Build up to a package     ./build.py --packages-up-to wombat_hello_world
Build only a package      ./build.py --packages-select wombat_hello_world
Test All                  ./build.py test
Test only a package       ./build.py test --packages-select wombat_hello_world
'''
)

  parser.add_argument("-v", "--verbose", help="prints verbose logs to screen", action="store_true")
  parser.add_argument("--test-name", help="Build and run unit-tests that match this regex", metavar="<regex>")
  parser.add_argument("--reset", help="Reset CMake cache", action="store_true")
  parser.add_argument("--reset-hard", help="Completely erases the wombat workspace", action="store_true")

  wombat_args, colcon_args = parser.parse_known_args()
  return wombat_args, colcon_args

###
# Utilities
###

def execute(command, cwd=WOMBAT_DIR, dry_run=False):
  print('+ ' + command)
  actual_command=command if not dry_run else '/bin/true'
  p = subprocess.Popen(actual_command, shell=True, cwd=cwd, stdout=None, stderr=None)
  return p.wait()

def has_custom_colcon_path(args):
  return "--base-path" in args or "--base-paths" in args or "--build-base" in args or "--install-base" in args or "--log-base" in args

def extend_at_value(to_be_extended, value, extension, default_value_index=0):
  # inserts a list as individual elements into another list after the specified value
  # if the list does not include the value, it will first insert it at default_value_index

  # make sure that value is included in the list to be extended
  if not value in to_be_extended:
    to_be_extended.insert(default_value_index, value)
  # get index immediately after value
  extension_index = to_be_extended.index(value) + 1
  # inserts a list as individual elements into another list at the specified index
  to_be_extended[extension_index:extension_index] = extension

###
# Workflows
###

class BaseWorkflow():
  def __init__(self, wombat_args, colcon_args):
    self.wombat_args = wombat_args
    self.colcon_args = colcon_args

class BuildWorkflow(BaseWorkflow):
  BUILD_KEYWORD = "build"

  def run(self):
    # Make sure that the workspace directory exists
    if not os.path.exists(WORKSPACE_DIR):
      os.makedirs(WORKSPACE_DIR)

    # Always set the base path as we run commands in the workspace directory
    build_args = ["--base-paths", WOMBAT_DIR]

    # If user requests verbose mode redirect colcon logs to console
    event_handlers_keyword = "--event-handlers"
    if self.wombat_args.verbose and not event_handlers_keyword in self.colcon_args:
      build_args.extend([event_handlers_keyword, "console_direct+"])

    # Build args must be added after the `build` keyword
    extend_at_value(self.colcon_args, self.BUILD_KEYWORD, build_args)

    colcon_args_joint = " ".join(self.colcon_args)
    execute(f"colcon {colcon_args_joint}", WORKSPACE_DIR)

class TestWorkflow(BaseWorkflow):
  TEST_KEYWORD = "test"

  def run(self):
    ctest_args = []

    # Always set the base path as we run commands in the workspace directory
    test_args = ["--base-paths", WOMBAT_DIR]

    # If user requests verbose mode redirect colcon logs to console
    event_handlers_keyword = "--event-handlers"
    if self.wombat_args.verbose and not event_handlers_keyword in self.colcon_args:
      test_args.extend([event_handlers_keyword, "console_direct+"])

    if self.wombat_args.test_name:
      ctest_args.extend(["-R", self.wombat_args.test_name])

    # Test args must be added after the `test` keyword
    extend_at_value(self.colcon_args, self.TEST_KEYWORD, test_args)

    # Ctest args must be added at the end
    if ctest_args:
      ctest_keyword = "--ctest-args"
      default_ctest_index = len(self.colcon_args)
      extend_at_value(self.colcon_args, ctest_keyword, ctest_args, default_ctest_index)

    # Run colcon test
    colcon_args_joint = " ".join(self.colcon_args)
    execute(f"colcon {colcon_args_joint}", WORKSPACE_DIR)

class ResetWorkflow(BaseWorkflow):
  def run(self):
    if not os.path.exists(WORKSPACE_DIR):
      print(bcolors.warning("Workspace does not exist, nothing to reset!"))
      return
    
    if self.wombat_args.reset_hard:
      shutil.rmtree(WORKSPACE_DIR)
      return
    
    if self.wombat_args.reset:
      cmake_cache_name = "CMakeCache.txt"
      build_dir = os.path.join(WORKSPACE_DIR, "build")
      package_dirs = [os.path.join(build_dir, name) for name in os.listdir(build_dir)]
      for dir in package_dirs:
        cmake_cache_file = os.path.join(dir, cmake_cache_name)
        if os.path.exists(cmake_cache_file):
          os.remove(cmake_cache_file)
      return

def create_workflow(wombat_args, colcon_args):
  workflow = None

  if BuildWorkflow.BUILD_KEYWORD in colcon_args:
    workflow = BuildWorkflow(wombat_args, colcon_args)
  elif TestWorkflow.TEST_KEYWORD in colcon_args:
    workflow = TestWorkflow(wombat_args, colcon_args)
  elif wombat_args.reset or wombat_args.reset_hard:
    workflow = ResetWorkflow(wombat_args, colcon_args)
  else:
    workflow = BuildWorkflow(wombat_args, colcon_args)

  return workflow

if __name__ == "__main__":

  if platform.system() != 'Linux' or distro.linux_distribution(full_distribution_name=False)[1] != '20.04':
    print(bcolors.warning("You are not running the default supported OS: Ubuntu 20.04."))

  wombat_args, colcon_args = parse_args()

  if has_custom_colcon_path(colcon_args):
    raise ValueError("You can't specify colcon paths when using build.py")

  workflow = create_workflow(wombat_args, colcon_args)
  workflow.run()
