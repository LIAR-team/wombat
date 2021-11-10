#!/usr/bin/python3

import argparse
import distro
import os
from pathlib import Path
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
  formatter = lambda prog: argparse.RawDescriptionHelpFormatter(prog, max_help_position=52)
  parser = argparse.ArgumentParser(
    formatter_class=formatter,
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
Test without linters      ./build.py test --no-lint
'''
)

  general_args = parser.add_argument_group(bcolors.header("General Arguments"))
  general_args.add_argument("-v", "--verbose", help="prints verbose logs to screen", action="store_true")
  general_args.add_argument("--reset", help="Reset CMake cache", action="store_true")
  general_args.add_argument("--reset-hard", help="Completely erases the wombat workspace", action="store_true")
  general_args.add_argument("--coverage", help="Compute test coverage metrics", action="store_true")

  test_args = parser.add_argument_group(bcolors.header("Test Arguments"))
  test_args.add_argument("--test-name", help="Run only unit-tests that match this regex", metavar="<regex>")
  test_args.add_argument("--no-test-name", help="Exclude unit-tests that match this regex", metavar="<regex>")
  test_args.add_argument("--no-lint", help="Exclude Linters unit-tests", action="store_true")

  wombat_args, colcon_args = parser.parse_known_args()
  return wombat_args, colcon_args

###
# Utilities
###

def execute(command, cwd=WOMBAT_DIR, stdout=None, stderr=None, dry_run=False):
  '''
  Runs the provided command in a new shell.
  The command is called from the specified directory.
  By default the command output (stdout and stderr) is redirected to terminal.
  It returns a CompletedProcess instance.
  '''
  
  print('+ ' + command)
  actual_command=command if not dry_run else '/bin/true'
  cp = subprocess.run(actual_command, shell=True, cwd=cwd, universal_newlines=True, stdout=stdout, stderr=stderr)
  return cp

def has_custom_colcon_path(args):
  return any(x in args for x in ["--base-path", "--base-paths", "--build-base", "--install-base", "--log-base"])

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

    cmd_prefix = ""

    # Always set the base path as we run commands in the workspace directory
    build_args = ["--base-paths", WOMBAT_DIR]

    # We always pass output to stdout/stderr unless user has their own options
    event_handlers_keyword = "--event-handlers"
    if not event_handlers_keyword in self.colcon_args:
      build_args.extend([event_handlers_keyword, "console_direct+"])

    # If user requests verbose mode then print all build invocations
    if self.wombat_args.verbose:
      cmd_prefix = "VERBOSE=1"

    cmake_args = [
      "-DCMAKE_EXPORT_COMPILE_COMMANDS=True", # Compile commands are needed to run clang-tidy linters
      "-GNinja" # Use Ninja backend because to speed-up builds
    ]

    # Build args must be added after the `build` keyword
    extend_at_value(self.colcon_args, self.BUILD_KEYWORD, build_args)

    # CMake args must be added at the end
    if cmake_args:
      cmake_keyword = "--cmake-args"
      default_cmake_index = len(self.colcon_args)
      extend_at_value(self.colcon_args, cmake_keyword, cmake_args, default_cmake_index)

    colcon_args_joint = " ".join(self.colcon_args)
    execute(f"{cmd_prefix} colcon {colcon_args_joint}", WORKSPACE_DIR)

class TestWorkflow(BaseWorkflow):
  TEST_KEYWORD = "test"
  # NOTE: this regex needs to be manually updated to match our linters
  LINTERS_REGEX = "\"(clang|flake|lint|uncrustify)\""

  def run(self):
    if not os.path.exists(WORKSPACE_DIR):
      raise OSError("Workspace directory is not present: build before running tests.")

    ctest_args = []

    # Always set the base path as we run commands in the workspace directory
    test_args = ["--base-paths", WOMBAT_DIR]

    # If user requests verbose mode redirect colcon logs to console
    event_handlers_keyword = "--event-handlers"
    if self.wombat_args.verbose and not event_handlers_keyword in self.colcon_args:
      test_args.extend([event_handlers_keyword, "console_direct+"])

    # Match the provided tests
    if self.wombat_args.test_name:
      ctest_args.extend(["-R", self.wombat_args.test_name])

    # Exclude the provided tests
    if self.wombat_args.no_test_name:
      ctest_args.extend(["-E", self.wombat_args.no_test_name])

    # Exclude linter tests
    if self.wombat_args.no_lint:
      ctest_args.extend(["-E", self.LINTERS_REGEX])

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

class CoverageWorkflow(BaseWorkflow):
  def run(self):
    if not os.path.exists(WORKSPACE_DIR):
      raise OSError("Workspace directory is not present: build and run tests before computing coverage.")

    build_dir = os.path.join(WORKSPACE_DIR, "build")

    if not list(Path(build_dir).rglob("*.gcno")):
      raise OSError("Code must be built with --cmake-args -DCOVERAGE_ENABLED=True before computing coverage.")
    
    if not list(Path(build_dir).rglob("*.gcda")):
      raise OSError("Code must be tested before computing coverage.")

    # Get names of all ROS packages in workspace
    list_pkg_command = f"colcon list --names-only --base-paths {WOMBAT_DIR}"
    cp = execute(list_pkg_command, cwd=WORKSPACE_DIR, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    cp.check_returncode()
    packages = cp.stdout.strip().replace('\n',' ')

    # Generage a coverage info file
    cov_info_file = os.path.join(WORKSPACE_DIR, "coverage.info")
    execute(f"fastcov --lcov -d {build_dir} --include {packages} --exclude test/ --process-gcno --validate-sources --dump-statistic --output {cov_info_file}", WORKSPACE_DIR)
    cp.check_returncode()

    # Generage html visualization
    cov_html_dir = os.path.join(WORKSPACE_DIR, "cov_html")
    execute(f"genhtml {cov_info_file} --output-directory {cov_html_dir}", WORKSPACE_DIR)
    cp.check_returncode()

    print(f"\n---> Find coverage report at {cov_html_dir}")

def create_workflow(wombat_args, colcon_args):
  workflow = None

  if BuildWorkflow.BUILD_KEYWORD in colcon_args:
    workflow = BuildWorkflow(wombat_args, colcon_args)
  elif TestWorkflow.TEST_KEYWORD in colcon_args:
    workflow = TestWorkflow(wombat_args, colcon_args)
  elif wombat_args.reset or wombat_args.reset_hard:
    workflow = ResetWorkflow(wombat_args, colcon_args)
  elif wombat_args.coverage:
    workflow = CoverageWorkflow(wombat_args, colcon_args)
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
