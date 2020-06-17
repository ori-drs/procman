import argparse
import os
import signal
import subprocess
import sys
import time

from lcm import LCM
from procman_ros.sheriff_script import ScriptManager, ScriptListener
from procman_ros.sheriff import Sheriff
import procman_ros.sheriff as sheriff


class SheriffHeadless(ScriptListener):
    def __init__(self, lcm_obj, config, spawn_deputy, script_name, script_done_action):
        self.sheriff = Sheriff(lcm_obj)
        self.script_manager = ScriptManager(self.sheriff)
        self.spawn_deputy = spawn_deputy
        self.spawned_deputy = None
        self.config = config
        self.script_name = script_name
        self.script = None
        self.mainloop = None
        self.lcm_obj = lcm_obj
        self._should_exit = False
        if script_done_action is None:
            self.script_done_action = "exit"
        else:
            self.script_done_action = script_done_action

    def _shutdown(self):
        if self.spawned_deputy:
            print("Terminating local deputy..")
            try:
                self.spawned_deputy.terminate()
            except AttributeError:
                os.kill(self.spawned_deputy.pid, signal.SIGTERM)
                self.spawned_deputy.wait()
        self.spawned_deputy = None
        self.sheriff.shutdown()
        self.script_manager.shutdown()

    def _start_script(self):
        if not self.script:
            return False
        print("Running script {}".format(self.script_name))
        errors = self.script_manager.execute_script(self.script)
        if errors:
            print("Script failed to run.  Errors detected:\n" + "\n".join(errors))
            self._shutdown()
            sys.exit(1)
        return False

    def script_finished(self, script_object):
        # Overriden from ScriptListener. Called by ScriptManager when a
        # script is finished.
        if self.script_done_action == "exit":
            self._request_exit()
        elif self.script_done_action == "observe":
            self.sheriff.set_observer(True)

    def _request_exit(self):
        self._should_exit = True

    def run(self):
        # parse the config file
        if self.config is not None:
            self.sheriff.load_config(self.config)
            self.script_manager.load_config(self.config)

        # start a local deputy?
        if self.spawn_deputy:
            args = ["rosrun", "procman_ros", "procman_ros_deputy", "-i", "localhost"]
            self.spawned_deputy = subprocess.Popen(args)
        else:
            self.spawned_deputy = None

        # run a script
        if self.script_name:
            self.script = self.script_manager.get_script(self.script_name)
            if not self.script:
                print("No such script: {}".format(self.script_name))
                self._shutdown()
                sys.exit(1)
            errors = self.script_manager.check_script_for_errors(self.script)
            if errors:
                print("Unable to run script.  Errors were detected:\n\n")
                print("\n    ".join(errors))
                self._shutdown()
                sys.exit(1)

            self.script_manager.add_listener(self)

        signal.signal(signal.SIGINT, lambda *s: self._request_exit())
        signal.signal(signal.SIGTERM, lambda *s: self._request_exit())
        signal.signal(signal.SIGHUP, lambda *s: self._request_exit())

        try:
            if self.script:
                time.sleep(0.2)
                self._start_script()

            while not self._should_exit:
                self.lcm_obj.handle_timeout(200)
        except KeyboardInterrupt:
            pass
        except OSError:
            pass
        finally:
            print("Sheriff terminating..")
            self._shutdown()

        return 0


def main():
    parser = argparse.ArgumentParser(
        description="Process management operating console.",
        epilog="If procman_config_file is specified, then the sheriff tries to load "
        "deputy commands from the file.\n\nIf script_name is additionally "
        "specified, then the sheriff executes the named script once the config "
        "file is loaded.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument("procman_config_file", help="The configuration file to load")

    parser.add_argument(
        "--script", help="A script to execute after the config file is loaded."
    )

    parser.add_argument(
        "-l",
        "--lone-ranger",
        action="store_true",
        dest="spawn_deputy",
        help="Automatically run a deputy within the sheriff process. This deputy terminates with the "
        "sheriff, along with all the commands it hosts.",
    )
    parser.add_argument(
        "-o",
        "--observer",
        action="store_true",
        help="Runs in observer mode on startup.  This "
        "prevents the sheriff from sending any "
        "commands, and is useful for monitoring "
        "existing procman_ros sheriff and/or deputy "
        "instances.",
    )
    parser.add_argument(
        "--on-script-complete",
        choices=["exit", "observer"],
        dest="script_done_action",
        help='Only valid if a script is specified.  If set to "exit", then the sheriff exits when '
        'the script is done executing. If set to "observe", then the sheriff self-demotes to '
        "observer mode.",
    )

    args = parser.parse_args(sys.argv[1:])

    try:
        cfg = sheriff.load_config_file(file(args.procman_config_file))
    except Exception as xcp:
        print("Unable to load config file.")
        print(xcp)
        sys.exit(1)

    if args.observer:
        if cfg:
            print(
                "Loading a config file is not allowed when starting in observer mode."
            )
            sys.exit(1)
        if args.spawn_deputy:
            print("Lone ranger mode and observer mode are mutually exclusive.")
            sys.exit(1)

    lcm_obj = LCM()

    SheriffHeadless(lcm_obj, cfg, args.spawn_deputy, args.script, args.script_done_action).run()


if __name__ == "__main__":
    main()
