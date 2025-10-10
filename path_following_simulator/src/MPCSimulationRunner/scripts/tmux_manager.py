#!/usr/bin/env python3
"""
TmuxManager: Manages tmux sessions for ROS simulation experiments
Provides clean session management, pane creation, command execution, and monitoring
"""

import subprocess
import time
import re

class TmuxManager:
    """
    Manages tmux sessions for running ROS simulation components
    """

    def __init__(self, session_name="mpc_sim_session"):
        """
        Initialize TmuxManager with a session name

        Args:
            session_name (str): Name of the tmux session
        """
        self.session_name = session_name
        self.panes = {}  # Maps pane_name -> pane_id (window:pane format)
        self.session_exists = False

    def _run_tmux_command(self, command, capture_output=True):
        """
        Execute a tmux command

        Args:
            command (list): Command to execute
            capture_output (bool): Whether to capture output

        Returns:
            subprocess.CompletedProcess or None
        """
        try:
            if capture_output:
                result = subprocess.run(
                    command,
                    capture_output=True,
                    text=True,
                    check=False
                )
                return result
            else:
                subprocess.run(command, check=False)
                return None
        except Exception as e:
            print("Error running tmux command {}: {}".format(command, e))
            return None

    def start_session(self):
        """
        Start a new tmux session (kills existing session if it exists)

        Returns:
            bool: True if successful, False otherwise
        """
        # Kill existing session if it exists
        self.kill_session()

        # Create new detached session with a default window
        result = self._run_tmux_command([
            'tmux', 'new-session', '-d', '-s', self.session_name, '-n', 'main'
        ])

        if result and result.returncode == 0:
            self.session_exists = True
            # Store the first pane as "main"
            self.panes['main'] = "{}:0.0".format(self.session_name)
            print("Created tmux session: {}".format(self.session_name))
            return True
        else:
            print("Failed to create tmux session: {}".format(self.session_name))
            return False

    def create_pane(self, pane_name, split_from='main', split_direction='h'):
        """
        Create a new pane by splitting an existing pane

        Args:
            pane_name (str): Name to assign to the new pane
            split_from (str): Name of the pane to split from
            split_direction (str): 'h' for horizontal split, 'v' for vertical split

        Returns:
            bool: True if successful, False otherwise
        """
        if not self.session_exists:
            print("Error: No active tmux session")
            return False

        if split_from not in self.panes:
            print("Error: Pane '{}' does not exist".format(split_from))
            return False

        # Split the pane
        split_option = '-h' if split_direction == 'h' else '-v'
        result = self._run_tmux_command([
            'tmux', 'split-window', split_option,
            '-t', self.panes[split_from]
        ])

        if result and result.returncode == 0:
            # Get the pane ID of the newly created pane
            # The new pane is always the last pane in the window
            pane_list_result = self._run_tmux_command([
                'tmux', 'list-panes', '-t', self.session_name,
                '-F', '#{pane_id}'
            ])

            if pane_list_result and pane_list_result.returncode == 0:
                pane_ids = pane_list_result.stdout.strip().split('\n')
                new_pane_id = pane_ids[-1]  # Last pane is the newly created one
                self.panes[pane_name] = new_pane_id
                print("Created pane '{}' (ID: {})".format(pane_name, new_pane_id))
                return True

        print("Failed to create pane '{}'".format(pane_name))
        return False

    def send_command(self, pane_name, command, wait_for_enter=True):
        """
        Send a command to a specific pane

        Args:
            pane_name (str): Name of the pane
            command (str): Command to send
            wait_for_enter (bool): Whether to send Enter key after command

        Returns:
            bool: True if successful, False otherwise
        """
        if pane_name not in self.panes:
            print("Error: Pane '{}' does not exist".format(pane_name))
            return False

        pane_id = self.panes[pane_name]

        # Send the command
        result = self._run_tmux_command([
            'tmux', 'send-keys', '-t', pane_id, command
        ])

        if wait_for_enter:
            # Send Enter key
            self._run_tmux_command([
                'tmux', 'send-keys', '-t', pane_id, 'Enter'
            ])

        if result and result.returncode == 0:
            print("Sent command to '{}': {}".format(pane_name, command))
            return True
        else:
            print("Failed to send command to '{}'".format(pane_name))
            return False

    def wait_for_pattern(self, pane_name, pattern, timeout=30, check_interval=1):
        """
        Wait for a specific pattern to appear in pane output

        Args:
            pane_name (str): Name of the pane to monitor
            pattern (str): Regex pattern to search for
            timeout (int): Maximum time to wait in seconds
            check_interval (float): Time between checks in seconds

        Returns:
            bool: True if pattern found, False if timeout
        """
        if pane_name not in self.panes:
            print("Error: Pane '{}' does not exist".format(pane_name))
            return False

        pane_id = self.panes[pane_name]
        pattern_re = re.compile(pattern)
        start_time = time.time()

        print("Waiting for pattern '{}' in pane '{}'...".format(pattern, pane_name))

        while (time.time() - start_time) < timeout:
            # Capture pane content
            result = self._run_tmux_command([
                'tmux', 'capture-pane', '-t', pane_id, '-p'
            ])

            if result and result.returncode == 0:
                output = result.stdout
                if pattern_re.search(output):
                    print("Pattern '{}' found in pane '{}'".format(pattern, pane_name))
                    return True

            time.sleep(check_interval)

        print("Timeout waiting for pattern '{}' in pane '{}'".format(pattern, pane_name))
        return False

    def get_pane_output(self, pane_name, last_n_lines=None):
        """
        Get the current output from a pane

        Args:
            pane_name (str): Name of the pane
            last_n_lines (int): Number of last lines to return (None for all)

        Returns:
            str: Pane output or None if error
        """
        if pane_name not in self.panes:
            print("Error: Pane '{}' does not exist".format(pane_name))
            return None

        pane_id = self.panes[pane_name]

        # Capture pane content
        result = self._run_tmux_command([
            'tmux', 'capture-pane', '-t', pane_id, '-p'
        ])

        if result and result.returncode == 0:
            output = result.stdout
            if last_n_lines:
                lines = output.split('\n')
                return '\n'.join(lines[-last_n_lines:])
            return output

        return None

    def kill_session(self):
        """
        Kill the tmux session and clean up

        Returns:
            bool: True if successful or session doesn't exist, False otherwise
        """
        if not self.session_exists and not self._session_exists_check():
            print("No session to kill")
            return True

        result = self._run_tmux_command([
            'tmux', 'kill-session', '-t', self.session_name
        ])

        if result is None or result.returncode == 0:
            self.session_exists = False
            self.panes = {}
            print("Killed tmux session: {}".format(self.session_name))
            return True
        else:
            print("Failed to kill tmux session: {}".format(self.session_name))
            return False

    def _session_exists_check(self):
        """
        Check if the tmux session exists

        Returns:
            bool: True if session exists, False otherwise
        """
        result = self._run_tmux_command([
            'tmux', 'has-session', '-t', self.session_name
        ])

        return result is not None and result.returncode == 0

    def send_keys(self, pane_name, keys):
        """
        Send specific keys to a pane (e.g., 'C-c' for Ctrl+C)

        Args:
            pane_name (str): Name of the pane
            keys (str): Keys to send (tmux format, e.g., 'C-c', 'Enter')

        Returns:
            bool: True if successful, False otherwise
        """
        if pane_name not in self.panes:
            print("Error: Pane '{}' does not exist".format(pane_name))
            return False

        pane_id = self.panes[pane_name]

        result = self._run_tmux_command([
            'tmux', 'send-keys', '-t', pane_id, keys
        ])

        if result and result.returncode == 0:
            print("Sent keys '{}' to pane '{}'".format(keys, pane_name))
            return True
        else:
            print("Failed to send keys to pane '{}'".format(pane_name))
            return False

    def list_panes(self):
        """
        List all panes in the current session

        Returns:
            dict: Dictionary of pane_name -> pane_id
        """
        return self.panes.copy()

    def attach_session(self):
        """
        Attach to the tmux session (for debugging)
        Note: This will block until user detaches
        """
        if not self.session_exists:
            print("Error: No active tmux session")
            return False

        subprocess.run(['tmux', 'attach-session', '-t', self.session_name])
        return True


# Example usage
if __name__ == '__main__':
    # Create a tmux manager
    tmux = TmuxManager("test_session")

    # Start session
    tmux.start_session()

    # Create panes
    tmux.create_pane("roscore", split_from="main", split_direction='h')
    tmux.create_pane("gazebo", split_from="main", split_direction='v')

    # Send commands
    tmux.send_command("roscore", "roscore")
    time.sleep(2)

    tmux.send_command("gazebo", "echo 'Gazebo would start here'")

    # Wait for pattern
    tmux.wait_for_pattern("roscore", "started core service", timeout=10)

    # Get output
    print("\n=== Roscore Output ===")
    print(tmux.get_pane_output("roscore", last_n_lines=10))

    # Clean up
    print("\nCleaning up in 5 seconds...")
    time.sleep(5)
    tmux.kill_session()
