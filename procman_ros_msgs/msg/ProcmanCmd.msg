# Uniquely identifies the command
string command_id

# Executable string.
string exec_str

# Logical group that the command belongs to.
string group

# If true and the command stops running on its own, then the deputy will
# automatically restart the command.
bool auto_respawn

# The POSIX signal number to send when stopping the command. Typically
# this is something like SIGINT or SIGTERM
int8 stop_signal

# How much time (seconds) to allow the command to gracefully terminate,
# when stopping the command, before issuing a SIGKILL.
int8 stop_time_allowed

