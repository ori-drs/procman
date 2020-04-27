# describes the state of a command managed by the procman sheriff/deputy

# The command to execute.
ProcmanCmd cmd

# If the command is running, then this is the pid if not, then this is
# zero.
int32 pid

# an ID for the run instance
int32 actual_runid

# the last exit code
int32 exit_code

# [0, 1]
float32 cpu_usage

# total virtual memory used by the process
int64 mem_vsize_bytes

# total physical memory used by the process
int64 mem_rss_bytes

