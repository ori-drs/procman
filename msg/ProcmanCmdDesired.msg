# describes the state of a command managed by the procman sheriff/deputy
# ============ set by the sheriff ==========

# describes the command
ProcmanCmd cmd

# to start a command running, the sheriff should change desired_runid and
# unset force_quit
int32 desired_runid

# flag set by the sheriff.  If set, then the deputy should forcefully
# terminate the command
bool force_quit

