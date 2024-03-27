# message sent by a procman deputy, primarily intended for the procman
# sheriff.  informs the sheriff of the status of processes running on the
# host managed by the deputy.

builtin_interfaces/Time timestamp
string deputy_id

# [0, 1]
float32 cpu_load

int64 phys_mem_total_bytes
int64 phys_mem_free_bytes
int64 swap_total_bytes
int64 swap_free_bytes

int32 ncmds
ProcmanCmdStatus[] cmds

