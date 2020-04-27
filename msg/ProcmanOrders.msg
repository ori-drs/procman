# message sent by the procman sheriff to provide instructions for a procman
# deputy.

time timestamp
string deputy_id
string sheriff_id

int32 ncmds
ProcmanCmdDesired[] cmds

