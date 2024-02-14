# message sent by a procman deputy or sheriff to discover other deputies and
# sheriffs on the network.

builtin_interfaces/Time timestamp

# Id of the transmitter. If the tranmsitter is a deputy, then this is the
# deputy id. If the transmitter is a sheriff, then this is the sheriff id.
string transmitter_id

int64 nonce

