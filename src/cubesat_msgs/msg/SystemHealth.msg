builtin_interfaces/Time stamp

# true if the node has published within the configured silence timeout
bool pi_io_ok
bool stm_bridge_ok
bool radio_ok

# seconds since the last message was heard from each node
# (negative if never heard since captain start)
float32 pi_io_silent_s
float32 stm_bridge_silent_s
float32 radio_silent_s
