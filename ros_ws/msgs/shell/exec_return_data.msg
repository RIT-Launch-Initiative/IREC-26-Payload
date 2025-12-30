int32 COMMAND_UNFINISHED_RETURN_CODE=2147483647 #indicates that you're asking for the results of a command that is still running
int32 COMMAND_NOT_RUN_RETURN_CODE=2147483646 #indicates that this command was not run for some reason (system resources)

# if return_code is COMMAND_UNFINISHED_RETURN_CODE, lengths (both compressed and uncompressed) are able to be changed between reads
int32 return_code # Shell exit code of process. OR special value noted above
uint16 stdout_len # length of captured stdout. 
uint16 compressed_stdout_len # length of captured stdout when gzip compressed
uint16 stderr_len # length of captured stderr
uint16 compressed_stderr_len # length of captured stderr when gzip compressed 
