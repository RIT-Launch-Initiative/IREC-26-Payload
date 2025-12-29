New Flight corresponds to making a new directory

```
├── 0                               Directory for 0th flight. If a new flight is commanded, a directory called 1/ is created
│   ├── commands                        Shell commands executed by radio control. Each new command gets a new directory
│   │   └── 0                               0th Shell command 
│   │       ├── command                     The value passed to a shell to execute
│   │       ├── date                        The wall-clock time the command was executed, as milliseconds since epoch
│   │       ├── exit_code                   The exit code of the command, as 32 bit signed int
│   │       ├── stderr                      Capture of stderr from the command
│   │       ├── stderr.gz                   Compressed version of stderr
│   │       ├── stdout                      Capture of stdout from the command
│   │       └── stdout.gz                   Compressed version of stdout
│   ├── expected                        Flag file indicating that the 'expect flight' command was heard and that we're in fast-telem mode (maybe)
│   ├── images                          Directory for images taken
│   │   └── 0                               0th image taken
│   │       ├── actuators                   angles of all joints at the time of the picture
│   │       ├── camera_settings             camera settings for the picture (exposure, f. stop, etc)
│   │       ├── date                        time of picture taken as milliseconds since epoch
│   │       ├── original.png                the original, uncropped, uncompressed picture
│   │       ├── parameters                  parameters about the transmitted part of the picture (crop or digital zoom)
│   │       ├── position                    GPS location where the picture was taken
│   │       ├── scaled.png                  the scaled and cropped image according to the parameters file, uncompressed
│   │       └── uptime                      the time of the picture taken as milliseconds uptime
│   ├── landed                          Flag file indicating that the landing timer has expired
│   ├── launched                        Flag file indicating that boost has been detected or forced
│   ├── logs                            Human readable logs of what the subsystems are doing
│   │   ├── actuators.log                   Example 
│   │   ├── cpu.log
│   │   └── flight.log
│   ├── radio                           Directory for radio logging / handling - radio subsystem has exclusive write access
│   │   ├── received                    Directory of packets received from the ground station for later analysis 
│   │   │   └── 0                           0th packet received
│   │   │       ├── date                    wall clock time this packet was received, milliseconds since epoch
│   │   │       ├── uptime                  uptime that this packet was received
│   │   │       ├── packet                  raw binary from the radio
│   │   │       ├── rssi                    RSSI of received packet
│   │   │       └── snr                     SNR of received packet
│   │   └── sent                        
│   │       └── 0                       Directory of packets sent to the ground station for later analysis
│   │           ├── date                    Wall clock time the packet was sent, milliseconds since epoch
│   │           ├── packet                  Raw binary sent on air
│   │           └── settings                Radio settings (SF, BW, CR, Freq) that this packet was sent at
│   └── videos                          Directory of videos taken - video subsystem has exclusive write access
│       └── 0                               0th video taken
│           ├── date                        Wall clock time that the video was started, milliseconds since epoch
│           ├── raw.mp4                     Video file
│           └── uptime                      Uptime that the video was started, milliseconds since boot
└── README.md
```