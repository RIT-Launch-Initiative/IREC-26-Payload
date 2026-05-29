# SHOULD MATCH packets_p2g DEFINITIONS
uint8 telem_id
uint8 FLIGHT_HEARTBEAT = 0  # GPS Data, acceleration
uint8 LANDED_HEARTBEAT = 1  # FlightState, StatusBitset, Next Image ID, Next Command ID
uint8 ACTUATORS = 2         # Angles of joints, Angles of Servos, Stalls, Low precision Accelerometer readings
uint8 GNSS = 3              # Lat Long Altitude FixType Satellites
uint8 SYSTEM = 4            # CPU/RAM/Storage/Bootcount%256  (top -b -n 1 | head -n 5 , free, df)
uint8 ORIENTATIONS = 5      # High quality Accel. readings from all over
uint8 TEMPS = 6             # Temperatures from all over (CPU, STM, Board1, Board2, Ambient)
uint8 POWER = 7             # Battery Voltage, BatteryCurrent, Enabled Regulators
