# A message for Ubiquiti Radios
# Publish a different topic for each "remote" station reported.
# Duplicate the local radio information in each.

Header header

string          raw_status
# Local radio information
string          local_hostname
float32         local_uptime
float32         local_temperature
float32         local_frequency
float32         local_distance
float32         local_selfnoisedB
float32         local_txpower
float32         local_channelbw
float32         local_tx_throughput
float32         local_rx_throughput

# Remote Station Link information
#     self.status['wireless']['sta'][0]['remote']['hostname']
string          remote_hostname
float32         remote_uplink_capacity
float32         remote_downlink_capacity
float32         remote_signal
float32         remote_rssi
float32         remote_noisefloor

float32         remote_temperature
float32         remote_uptime
float32         remote_tx_throughput
float32         remote_rx_throughput
float32         remote_tx_power
float32         remote_distance
