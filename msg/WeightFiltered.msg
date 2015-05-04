# This message is used to give filtered scales output showing 
# last time weight changed
# positive/ negavite change 1/0
# current weight
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')

# here shows last time weight change has been measuered
time change_time

# occured a change now
bool weight_changed

# current stable weigth measure
float64 weight_stable

# weight difference between last stable weigth
float64 weight_change
