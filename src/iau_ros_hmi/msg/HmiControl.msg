time    stamp             # timestamp
int32   e_stop            # emergency stop flag
float32 speed             # speed in m/s
float32 ang_velo          # turn angle velocity deg/s
float32 distance          # run distance in m
float32 angle             # turn angle in degree(L:+/R:-)
int32   control_flag      # is start the remote control 0 means free ，1 means pause ,2 means stop ,3 means remote
int32   action_flag       # 0 means none ,1 means apply mitionPoint ,2 means navigation
int32   mition_num        # mitionPoint number
float32[] mition_point_x  # mitionPoint x
float32[] mition_point_y  # mitionPoint y
float32[] mition_point_speed  # mitionPoint speed
int32[]   mition_point_a  # mitionPoint attribute
float32 origin_x
float32 origin_y
float32 origin_z
float32 origin_yaw
bool    mition_finish     # is finishi mition