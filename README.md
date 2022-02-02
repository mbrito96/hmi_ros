# hmi_ros
 Basic HMI functionality for the Jetson Nano

To explain:
- Input and output configuration -> from ROS parameter server
- Output order 
    - In topic node_name/output_order
    - According to outCmd.msg type
        - outputs[]: list of outputs for the order to apply
        - type: TOGGLE, SET or CLEAR
        - params: string to indicate the output sequence (only applicable for type TOGGLE)
            - 'tOn=%d' -> time_on = 100
            + 's' or '%' -> Time on unit
            + ';p=%d' -> Period in seconds
            + ';seq=%d' -> Sequence length
            + 's' or 'n' -> Sequence unit. Repeat toggle for $seq seconds or $seq times
            + ';r=%d' -> Respawn delay in seconds. -1 disables respawn feature.