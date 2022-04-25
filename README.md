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
            - 'on=' + $time_on$ + 's' or '%'
                - $time_on$ -> float. In seconds or as percentage
            + ' p=' + $period$
                - $period$ -> Float. Sequence period in seconds
            + ' seq=' + $sequence$ + 's' or 'n'
                - $sequence$ -> Float. Sequence length in       seconds ('s') or as number of times ('n')
            + ' r=' + $respawn$
                - $respawn$ -> Respawn delay in seconds before repeating the sequence. -1 disables respawn feature.