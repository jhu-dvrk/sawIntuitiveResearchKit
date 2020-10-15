# Socket Streamer Configuration Files for the dVRK

Please check https://github.com/jhu-saw/sawSocketStreamer for details and usage.

These files are provided as examples to show how to configure the sawSocketStreamer with the dVRK core components.  The socket streamer can stream data from the dVRK using a UDP socket.  The data is sent in JSON format.  For examples:
```json
{"gripper/measured_js":{"AutomaticTimestamp":true,"Effort":null,"Name":["gripper"],"Position":[4.6701996844274162e-310],"Timestamp":19.699493094000001,"Valid":false,"Velocity":null},"measured_cp":{"AutomaticTimestamp":false,"MovingFrame":"MTML","Position":{"Rotation":[[3.6732051033050439e-06,0.99999999996626898,7.3464102068321324e-06],[-2.6984803280782899e-11,-7.3464102068321324e-06,0.99999999997301525],[0.99999999999325384,-3.6732051035270885e-06,0]],"Translation":[-1.3388794904003909e-06,-0.36449897370403511,-0.12879999999811512]},"ReferenceFrame":"MTML_base","Timestamp":19.699094908999999,"Valid":true},"measured_js":{"AutomaticTimestamp":false,"Effort":[0,0,0,0,0,0,0],"Name":["outer_yaw","shoulder_pitch","elbow_pitch","wrist_platform","wrist_pitch","wrist_yaw","wrist_roll"],"Position":[0,0,0,0,0,0,0],"Timestamp":19.699094908999999,"Valid":true,"Velocity":[0,0,0,0,0,0,0]}}
```

