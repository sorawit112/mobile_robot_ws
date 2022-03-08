define lidar ranger
(
  sensor(
    # laser-specific properties
    # factory settings for LMS200
    range [ 0.0  2.0 ]
    fov 270.0
    samples 1080
  )
)

define cam_front camera
(
      origin [0.0 0.0 0.5 0.0]
      # laser properties
      resolution [ 640 480 ]
      range [ 0.2 30.0 ]
      fov [ 100.0 75.0 ]
      pantilt [ 0.0 25.0 ]

      # model properties
      size [ 0.1 0.1 0.05 ]
      color "black"

)

# Robot Definition
define sr1 position
(
    pose [0.0 0.0 0.0 0.0]
    origin [ 0.0 0.0 0.0 0.0 ]
    odom_error [0.03 0.03 999999 999999 999999 0.02]
    size [0.7 0.7 1.5]
    gui_nose 1
    drive "diff"
    color "blue"
    lidar (pose [ 0.0 0.0 1.28 0.0 ])
    # cam_front (pose [0.0 0.0 0.5 0.0])
)
