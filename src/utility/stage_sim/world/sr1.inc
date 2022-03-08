define lidar ranger
(
  sensor(
    # laser-specific properties
    # factory settings for UTM30LXEW
    range [ 0.0  25.0 ]
    fov 210.0
    samples 1080
    resolution 2
  )
  model
  (
    # generic model properties
    size [ 0.0 0.0 0.0 ]
    color "blue"
  )
)

define qr_reader fiducial
(
  # fiducial properties
  name "marker_reader"
  range_min 0.0
  range_max 2.0
  range_max_id 2.0
  fov 110.0
  ignore_zloc 0  #  When set to 1, the fiducial finder ignores the z component when checking a fiducial
  size [ 0.05 0.05 0.05 ]
  color "blue"
)

define charger_reader fiducial
(
  # fiducial properties
  name "lidar_reader"
  range_min 0.0
  range_max 2.0
  range_max_id 2.0
  fov 110.0
  ignore_zloc 0 #  When set to 1, the fiducial finder ignores the z component when checking a fiducial
  size [ 0.05 0.05 0.05 ]
  color "blue"
)

# Robot Definition
define sr1 position
(
    # localization "odom"
    # pose [-0.4 -0.4 0.0 0.0]
    # odom_error [0.03 0.03 0 0 0 0.02]
    size [0.60 0.60 1.5]
    gui_nose 1
    drive "omni"
    color "blue"
    blob_return 1                 # Seen by blobfinders
    fiducial_return 0             # Seen as "1" fiducial finders
    lidar (pose [ 0.3 0.0 -1.28 0.0 ])
    qr_reader( pose [ 0.25 0.0 -0.6 0.0 ] )
    charger_reader( pose [ 0.25 0.0 -1.28 0.0 ] )
)
