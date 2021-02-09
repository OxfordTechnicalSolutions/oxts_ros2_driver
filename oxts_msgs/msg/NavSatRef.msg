# OxTS local coordinate reference frame

std_msgs/Header header      # Standard ROS message header
float64 latitude            # Latitude [degrees]. Positive is north of equator; negative is south.
float64 longitude           # Longitude [degrees]. Positive is east of prime meridian; negative is west.
float64 altitude            # Altitude [m]. Positive is ablve the WGS 84 ellipsoid
float64 heading             # Heading [degrees]. Anti-clockwise positive arounf the up axis; east is 0