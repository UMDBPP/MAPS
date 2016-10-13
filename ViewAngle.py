def ViewAngle(origin_coordinates_lla, target_coordinates_lla):
#   calculates azimuth_angle (horizontal angle) and elevation_angle (vertical angle) to a point given an origin and target
#       from algorithm found at http://gis.stackexchange.com/questions/58923/calculate-view-angle
#
#       input:
#       origin - 3 tuple of (lat [deg], lng [deg], alt[m]) giving location of observer
#       target - 3 tuple of (lat [deg], lng [deg], alt[m]) giving location of target
#
#       output:
#       2 element tuple contining azimuth_angle,elevation_angle in deg

    import math

    # convert origin coordinates from LLA to ECEF
    origin_coordinates_ecef = CoordinatesToECEF(origin_coordinates_lla[0], origin_coordinates_lla[1], origin_coordinates_lla[2]);

    # convert target coordinates from LLA to ECEF
    target_coordinates_ecef = CoordinatesToECEF(target_coordinates_lla[0], target_coordinates_lla[1], target_coordinates_lla[2]);

# Given two points (x,y,z) and (x',y',z') in an earth-centered coordinate system,
# the vector from the first to the second is (dx,dy,dz) = (x'-x, y'-y, z'-z).

    x = origin_coordinates_ecef[0];
    y = origin_coordinates_ecef[1];
    z = origin_coordinates_ecef[2];
    dx = target_coordinates_ecef[0] - x;
    dy = target_coordinates_ecef[1] - y;
    dz = target_coordinates_ecef[2] - z;

# # Elevation

# To determine the elevation_angle angle we project the difference vector, dx, into the
# position vector at the observer, which is in the radial direction. From this we
# projection we can determine the angle of the dx from the radial vector, which is
# the elevation_angle.

#   cos(el) = (x dot dx) / (norm(x) * norm(dx))
#   cos(el) = (x dot dx) / sqrt(x dot x) * sqrt(dx dot dx))
#   cos(el) = (x dot dx) / sqrt(x^2 * dx^2)
#   cos(el) = (x*dx + y*dy + z*dz) / sqrt((x**2+y**2+z**2)*(dx**2+dy**2+dz**2))
    cosine_of_elevation_angle = (x * dx + y * dy + z * dz) / math.sqrt((x ** 2 + y ** 2 + z ** 2) * (dx ** 2 + dy ** 2 + dz ** 2));
    elevation_angle = math.acos(cosine_of_elevation_angle);
    
#   the el angle in spherical coordinates is defined from vertical. To convert it to the
#   standard notion of el (in relation to horizon), subtract from 90deg
    elevation_angle = math.pi / 2 - elevation_angle;

# # Azimuth
    
#   For azimuth_angle, we need a level vector (u,v,w) that points due north. One such vector at the location
#   (x,y,z) is (-z*x, -z*y, x^2+y^2). (The inner product of these two vectors is zero,
#   proving it is orthogonal to the radial direction, ie it is level with the horizon.
#   Its projection onto the Equatorial plane is proportional to (-x,-y) which points directly inward,
#   making it the projection of a north-pointing vector.
#   These two calculations confirm that this is indeed the desired vector). Therefore

#   Cos(azimuth) = (-z*x*dx - z*y*dy + (x**2+y**2)*dz) / Sqrt((x**2+y**2)(x**2+y**2+z**2)(dx**2+dy**2+dz**2))
    cosine_of_azimuth_angle = (-z * x * dx - z * y * dy + (x ** 2 + y ** 2) * dz) / math.sqrt((x ** 2 + y ** 2) * (x ** 2 + y ** 2 + z ** 2) * (dx ** 2 + dy ** 2 + dz ** 2));

#   We also need the sine of the azimuth, which is similarly obtained once we know a
#   vector pointing due East (locally). Such a vector is (-y, x, 0), because it is
#   perpendicular to (x,y,z) (the up direction) and the northern direction. Therefore
    
#     Sin(azimuth) = (-y*dx + x*dy) / Sqrt((x**2+y**2)*(dx**2+dy**2+dz**2))
    sine_of_azimuth_angle = (-y * dx + x * dy) / math.sqrt((x ** 2 + y ** 2) * (dx ** 2 + dy ** 2 + dz ** 2));

#   These values enable us to recover the azimuth as the inverse tangent of the cosine and sine.
    azimuth_angle = math.atan2(sine_of_azimuth_angle, cosine_of_azimuth_angle);

    # wrap azimuth angle if its less than zero
    if(azimuth_angle < 0):
        azimuth_angle = azimuth_angle + 2 * math.pi;

    # output angles in degrees
    return [azimuth_angle * 180 / math.pi, elevation_angle * 180 / math.pi]


def CoordinatesToECEF(latitude, longitude, altitude_m):
#         lat,lon,height to xyz vector
#       adapted from javascript from http://www.oc.nps.edu/oc2902w/coord/llhxyz.htm
# 
#      input:
#           latitude      geodetic latitude in deg
#           longitude              longitude in deg
#           altitude_m             altitude in meters
#      output:
#           returns vector x 3 long ECEF in meters

    import math

# WGS84 and Earth Constants
    ellipsoid_semimajor_axis_m = 6378137;
    flattening_factor = 1.0 / 298.257223563;
    ellipsoid_semiminor_axis_m = ellipsoid_semimajor_axis_m * (1.0 - flattening_factor);
    eccentricity_squared = 1 - ellipsoid_semiminor_axis_m * ellipsoid_semiminor_axis_m / (ellipsoid_semimajor_axis_m * ellipsoid_semimajor_axis_m);
    
    cosine_of_latitude = math.cos(latitude * math.pi / 180.0);
    sine_of_latitude = math.sin(latitude * math.pi / 180.0);
    cosine_of_longitude = math.cos(longitude * math.pi / 180.0);
    sine_of_longitude = math.sin(longitude * math.pi / 180.0);

    radii = RadiiOfCurvature(ellipsoid_semimajor_axis_m, ellipsoid_semiminor_axis_m, latitude);
    rn = radii[2];

    x = (rn + altitude_m) * cosine_of_latitude * cosine_of_longitude;
    y = (rn + altitude_m) * cosine_of_latitude * sine_of_longitude;
    z = ((1 - eccentricity_squared) * rn + altitude_m) * sine_of_latitude;

    ECEF = [x, y, z];
    return ECEF;

def RadiiOfCurvature(semimajor_axis, semiminor_axis, latitude):
#        compute the radii at the geodetic latitude (in degrees)
#      
#      input:
#                latitude       geodetic latitude in degrees
#      output:   
#                radii     an array 3 long
#                          r,  rn,  rm

    import math
    
    eccentricity_squared = 1 - ((semimajor_axis * semimajor_axis) / (semiminor_axis * semiminor_axis));

    cosine_of_latitude = math.cos(latitude * math.pi / 180.0);
    sine_of_latitude = math.sin(latitude * math.pi / 180.0);

    dsq = 1.0 - eccentricity_squared * (sine_of_latitude * sine_of_latitude);
    d = math.sqrt(dsq);

    rn = semimajor_axis / d;
    rm = rn * (1.0 - eccentricity_squared) / dsq;

    rho = rn * cosine_of_latitude;
    z = (1.0 - eccentricity_squared) * rn * sine_of_latitude;
    radius_squared = rho * rho + z * z;
    r = math.sqrt(radius_squared);

    radii = [r, rn, rm];

    return radii
