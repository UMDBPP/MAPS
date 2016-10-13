def ViewAngle(origin_lla, target_lla):
#   calculates Az,El to a point given an origin and target
#       from algorithm found at http://gis.stackexchange.com/questions/58923/calculate-view-angle
#
#       input:
#       origin - 3 tuple of (lat [deg], lng [deg], alt[m]) giving location of observer
#       target - 3 tuple of (lat [deg], lng [deg], alt[m]) giving location of target
#
#       output:
#       2 element tuple contining Az,El in deg
#

    import math

    # convert origin coordinates from LLA to ECEF
    origin_ecef = llhxyz(origin_lla[0], origin_lla[1], origin_lla[2]);

    # convert target coordinates from LLA to ECEF
    target_ecef = llhxyz(target_lla[0], target_lla[1], target_lla[2]);

# Given two points (x,y,z) and (x',y',z') in an earth-centered coordinate system,
# the vector from the first to the second is (dx,dy,dz) = (x'-x, y'-y, z'-z).

    x = origin_ecef[0];
    y = origin_ecef[1];
    z = origin_ecef[2];
    dx = target_ecef[0] - x;
    dy = target_ecef[1] - y;
    dz = target_ecef[2] - z;

## Elevation

# To determine the elevation angle we project the difference vector, dx, into the
# position vector at the observer, which is in the radial direction. From this we
# projection we can determine the angle of the dx from the radial vector, which is
# the elevation.

#   cos(el) = (x dot dx) / (norm(x) * norm(dx))
#   cos(el) = (x dot dx) / sqrt(x dot x) * sqrt(dx dot dx))
#   cos(el) = (x dot dx) / sqrt(x^2 * dx^2)
#   cos(el) = (x*dx + y*dy + z*dz) / sqrt((x**2+y**2+z**2)*(dx**2+dy**2+dz**2))
    cosEl = (x*dx + y*dy + z*dz) / math.sqrt((x**2+y**2+z**2)*(dx**2+dy**2+dz**2));
    El =  math.acos(cosEl);
    
#   the el angle in spherical coordinates is defined from vertical. To convert it to the
#   standard notion of el (in relation to horizon), subtract from 90deg
    El = math.pi/2 - El;

## Azimuth
    
#   For Az, we need a level vector (u,v,w) that points due north. One such vector at the location
#   (x,y,z) is (-z*x, -z*y, x^2+y^2). (The inner product of these two vectors is zero,
#   proving it is orthogonal to the radial direction, ie it is level with the horizon.
#   Its projection onto the Equatorial plane is proportional to (-x,-y) which points directly inward,
#   making it the projection of a north-pointing vector.
#   These two calculations confirm that this is indeed the desired vector). Therefore

#   Cos(azimuth) = (-z*x*dx - z*y*dy + (x**2+y**2)*dz) / Sqrt((x**2+y**2)(x**2+y**2+z**2)(dx**2+dy**2+dz**2))
    cosAz = (-z*x*dx - z*y*dy + (x**2+y**2)*dz) / math.sqrt((x**2+y**2)*(x**2+y**2+z**2)*(dx**2+dy**2+dz**2));

#   We also need the sine of the azimuth, which is similarly obtained once we know a
#   vector pointing due East (locally). Such a vector is (-y, x, 0), because it is
#   perpendicular to (x,y,z) (the up direction) and the northern direction. Therefore
    
#     Sin(azimuth) = (-y*dx + x*dy) / Sqrt((x**2+y**2)*(dx**2+dy**2+dz**2))
    sinAz = (-y*dx + x*dy) / math.sqrt((x**2+y**2)*(dx**2+dy**2+dz**2));

#   These values enable us to recover the azimuth as the inverse tangent of the cosine and sine.
    Az = math.atan2(sinAz,cosAz);

    # wrap az angle if its less than zero
    if(Az < 0):
        Az = Az+2*math.pi;

    # convert to deg and output
    return [Az*180/math.pi, El*180/math.pi]


def llhxyz(flat,flon, altm):
#         lat,lon,height to xyz vector
#       adapted from javascript from http://www.oc.nps.edu/oc2902w/coord/llhxyz.htm
# 
#      input:
#           flat      geodetic latitude in deg
#           flon      longitude in deg
#           altkm     altitude in km
#      output:
#           returns vector x 3 long ECEF in km
# 

    import math
    
    altkm = altm * 0.001;

    #initalize constants
    K = LoadK();

    clat = math.cos(K['d2r']*flat);
    slat = math.sin(K['d2r']*flat);
    clon = math.cos(K['d2r']*flon);
    slon = math.sin(K['d2r']*flon);

    rrnrm  = radcur(K, flat);
    rn     = rrnrm[2];

    ecc    = K['EARTH_Ecc'];
    esq    = ecc*ecc;

    x      =  (rn + altkm) * clat * clon;
    y      =  (rn + altkm) * clat * slon;
    z      =  ( (1-esq)*rn + altkm ) * slat;

    xvec = [x, y, z];
    return xvec;

def radcur(K, lat):
#        compute the radii at the geodetic latitude lat (in degrees)
#      
#      input:
#                lat       geodetic latitude in degrees
#      output:   
#                rrnrm     an array 3 long
#                          r,  rn,  rm   in km
# 
#
    import math
    
    a     = K['EARTH_A'];
    b     = K['EARTH_B'];

    asq   = a*a;
    bsq   = b*b;
    eccsq  =  1 - bsq/asq;
    ecc = math.sqrt(eccsq);

    clat  =  math.cos(K['d2r']*lat);
    slat  =  math.sin(K['d2r']*lat);

    dsq   =  1.0 - eccsq * slat * slat;
    d     =  math.sqrt(dsq);

    rn    =  a/d;
    rm    =  rn * (1.0 - eccsq ) / dsq;

    rho   =  rn * clat;
    z     =  (1.0 - eccsq ) * rn * slat;
    rsq   =  rho*rho + z*z;
    r     =  math.sqrt( rsq );

    rrnrm  =  [r, rn, rm];

    return rrnrm

def LoadK():
# load earth model constants
    import math

    K = {};
# WGS84 Earth Constants
    a =  6378.137;
    f =  1.0/298.257223563;
    b =  a * ( 1.0 - f );
    K['wgs84a'] = a;
    K['wgs84f'] = f;
    K['wgs84b'] = b;
    
# Earth constants
    f        =  1-b/a;
    eccsq    =  1 - b*b/(a*a);
    ecc      =  math.sqrt(eccsq);

    K['EARTH_A'] = a;
    K['EARTH_B'] = b;
    K['EARTH_F'] = f;
    K['EARTH_Ecc'] = ecc;
    K['EARTH_Esq'] = eccsq;
    
# other constants
    K['d2r'] =math.pi/180.0;
    
    return K
