//reference file for gps initialization


sensor_msgs::NavSatFix carGps;
Eigen::Vector3d mapGps;
Eigen::Vector3d mapXYZ;

geometry_msgs::PoseStamped mapState;
geometry_msgs::PoseStamped carState;

const double _earth_a = 6378137.00000;       // [m] WGS84 equator radius
const double _earth_b = 6356752.31414;       // [m] WGS84 epolar radius
const double _earth_e = 8.1819190842622e-2;  //  WGS84 eccentricity
const double _aa = _earth_a * _earth_a;
const double _ee = _earth_e * _earth_e;

////////////////////////////////////////// Helper Functions /////////////////////////////////////////////////

/** Convert GPS to X-Y-Z in utm **/
Eigen::Vector3d gpsToUTM(double lat, double lon, double alt)
{
    lanelet::GPSPoint origin_latlon;
    origin_latlon.lat = lat;
    origin_latlon.lon = lon;
    origin_latlon.ele = alt;
    lanelet::Origin origin{ origin_latlon };
    auto projector = lanelet::projection::UtmProjector(origin, false, false);
    auto origin_xy = projector.forward(origin_latlon);

    Eigen::Vector3d pose;
    pose(0) = origin_xy.x();
    pose(1) = origin_xy.y();
    pose(2) = origin_xy.z();
    return pose;
}

/** Convert GPS to X-Y-Z using spherical model of earth **/
Eigen::Vector3d gpsXYZ(double lat, double lon, double alt)
{
    double r = 6371000 + alt;
    double x = r * cos(lat * 3.14 / 180.0) * cos(lon * 3.14 / 180.0);
    double y = r * cos(lat * 3.14 / 180.0) * sin(lon * 3.14 / 180.0);
    double z = r * sin(lat * 3.14 / 180.0);
    Eigen::Vector3d pose;
    pose(0) = x;
    pose(1) = y;
    pose(2) = z;
    return pose;
}

/** Convert GPS coordinates to X-Y-Z coordinates using WGS-84 **/
Eigen::Vector3d gpsToXYZ(double lat, double lon, double alt)
{
    double a, b, x, y, z, h, l, c, s;
    a = lon;
    b = lat;
    h = alt;
    c = cos(b);
    s = sin(b);
    // WGS84 from eccentricity
    l = _earth_a / sqrt(1.0 - (_ee * s * s));
    x = (l + h) * c * cos(a);
    y = (l + h) * c * sin(a);
    z = (((1.0 - _ee) * l) + h) * s;

    Eigen::Vector3d coordinate;
    coordinate(0) = x;
    coordinate(1) = y;
    coordinate(2) = z;
    // std::cout << "Car Coordinates ";  //<<coordinate.transpose()<<std::endl;
    return coordinate;
}

/** Get Distance between GPS coordinates **/
double getDistance(double lat1, double lon1, double lat2, double lon2)
{
    double earthRadius = 6371000;
    double dLat = (lat2 - lat1) * 3.14 / 180.0;
    double dLon = (lon2 - lon1) * 3.14 / 180.0;

    lat1 = lat1 * 3.14 / 180.0;
    lat2 = lat2 * 3.14 / 180.0;
    lon1 = lon1 * 3.14 / 180.0;

    double a = sin(dLat / 2) * sin(dLat / 2) + sin(dLon / 2) * sin(dLon / 2) * cos(lat1) * cos(lat2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return abs(earthRadius * c);
}


/* Callback for accessing local pose from mavros */
void mavrosPoseHandler(geometry_msgs::PoseStamped msg)
{
    if (mavPoseCount == 0)
    {
        mapState = msg;
        tf::Quaternion qMap(mapState.pose.orientation.x, mapState.pose.orientation.y, mapState.pose.orientation.z, mapState.pose.orientation.w);
        tf::Matrix3x3 m(qMap);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        mapYaw_ = (float)yaw;
    }
    else
    {
        carState = msg;
        // now set the car yaw
        tf::Quaternion qCar(carState.pose.orientation.x, carState.pose.orientation.y, carState.pose.orientation.z, carState.pose.orientation.w);
        tf::Matrix3x3 m(qCar);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        carYaw_ = yaw;
    }
    // std::cout<<mapState <<std::endl;
    mavPoseCount++;
    gotImuPose=true;
}
