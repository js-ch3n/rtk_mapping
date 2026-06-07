/**
 * gnss_conversion_node.cpp
 *
 * Converts raw GNSS/RTK NavSatFix data (lat/lon/alt) into relative
 * odometry starting from (0,0,0) with identity orientation.
 *
 * Pipeline:
 *   sensor_msgs::NavSatFix → gnss_conversion_node → /rtk_odom → FAST_LIO
 *
 * Conversion: WGS84 lat/lon/alt → ECEF → ENU (relative to first fix)
 */

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <string>

using V3D = Eigen::Vector3d;
using M3D = Eigen::Matrix3d;

// ─── WGS84 constants ──────────────────────────────────────────────────
static constexpr double WGS84_A  = 6378137.0;           // semi-major axis (m)
static constexpr double WGS84_F  = 1.0 / 298.257223563; // flattening
static constexpr double WGS84_E2 = WGS84_F * (2.0 - WGS84_F); // eccentricity²

// ─── GNSS status mapping ──────────────────────────────────────────────
// NavSatFix status constants (from sensor_msgs/NavSatStatus)
//  -1 = NO_FIX, 0 = FIX, 1 = SBAS_FIX, 2 = GBAS_FIX
static constexpr int8_t STATUS_NO_FIX = -1;

// ─── State ────────────────────────────────────────────────────────────
static bool   origin_set    = false;
static double origin_lat    = 0.0;   // radians
static double origin_lon    = 0.0;   // radians
static double origin_alt    = 0.0;   // meters
static V3D    origin_ecef   = V3D::Zero();  // origin in ECEF
static V3D    last_enu      = V3D::Zero();  // last ENU position
static double last_yaw      = 0.0;   // computed heading from motion
static bool   heading_valid = false;

// ─── Configuration ────────────────────────────────────────────────────
static std::string input_topic   = "/rtk_fix";
static std::string output_topic  = "/rtk_odom";
static std::string world_frame   = "odom";
static std::string child_frame   = "base_link";
static bool   publish_tf         = true;
static bool   reset_on_jump      = true;
static double jump_threshold     = 50.0;           // meters
static double heading_min_speed  = 0.5;            // m/s — min speed for heading computation
static double fix_status_min     = 0;              // min NavSatFix status (0 = at least FIX)
static double antenna_offset_x   = 0.0;            // GNSS antenna offset from base_link (forward)
static double antenna_offset_y   = 0.0;            // GNSS antenna offset from base_link (left)
static double antenna_offset_z   = 0.0;            // GNSS antenna offset from base_link (up)

// ─── Geodesic helpers ─────────────────────────────────────────────────

/// WGS84 geodetic (lat,lon,alt) → ECEF (Earth-Centered Earth-Fixed)
inline V3D geodetic_to_ecef(double lat, double lon, double alt)
{
    double sin_lat = sin(lat);
    double cos_lat = cos(lat);
    double sin_lon = sin(lon);
    double cos_lon = cos(lon);

    double N = WGS84_A / sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat); // prime vertical radius

    double x = (N + alt) * cos_lat * cos_lon;
    double y = (N + alt) * cos_lat * sin_lon;
    double z = (N * (1.0 - WGS84_E2) + alt) * sin_lat;

    return V3D(x, y, z);
}

/// ECEF delta → ENU (East, North, Up) relative to reference point
inline V3D ecef_to_enu(const V3D &delta_ecef, double ref_lat, double ref_lon)
{
    double sin_lat = sin(ref_lat);
    double cos_lat = cos(ref_lat);
    double sin_lon = sin(ref_lon);
    double cos_lon = cos(ref_lon);

    double e = -sin_lon * delta_ecef.x() + cos_lon * delta_ecef.y();
    double n = -sin_lat * cos_lon * delta_ecef.x()
               - sin_lat * sin_lon * delta_ecef.y()
               + cos_lat * delta_ecef.z();
    double u =  cos_lat * cos_lon * delta_ecef.x()
               + cos_lat * sin_lon * delta_ecef.y()
               + sin_lat * delta_ecef.z();

    return V3D(e, n, u);
}

/// Compute yaw (heading) from position delta in ENU
/// Returns yaw in radians, 0 = East, positive = counter-clockwise (ENU convention)
inline double compute_heading(const V3D &delta_enu)
{
    return atan2(delta_enu.x(), delta_enu.y());  // ENU: East = x, North = y
}

/// Convert ENU yaw to ROS quaternion (rotation about ENU Up axis)
inline Eigen::Quaterniond enu_yaw_to_quat(double yaw)
{
    // ENU yaw: 0 = North, positive CW (ROS convention: yaw about Z=Up)
    // The ENU → ROS frame mapping: ENU East = ROS X, ENU North = ROS Y, ENU Up = ROS Z
    // In ENU, yaw from East: heading = atan2(E, N)
    // ROS yaw: rotation about Z, 0 = forward (X), which maps to ENU East
    // So ROS yaw = atan2(N, E)?? No...
    //
    // Standard: ENU East → ROS X, ENU North → ROS Y, ENU Up → ROS Z
    // Heading from ENU North, clockwise: yaw = atan2(E, N)
    // For ROS quaternion (rotation about Z=Up):
    Eigen::AngleAxisd rot(yaw, Eigen::Vector3d::UnitZ());
    return Eigen::Quaterniond(rot);
}

// ─── Callback ─────────────────────────────────────────────────────────
void fix_callback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    // ── Status check ────────────────────────────────────────────
    if (msg->status.status <= STATUS_NO_FIX)
    {
        ROS_WARN_THROTTLE(5.0, "GNSS: no fix (status=%d), skipping",
                          msg->status.status);
        return;
    }
    if (msg->status.status < fix_status_min)
    {
        ROS_WARN_THROTTLE(5.0, "GNSS: fix status %d < min %d, skipping",
                          msg->status.status, (int)fix_status_min);
        return;
    }

    // NaN / zero check
    if (std::isnan(msg->latitude) || std::isnan(msg->longitude) || std::isnan(msg->altitude))
    {
        ROS_WARN_THROTTLE(5.0, "GNSS: NaN in fix, skipping");
        return;
    }
    if (msg->latitude == 0.0 && msg->longitude == 0.0)
    {
        ROS_WARN_THROTTLE(5.0, "GNSS: (0,0) fix, skipping");
        return;
    }

    double lat = msg->latitude * M_PI / 180.0;
    double lon = msg->longitude * M_PI / 180.0;
    double alt = msg->altitude;

    // ── First fix: set origin ───────────────────────────────────
    if (!origin_set)
    {
        origin_lat  = lat;
        origin_lon  = lon;
        origin_alt  = alt;
        origin_ecef = geodetic_to_ecef(lat, lon, alt);
        last_enu    = V3D::Zero();
        heading_valid = false;
        origin_set  = true;

        ROS_INFO("GNSS Conversion: origin set (%.7f°, %.7f°, %.3fm)",
                 msg->latitude, msg->longitude, alt);
        // Publish identity odometry at origin
    }

    // ── Convert to ENU relative to origin ───────────────────────
    V3D current_ecef = geodetic_to_ecef(lat, lon, alt);
    V3D enu = ecef_to_enu(current_ecef - origin_ecef, origin_lat, origin_lon);

    // ── Jump detection ──────────────────────────────────────────
    if (origin_set)
    {
        double jump_dist = enu.norm();
        if (reset_on_jump && jump_dist > jump_threshold)
        {
            ROS_WARN("GNSS Conversion: position jump (%.1f m), resetting origin",
                     jump_dist);
            origin_lat  = lat;
            origin_lon  = lon;
            origin_alt  = alt;
            origin_ecef = current_ecef;
            last_enu    = V3D::Zero();
            enu         = V3D::Zero();
            heading_valid = false;
        }
    }

    // ── Compute heading from motion ─────────────────────────────
    double yaw = 0.0;
    if (origin_set)
    {
        V3D delta = enu - last_enu;
        double speed = delta.head<2>().norm();  // horizontal speed
        if (speed > heading_min_speed)
        {
            yaw = compute_heading(delta);
            last_yaw = yaw;
            heading_valid = true;
        }
        else if (heading_valid)
        {
            yaw = last_yaw;  // hold last valid heading
        }
    }
    last_enu = enu;

    // ── Apply antenna offset in local frame ─────────────────────
    // Antenna offset is defined in base_link frame:
    //   x=forward, y=left, z=up
    // We need to subtract the offset to get base_link from antenna.
    // If antenna is at (0.3, 0, 1.5) relative to base_link,
    // base_link is at (-0.3, 0, -1.5) relative to antenna.
    // Rotate offset by current heading:
    double cos_yaw = cos(yaw);
    double sin_yaw = sin(yaw);
    double offset_enu_e = cos_yaw * antenna_offset_x - sin_yaw * antenna_offset_y;
    double offset_enu_n = sin_yaw * antenna_offset_x + cos_yaw * antenna_offset_y;
    double offset_enu_u = antenna_offset_z;

    V3D base_enu = enu;
    base_enu.x() -= offset_enu_e;
    base_enu.y() -= offset_enu_n;
    base_enu.z() -= offset_enu_u;

    // ── Orientation: yaw (ENU East-based) → quaternion ─────────
    Eigen::Quaterniond q = enu_yaw_to_quat(yaw);

    // ── Build odometry message ──────────────────────────────────
    nav_msgs::Odometry odom;
    odom.header.stamp    = msg->header.stamp;
    odom.header.frame_id = world_frame;
    odom.child_frame_id  = child_frame;

    odom.pose.pose.position.x    = base_enu.x();
    odom.pose.pose.position.y    = base_enu.y();
    odom.pose.pose.position.z    = base_enu.z();
    odom.pose.pose.orientation.w = q.w();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();

    // Covariance: from NavSatFix position_covariance (ENU diagonal)
    // NavSatFix covariance is in m², type is 3x3 in ENU order [E, N, U]
    double cov_e = msg->position_covariance[0];  // East
    double cov_n = msg->position_covariance[4];  // North
    double cov_u = msg->position_covariance[8];  // Up
    // Set ENU position covariance (orientation stays as large uncertainty)
    for (int i = 0; i < 36; i++) odom.pose.covariance[i] = 0.0;
    odom.pose.covariance[0]  = cov_e;
    odom.pose.covariance[7]  = cov_n;
    odom.pose.covariance[14] = cov_u;
    odom.pose.covariance[21] = 999.0;  // orientation X (unknown)
    odom.pose.covariance[28] = 999.0;  // orientation Y (unknown)
    odom.pose.covariance[35] = 999.0;  // orientation Z (unknown)

    // ── Publish ─────────────────────────────────────────────────
    static ros::Publisher pub =
        ros::NodeHandle().advertise<nav_msgs::Odometry>(output_topic, 100);
    pub.publish(odom);

    // ── Broadcast TF ────────────────────────────────────────────
    if (publish_tf)
    {
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(base_enu.x(), base_enu.y(), base_enu.z()));
        transform.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
        br.sendTransform(tf::StampedTransform(transform, odom.header.stamp,
                                               world_frame, child_frame));
    }
}

// ─── Main ─────────────────────────────────────────────────────────────
int main(int argc, char **argv)
{
    ros::init(argc, argv, "gnss_conversion_node");
    ros::NodeHandle nh("~");

    // ── Parameters ──────────────────────────────────────────────
    nh.param<std::string>("input_rtk_topic",    input_topic,   "/rtk_fix");
    nh.param<std::string>("output_odom_topic",  output_topic,  "/rtk_odom");
    nh.param<std::string>("world_frame_id",     world_frame,   "odom");
    nh.param<std::string>("child_frame_id",     child_frame,   "base_link");
    nh.param<bool>       ("publish_tf",         publish_tf,    true);
    nh.param<bool>       ("reset_on_jump",      reset_on_jump, true);
    nh.param<double>     ("jump_threshold",     jump_threshold, 50.0);
    nh.param<double>     ("heading_min_speed",  heading_min_speed, 0.5);
    nh.param<double>     ("fix_status_min",     fix_status_min, 0.0);
    nh.param<double>     ("antenna_offset_x",   antenna_offset_x, 0.0);
    nh.param<double>     ("antenna_offset_y",   antenna_offset_y, 0.0);
    nh.param<double>     ("antenna_offset_z",   antenna_offset_z, 0.0);

    ROS_INFO("GNSS Conversion Node starting:");
    ROS_INFO("  Input:           %s (sensor_msgs::NavSatFix)", input_topic.c_str());
    ROS_INFO("  Output:          %s (nav_msgs::Odometry)", output_topic.c_str());
    ROS_INFO("  Frame:           %s → %s", world_frame.c_str(), child_frame.c_str());
    ROS_INFO("  Jump threshold:  %.1f m", jump_threshold);
    ROS_INFO("  Heading min spd: %.1f m/s", heading_min_speed);
    ROS_INFO("  Fix status min:  %d", (int)fix_status_min);
    ROS_INFO("  Antenna offset:  [%.3f, %.3f, %.3f] (x,y,z base_link)",
             antenna_offset_x, antenna_offset_y, antenna_offset_z);

    // ── Subscribe ───────────────────────────────────────────────
    ros::NodeHandle nh_public;
    ros::Subscriber sub = nh_public.subscribe<sensor_msgs::NavSatFix>(
        input_topic, 1000, fix_callback);

    ros::spin();
    return 0;
}
