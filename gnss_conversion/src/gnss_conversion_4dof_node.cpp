/**
 * gnss_conversion_4dof_node.cpp
 *
 * 4-DOF RTK Odometry Node — dedicated to RTK mapping with external yaw.
 *
 * This node addresses a specific limitation of the standard 3-DOF
 * gnss_conversion_node: when the vehicle is stationary or at low speed,
 * heading-from-motion yields no usable orientation.  For RTK mapping
 * systems (FAST_LIO Mode 2) that expect a full 6-DOF pose, missing yaw
 * causes the map to rotate randomly around the origin.
 *
 * This node solves the problem by ingesting an external yaw signal:
 *
 *   Input 1:  /rtk_fix   (sensor_msgs::NavSatFix)  — RTK lat/lon/alt
 *   Input 2:  /rtk_yaw   (nav_msgs::Odometry)       — vehicle yaw from pose.orientation
 *              Convention: clockwise from East, positive clockwise when
 *              viewed top-down (ENU frame).  See param yaw_from_east_cw.
 *   Output:   /rtk_odom  (nav_msgs::Odometry)       — 4-DOF 零起点里程计
 *
 * Conversion pipeline:
 *   WGS84(lat,lon,alt) → ECEF → ENU (relative to first fix)
 *   Position: (0,0,0) at first fix, subsequent fixes in ENU meters
 *   Orientation: quaternion from external yaw (identity at origin)
 *
 * Use case:
 *   - Dual-antenna RTK providing fixed yaw
 *   - Ground-truth trajectory replay (yaw + position from truth)
 *   - Any scenario where yaw is known but pitch/roll are not
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
static constexpr double WGS84_A  = 6378137.0;
static constexpr double WGS84_F  = 1.0 / 298.257223563;
static constexpr double WGS84_E2 = WGS84_F * (2.0 - WGS84_F);

static constexpr int8_t STATUS_NO_FIX = -1;

// ─── Shared state (protected by ROS spin mutex) ───────────────────────
static bool   origin_set    = false;
static double origin_lat    = 0.0;   // radians
static double origin_lon    = 0.0;
static double origin_alt    = 0.0;
static V3D    origin_ecef   = V3D::Zero();

static double latest_yaw    = 0.0;   // latest received yaw [rad]
static bool   yaw_received  = false;

// ─── Configuration ────────────────────────────────────────────────────
static std::string input_fix_topic  = "/rtk_fix";
static std::string input_yaw_topic  = "/rtk_yaw";
static std::string output_topic     = "/rtk_odom";
static std::string world_frame      = "odom";
static std::string child_frame      = "base_link";
static bool   publish_tf            = true;
static bool   reset_on_jump         = true;
static double jump_threshold        = 50.0;
static double fix_status_min        = 0;
static bool   yaw_from_east_cw      = true;   // false = yaw is standard CCW from East
static double antenna_offset_x      = 0.0;
static double antenna_offset_y      = 0.0;
static double antenna_offset_z      = 0.0;

// ─── Geodesic helpers ─────────────────────────────────────────────────
inline V3D geodetic_to_ecef(double lat, double lon, double alt)
{
    double sin_lat = sin(lat), cos_lat = cos(lat);
    double sin_lon = sin(lon), cos_lon = cos(lon);
    double N = WGS84_A / sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat);
    return V3D((N + alt) * cos_lat * cos_lon,
               (N + alt) * cos_lat * sin_lon,
               (N * (1.0 - WGS84_E2) + alt) * sin_lat);
}

inline V3D ecef_to_enu(const V3D &delta_ecef, double ref_lat, double ref_lon)
{
    double sl = sin(ref_lat), cl = cos(ref_lat);
    double sL = sin(ref_lon), cL = cos(ref_lon);
    double e = -sL * delta_ecef.x() + cL * delta_ecef.y();
    double n = -sl * cL * delta_ecef.x() - sl * sL * delta_ecef.y() + cl * delta_ecef.z();
    double u =  cl * cL * delta_ecef.x() + cl * sL * delta_ecef.y() + sl * delta_ecef.z();
    return V3D(e, n, u);
}

// ─── Yaw callback ─────────────────────────────────────────────────────
void yaw_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    // Extract yaw (rotation about Z, ENU Up) from pose quaternion.
    // ROS standard quaternion encodes CCW from East (right-hand rule with Z up).
    double yaw_enu = tf::getYaw(msg->pose.pose.orientation);

    if (yaw_from_east_cw)
        latest_yaw = -yaw_enu;   // source is CW from East → negate to CCW
    else
        latest_yaw = yaw_enu;
    yaw_received = true;
}

// ─── Fix callback ─────────────────────────────────────────────────────
void fix_callback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    if (msg->status.status <= STATUS_NO_FIX) return;
    if (msg->status.status < fix_status_min) return;
    if (std::isnan(msg->latitude) || std::isnan(msg->longitude) || std::isnan(msg->altitude)) return;
    if (msg->latitude == 0.0 && msg->longitude == 0.0) return;

    double lat = msg->latitude * M_PI / 180.0;
    double lon = msg->longitude * M_PI / 180.0;
    double alt = msg->altitude;

    if (!origin_set)
    {
        origin_lat  = lat;
        origin_lon  = lon;
        origin_alt  = alt;
        origin_ecef = geodetic_to_ecef(lat, lon, alt);
        origin_set  = true;
        ROS_INFO("GNSS 4-DOF: origin set (%.7f°, %.7f°, %.3fm)", msg->latitude, msg->longitude, alt);
    }

    V3D current_ecef = geodetic_to_ecef(lat, lon, alt);
    V3D enu = ecef_to_enu(current_ecef - origin_ecef, origin_lat, origin_lon);

    // Jump detection
    if (reset_on_jump && enu.norm() > jump_threshold)
    {
        ROS_WARN("GNSS 4-DOF: position jump (%.1f m), resetting origin", enu.norm());
        origin_lat  = lat;
        origin_lon  = lon;
        origin_alt  = alt;
        origin_ecef = current_ecef;
        enu = V3D::Zero();
    }

    // Antenna offset compensation using external yaw
    double yaw = latest_yaw;
    double cos_y = cos(yaw), sin_y = sin(yaw);
    double off_e = cos_y * antenna_offset_x - sin_y * antenna_offset_y;
    double off_n = sin_y * antenna_offset_x + cos_y * antenna_offset_y;
    V3D base_enu = enu;
    base_enu.x() -= off_e;
    base_enu.y() -= off_n;
    base_enu.z() -= antenna_offset_z;

    Eigen::Quaterniond q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));

    // Odometry
    nav_msgs::Odometry odom;
    odom.header.stamp    = msg->header.stamp;
    odom.header.frame_id = world_frame;
    odom.child_frame_id  = child_frame;

    odom.pose.pose.position.x = base_enu.x();
    odom.pose.pose.position.y = base_enu.y();
    odom.pose.pose.position.z = base_enu.z();
    odom.pose.pose.orientation.w = q.w();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();

    // Covariance
    for (int i = 0; i < 36; i++) odom.pose.covariance[i] = 0.0;
    odom.pose.covariance[0]  = msg->position_covariance[0];
    odom.pose.covariance[7]  = msg->position_covariance[4];
    odom.pose.covariance[14] = msg->position_covariance[8];
    odom.pose.covariance[35] = 0.01;   // yaw variance: small, since external source

    static ros::Publisher pub =
        ros::NodeHandle().advertise<nav_msgs::Odometry>(output_topic, 100);
    pub.publish(odom);

    if (publish_tf)
    {
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(base_enu.x(), base_enu.y(), base_enu.z()));
        transform.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
        br.sendTransform(tf::StampedTransform(transform, odom.header.stamp, world_frame, child_frame));
    }
}

// ─── Main ─────────────────────────────────────────────────────────────
int main(int argc, char **argv)
{
    ros::init(argc, argv, "gnss_conversion_4dof_node");
    ros::NodeHandle nh("~");

    nh.param<std::string>("input_fix_topic",  input_fix_topic,  "/rtk_fix");
    nh.param<std::string>("input_yaw_topic",  input_yaw_topic,  "/rtk_yaw");
    nh.param<std::string>("output_odom_topic", output_topic,    "/rtk_odom");
    nh.param<std::string>("world_frame_id",    world_frame,     "odom");
    nh.param<std::string>("child_frame_id",    child_frame,     "base_link");
    nh.param<bool>       ("publish_tf",        publish_tf,      true);
    nh.param<bool>       ("reset_on_jump",     reset_on_jump,   true);
    nh.param<double>     ("jump_threshold",    jump_threshold,  50.0);
    nh.param<double>     ("fix_status_min",    fix_status_min,  0.0);
    nh.param<bool>       ("yaw_from_east_cw",  yaw_from_east_cw, true);
    nh.param<double>     ("antenna_offset_x",  antenna_offset_x, 0.0);
    nh.param<double>     ("antenna_offset_y",  antenna_offset_y, 0.0);
    nh.param<double>     ("antenna_offset_z",  antenna_offset_z, 0.0);

    ROS_INFO("GNSS 4-DOF Node starting:");
    ROS_INFO("  Input fix:  %s", input_fix_topic.c_str());
    ROS_INFO("  Input yaw:  %s", input_yaw_topic.c_str());
    ROS_INFO("  Output:     %s", output_topic.c_str());
    ROS_INFO("  Yaw mode:   %s", yaw_from_east_cw ? "clockwise from East" : "standard CCW");
    ROS_INFO("  Frame:      %s → %s", world_frame.c_str(), child_frame.c_str());

    ros::NodeHandle nh_public;
    ros::Subscriber sub_fix = nh_public.subscribe<sensor_msgs::NavSatFix>(input_fix_topic, 1000, fix_callback);
    ros::Subscriber sub_yaw = nh_public.subscribe<nav_msgs::Odometry>(input_yaw_topic, 100, yaw_callback);

    ros::spin();
    return 0;
}
