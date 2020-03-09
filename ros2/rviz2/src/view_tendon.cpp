#include <collision/CapsuleSequence.h>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <limits>

#include <cmath>

namespace E = Eigen;
using Vec = E::Vector3d;
using Mat = E::Matrix3d;
using Quaternion = E::Quaterniond;

inline Quaternion find_quat_from_zaxis(const Vec &A) {
  return Quaternion::FromTwoVectors(Vec{0, 0, 1}, A);
}

int main(int argCount, char* argList[]) {
  bool rotate = true;
  collision::CapsuleSequence sequence {{
    {0,                      0,                      0                    },
    {0.00013282379244664586, 8.8542686704915893e-05, 0.0048183362822623761},
    {0.00052783960456719817, 0.00017734411430676313, 0.0096222944924363833},
    {0.0011807888469575864,  0.00023774422435236941, 0.014398475543442689 },
    {0.0020848691901629154,  0.00024181221791107988, 0.019134001636465916 },
    {0.0032308218771827562,  0.00016204630127320717, 0.023816186856668101 },
    {0.0046069523943798856, -2.7725061486809984e-05, 0.028432680572448337 },
    {0.0061991284067021016, -0.00035193988107289743, 0.032971585424409165 },
    {0.0079909432484972492, -0.00083330351577287284, 0.037421377328421053 },
    {0.0099636590414980825, -0.0014929670596176901,  0.041770601259497581 },
    {0.012096483349451518,  -0.0023495436147191605,  0.046008424857334432 },
    {0.014366757923367069,  -0.0034187943683490949,  0.050124817023101453 },
    {0.016750135665100578,  -0.0047137365964838585,  0.054110548703798278 },
    {0.019220482205211914,  -0.0062448052947935203,  0.057957099895916533 },
    {0.021750667340813275,  -0.008019190198795885,   0.061657335489431711 },
    {0.024312913976440365,  -0.010040679141174146,   0.065205714795564404 },
    {0.026878975393225701,  -0.012309912655683711,   0.068598291197242908 },
    {0.029420097745194464,  -0.014824600981260724,   0.071832780109992153 },
    {0.031908079887735023,  -0.017579403035378263,   0.074908999842337148 },
    {0.034315681524303088,  -0.020565993965422275,   0.077828976374351017 },
    {0.036616697558473403,  -0.023773401539954657,   0.080596867379787301 },
    {0.038785880570702397,  -0.027188300501592648,   0.083219048076941721 },
    {0.040799881477261858,  -0.030795295294191095,   0.085704083226517097 },
    {0.042637572355448683,  -0.03457712912139755,    0.088062652661978325 },
    {0.044279952290016557,  -0.038514956165630457,   0.090307409998782284 },
    {0.045709898811377929,  -0.042588556794032881,   0.092453001552805364 },
    {0.04691283674803131,   -0.046776697971979746,   0.094515610880525625 },
    {0.047876926155331852,  -0.051057344086618592,   0.096512743273968762 },
    {0.048592856307176031,  -0.055407764210548729,   0.098463071437137284 },
    {0.049053496497500931,  -0.059804466118157332,   0.10038640775831636  },
    {0.04925440285973795,   -0.064223533491367668,   0.1023029919714103   },
    {0.049193918878868759,  -0.068640799325951934,   0.10423320813694903  },
    {0.048872975101987508,  -0.073031822319777631,   0.10619744256943542  },
    {0.048294857894473034,  -0.077371535870307609,   0.10821604897814674  },
    {0.047465604350272333,  -0.081634790293452553,   0.11030850839790331  },
    {0.046394046775197574,  -0.085796573499466469,   0.11249312009473898  },
    {0.045091655369252255,  -0.089831984936857096,   0.11478689615013155  },
    {0.043572546963745788,  -0.093715821745073757,   0.11720552268591571  },
    {0.041853549366646772,  -0.097423574286689893,   0.11976254988524013  },
    {0.039954123043569545,  -0.10093178396060946,    0.12246911529529105  },
    {0.037896267549349043,  -0.10421790770166464,    0.12533401771210487  },
    }, 0.01};

  ros::init(argCount, argList, "view_tendon");
  ros::NodeHandle n;
  //ros::Publisher marker_pub =
  //  n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  ros::Publisher markerarray_pub =
    n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);

  ros::Rate r(30);

  Quaternion frame_rotation(E::AngleAxisd(0.02*M_PI, Vec::UnitZ()));
  Quaternion current_rotation;
  current_rotation.setIdentity();

  while (ros::ok()) {
    //visualization_msgs::Marker line_strip;

    //line_strip.header.frame_id = "/my_frame";
    //line_strip.header.stamp = ros::Time::now();
    //line_strip.ns = "view_tendon";
    //line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    //line_strip.action = visualization_msgs::Marker::ADD;
    //line_strip.pose.orientation.w = 1.0;  // nonzero part of quat orientation
    //line_strip.id = 0;                    // unique id in namespace
    //line_strip.scale.x = sequence.r;      // line width
    //line_strip.color.b = 1.0;             // blue
    //line_strip.color.a = 1.0;             // opaque

    //for (auto &p : sequence.points) {
    //  geometry_msgs::Point new_point;
    //  new_point.x = p[0];
    //  new_point.y = p[1];
    //  new_point.z = p[2];
    //  line_strip.points.push_back(new_point);
    //}

    //marker_pub.publish(line_strip);

    visualization_msgs::MarkerArray robot;
    for (size_t i = 0; i < sequence.size(); i++) {
      auto capsule = sequence[i];
      visualization_msgs::Marker robot_segment;
      robot_segment.header.frame_id = "/my_frame";
      robot_segment.header.stamp = ros::Time::now();
      robot_segment.ns = "view_tendon";
      robot_segment.type = visualization_msgs::Marker::CYLINDER;
      robot_segment.action = visualization_msgs::Marker::ADD;
      robot_segment.pose.orientation.w = 1.0;
      robot_segment.id = i + 1;
      robot_segment.scale.x = capsule.r;
      robot_segment.scale.y = capsule.r;
      robot_segment.scale.z = (capsule.b - capsule.a).norm();
      robot_segment.color.r = 0.0;
      robot_segment.color.g = 1.0; // green color
      robot_segment.color.b = 0.0;
      robot_segment.color.a = 0.5; // semi-transparent

      auto quat = find_quat_from_zaxis(capsule.b - capsule.a);
      if (rotate) {
        quat = current_rotation * quat;
        capsule.a = current_rotation * capsule.a;
      }
      robot_segment.pose.position.x = capsule.a[0];
      robot_segment.pose.position.y = capsule.a[1];
      robot_segment.pose.position.z = capsule.a[2];
      robot_segment.pose.orientation.x = quat.x();
      robot_segment.pose.orientation.y = quat.y();
      robot_segment.pose.orientation.z = quat.z();
      robot_segment.pose.orientation.w = quat.w();
      robot_segment.lifetime = ros::Duration();

      robot.markers.push_back(robot_segment);
    }
    markerarray_pub.publish(robot);

    r.sleep();
    current_rotation *= frame_rotation;
  }

  return 0;
}

