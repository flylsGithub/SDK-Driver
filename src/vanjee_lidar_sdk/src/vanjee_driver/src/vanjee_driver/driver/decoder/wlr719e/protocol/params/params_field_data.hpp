/*********************************************************************************************************************
Copyright (c) 2023 Vanjee
All rights reserved

By downloading, copying, installing or using the software you agree to this
license. If you do not agree to this license, do not download, install, copy or
use the software.

License Agreement
For Vanjee LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

3. Neither the names of the Vanjee, nor Wanji Technology, nor the
names of other contributors may be used to endorse or promote products derived
from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

#include "vanjee_driver/common/super_header.hpp"
#include "vanjee_driver/common/wj_log.hpp"
#include "vanjee_driver/driver/difop/params_abstract.hpp"

namespace vanjee {
namespace lidar {

struct VanjeePoint2D {
  double x, y;
  VanjeePoint2D(double x = 0, double y = 0) : x(x), y(y) {
  }

  VanjeePoint2D operator+(const VanjeePoint2D& p) const {
    return VanjeePoint2D(x + p.x, y + p.y);
  }
  VanjeePoint2D operator-(const VanjeePoint2D& p) const {
    return VanjeePoint2D(x - p.x, y - p.y);
  }
  VanjeePoint2D operator*(double t) const {
    return VanjeePoint2D(x * t, y * t);
  }
  double dot(const VanjeePoint2D& p) const {
    return x * p.x + y * p.y;
  }
  double cross(const VanjeePoint2D& p) const {
    return x * p.y - y * p.x;
  }
};

namespace field_geom {
constexpr double EPS = 1e-9;
constexpr int RAY_COUNT = 1800;
constexpr int RANGE_BIN_COUNT = 1381;
constexpr int RANGE_OFFSET = 1560;  // map FOV bins onto full 360° ray grid
constexpr double ANGLE_RES = 0.2 * M_PI / 180.0;
constexpr float INVALID_RANGE = 500.0f;

inline double clampNonNeg(double t) {
  return t < 0.0 ? 0.0 : t;
}

inline double normalizeAngle(double theta) {
  theta = fmod(theta, 2 * M_PI);
  if (theta < 0)
    theta += 2 * M_PI;
  return theta;
}

inline void uniqueSortDistances(std::vector<double>& distances) {
  std::sort(distances.begin(), distances.end());
  distances.erase(std::unique(distances.begin(), distances.end(), [](double a, double b) { return fabs(a - b) < EPS; }), distances.end());
}

inline void prependZeroIfNeeded(std::vector<double>& distances, bool origin_inside) {
  if (origin_inside && !distances.empty() && distances.front() > EPS) {
    distances.insert(distances.begin(), 0.0);
  }
}

// Ray from origin along unit dir intersects segment pt_a->pt_b. Returns t, or -1 if none.
// skip_origin: ignore near-zero hits (e.g. sector apex at lidar origin).
inline double intersectRaySegment(const VanjeePoint2D& dir, const VanjeePoint2D& pt_a, const VanjeePoint2D& pt_b, bool skip_origin) {
  VanjeePoint2D ab = pt_b - pt_a;
  double cross_dir_ab = dir.cross(ab);
  if (fabs(cross_dir_ab) < EPS)
    return -1.0;
  double t = pt_a.cross(ab) / cross_dir_ab;
  double u = pt_a.cross(dir) / cross_dir_ab;
  const bool t_ok = skip_origin ? (t > EPS) : (t >= -EPS);
  if (t_ok && u >= -EPS && u <= 1.0 + EPS)
    return clampNonNeg(t);
  return -1.0;
}

// Circle-ray intersections along unit dir. Appends valid t (>= 0) into distances.
inline void intersectRayCircle(const VanjeePoint2D& dir, const VanjeePoint2D& center, double radius, std::vector<double>& distances) {
  double b = -2.0 * center.dot(dir);
  double c = center.dot(center) - radius * radius;
  double disc = b * b - 4.0 * c;
  if (disc < -EPS)
    return;
  if (disc < 0)
    disc = 0;
  double sqrt_disc = sqrt(disc);
  double t1 = (-b - sqrt_disc) * 0.5;
  double t2 = (-b + sqrt_disc) * 0.5;
  if (t1 >= -EPS)
    distances.push_back(clampNonNeg(t1));
  if (t2 >= -EPS && fabs(t2 - t1) > EPS)
    distances.push_back(clampNonNeg(t2));
}

inline int16_t readI16(const uint8_t* buf, uint32_t offset) {
  return static_cast<int16_t>(buf[offset] | (buf[offset + 1] << 8));
}

inline uint16_t readU16(const uint8_t* buf, uint32_t offset) {
  return static_cast<uint16_t>(buf[offset] | (buf[offset + 1] << 8));
}
}  // namespace field_geom

using field_geom::ANGLE_RES;
using field_geom::EPS;
using field_geom::INVALID_RANGE;
using field_geom::RANGE_BIN_COUNT;
using field_geom::RANGE_OFFSET;
using field_geom::RAY_COUNT;

class Shape {
 public:
  virtual ~Shape() {
  }
  virtual std::vector<double> intersectRay(const VanjeePoint2D& dir) const = 0;
};

class Polygon : public Shape {
  std::vector<VanjeePoint2D> vertices_;

  bool containsOrigin() const {
    bool inside = false;
    const int n = static_cast<int>(vertices_.size());
    for (int i = 0, j = n - 1; i < n; j = i++) {
      const VanjeePoint2D& pt_i = vertices_[i];
      const VanjeePoint2D& pt_j = vertices_[j];
      if ((pt_i.y > EPS) != (pt_j.y > EPS)) {
        const double x_int = pt_i.x + (0.0 - pt_i.y) * (pt_j.x - pt_i.x) / (pt_j.y - pt_i.y);
        if (x_int > EPS)
          inside = !inside;
      }
    }
    return inside;
  }

 public:
  explicit Polygon(const std::vector<VanjeePoint2D>& pts) : vertices_(pts) {
    if (vertices_.size() < 4)
      throw std::invalid_argument("多边形至少需要4个顶点");
  }

  std::vector<double> intersectRay(const VanjeePoint2D& dir) const override {
    std::vector<double> distances;
    const int n = static_cast<int>(vertices_.size());
    for (int i = 0; i < n; ++i) {
      double t = field_geom::intersectRaySegment(dir, vertices_[i], vertices_[(i + 1) % n], false);
      if (t >= 0)
        distances.push_back(t);
    }
    field_geom::uniqueSortDistances(distances);
    field_geom::prependZeroIfNeeded(distances, containsOrigin());
    return distances;
  }
};

class Circle : public Shape {
  VanjeePoint2D center_;
  double radius_;

 public:
  Circle(const VanjeePoint2D& center, double radius) : center_(center), radius_(radius) {
    if (radius_ < EPS)
      throw std::invalid_argument("圆半径太小");
  }

  std::vector<double> intersectRay(const VanjeePoint2D& dir) const override {
    std::vector<double> distances;
    field_geom::intersectRayCircle(dir, center_, radius_, distances);
    field_geom::uniqueSortDistances(distances);
    const bool origin_inside = center_.dot(center_) < radius_ * radius_ - EPS;
    field_geom::prependZeroIfNeeded(distances, origin_inside);
    return distances;
  }
};

class Sector : public Shape {
  VanjeePoint2D center_;
  double radius_;
  double start_angle_;  // [0, 2π)
  double end_angle_;    // may be > 2π when wrapping

 public:
  Sector(const VanjeePoint2D& center, double start_angle, double end_angle, double radius)
      : center_(center), radius_(radius), start_angle_(start_angle), end_angle_(end_angle) {
    if (radius_ < EPS)
      throw std::invalid_argument("扇形半径太小");

    start_angle_ = field_geom::normalizeAngle(start_angle_);
    end_angle_ = field_geom::normalizeAngle(end_angle_);
    if (end_angle_ <= start_angle_ + EPS)
      end_angle_ += 2 * M_PI;
  }

  bool angleInSector(double theta) const {
    theta = field_geom::normalizeAngle(theta);
    if (end_angle_ > 2 * M_PI + EPS)
      return (theta >= start_angle_ - EPS) || (theta <= (end_angle_ - 2 * M_PI) + EPS);
    return (theta >= start_angle_ - EPS && theta <= end_angle_ + EPS);
  }

  bool containsOrigin() const {
    const double dist2 = center_.dot(center_);
    if (dist2 > radius_ * radius_ + EPS)
      return false;
    if (dist2 < EPS)
      return true;
    return angleInSector(atan2(-center_.y, -center_.x));
  }

  std::vector<double> intersectRay(const VanjeePoint2D& dir) const override {
    std::vector<double> distances;

    std::vector<double> arc_hits;
    field_geom::intersectRayCircle(dir, center_, radius_, arc_hits);
    for (double t : arc_hits) {
      VanjeePoint2D pt = dir * t;
      if (angleInSector(atan2(pt.y - center_.y, pt.x - center_.x)))
        distances.push_back(t);
    }

    VanjeePoint2D start_point = center_ + VanjeePoint2D(radius_ * cos(start_angle_), radius_ * sin(start_angle_));
    VanjeePoint2D end_point = center_ + VanjeePoint2D(radius_ * cos(end_angle_), radius_ * sin(end_angle_));
    double t_rad1 = field_geom::intersectRaySegment(dir, center_, start_point, true);
    double t_rad2 = field_geom::intersectRaySegment(dir, center_, end_point, true);
    if (t_rad1 >= 0)
      distances.push_back(t_rad1);
    if (t_rad2 >= 0 && fabs(t_rad2 - t_rad1) > EPS)
      distances.push_back(t_rad2);

    field_geom::uniqueSortDistances(distances);
    field_geom::prependZeroIfNeeded(distances, containsOrigin());
    return distances;
  }
};

class VanjeeLidar {
  std::vector<std::unique_ptr<Shape>> shapes_;

 public:
  void addShape(std::unique_ptr<Shape> shape) {
    shapes_.push_back(std::move(shape));
  }

  uint32_t getShapeSize() const {
    return static_cast<uint32_t>(shapes_.size());
  }

  void scan(double dis_min[RAY_COUNT], double dis_max[RAY_COUNT]) const {
    for (int i = 0; i < RAY_COUNT; ++i) {
      dis_min[i] = std::numeric_limits<double>::max();
      dis_max[i] = -1.0;
    }

    for (int i = 0; i < RAY_COUNT; ++i) {
      const double angle = i * ANGLE_RES;
      const VanjeePoint2D dir(cos(angle), sin(angle));

      double ray_min = std::numeric_limits<double>::max();
      double ray_max = -1.0;
      for (const auto& shape : shapes_) {
        for (double d : shape->intersectRay(dir)) {
          ray_min = std::min(ray_min, d);
          ray_max = std::max(ray_max, d);
        }
      }
      if (ray_max >= 0.0) {
        dis_min[i] = ray_min;
        dis_max[i] = ray_max;
      }
    }
  }
};

class Params_FieldData719E : public ParamsAbstract {
 public:
  std::vector<double> lidar_ver_angle_ = std::vector<double>{355.500, 0.0, 4.500, 9.000};
  bool valid_data_flag_;

  bool last_pkt_flag_;
  uint32_t send_total_pkt_num_;

  uint32_t line_id_;
  uint32_t field_id_;
  uint8_t sampling_interval_;
  uint8_t resolution_;
  uint8_t total_pkt_num_;
  uint8_t pkt_id_;
  uint8_t field_definition_;
  float range_min_[RANGE_BIN_COUNT] = {INVALID_RANGE};
  float range_max_[RANGE_BIN_COUNT] = {INVALID_RANGE};

 public:
  virtual std::shared_ptr<std::vector<uint8_t>> GetBytes() {
    return nullptr;
  }

  virtual void Load(ProtocolBase& protocol) {
    std::fill_n(range_min_, RANGE_BIN_COUNT, INVALID_RANGE);
    std::fill_n(range_max_, RANGE_BIN_COUNT, INVALID_RANGE);
    VanjeeLidar lidar;
    valid_data_flag_ = true;

    last_pkt_flag_ = protocol.CmdParams.data()[1] != 0;
    auto buf = protocol.Content.data();
    const uint32_t content_size = static_cast<uint32_t>(protocol.Content.size());
    if (content_size < 7) {
      valid_data_flag_ = false;
      return;
    }

    // last packet: send_total_pkt_num_ = buf[0] | (buf[1] << 8)
    // data packet: field_id_ shares buf[1] with high byte of send_total_pkt_num_
    send_total_pkt_num_ = field_geom::readU16(buf, 0);
    field_id_ = buf[1];
    sampling_interval_ = buf[2];
    resolution_ = buf[3];
    total_pkt_num_ = buf[4];
    pkt_id_ = buf[5];
    field_definition_ = buf[6];
    uint32_t index = 7;

    while (index < content_size) {
      const uint8_t data_info = buf[index++];
      line_id_ = (data_info & 0x03);
      line_id_ = (line_id_ == 0) ? 4 : line_id_;
      const uint8_t graph_type = (data_info >> 4) & 0x03;
      const uint8_t data_type = (data_info >> 6) & 0x03;

      if (graph_type >= 3)
        break;
      if (index >= content_size)
        break;

      const uint8_t endpoints_num = buf[index];
      const uint32_t payload_bytes = static_cast<uint32_t>(endpoints_num) * 4;
      if (index + 1 + payload_bytes > content_size) {
        WJ_WARNING << "field data truncated, stop parse" << WJ_REND;
        break;
      }

      if (data_type == 0) {
        bool parsed = false;
        if (graph_type == 0 && endpoints_num >= 4) {
          std::vector<VanjeePoint2D> poly_pts;
          poly_pts.reserve(endpoints_num);
          for (uint32_t i = 0; i < endpoints_num; i++) {
            const int16_t x = field_geom::readI16(buf, index + 1);
            const int16_t y = field_geom::readI16(buf, index + 3);
            index += 4;
            poly_pts.emplace_back(x / 1000.0, y / 1000.0);
          }
          lidar.addShape(std::make_unique<Polygon>(poly_pts));
          parsed = true;
        } else if (graph_type == 1 && endpoints_num == 2) {
          const int16_t x0 = field_geom::readI16(buf, index + 1);
          const int16_t y0 = field_geom::readI16(buf, index + 3);
          const uint16_t r = field_geom::readU16(buf, index + 5);
          index += 8;
          lidar.addShape(std::make_unique<Circle>(VanjeePoint2D(x0 / 1000.0, y0 / 1000.0), r / 1000.0));
          parsed = true;
        } else if (graph_type == 2 && endpoints_num == 3) {
          const int16_t x0 = field_geom::readI16(buf, index + 1);
          const int16_t y0 = field_geom::readI16(buf, index + 3);
          const uint16_t start_angle = field_geom::readU16(buf, index + 5);
          const uint16_t end_angle = field_geom::readU16(buf, index + 7);
          const uint16_t r = field_geom::readU16(buf, index + 9);
          index += 12;
          lidar.addShape(std::make_unique<Sector>(VanjeePoint2D(x0 / 1000.0, y0 / 1000.0), start_angle / 10.0 / 180.0 * M_PI,
                                                  end_angle / 10.0 / 180.0 * M_PI, r / 1000.0));
          parsed = true;
        }

        if (!parsed) {
          // unmatched graph/endpoints: skip payload like data_type != 0
          index += payload_bytes;
        } else {
          WJ_DEBUG << "field_id: " << field_id_ << ", line id: " << line_id_ << WJ_REND;
        }
      } else {
        index += payload_bytes;
      }
      index += 1;  // pass endpoints_num byte
    }

    if (lidar.getShapeSize() == 0 && !last_pkt_flag_) {
      valid_data_flag_ = false;
      return;
    }

    double dis_min_temp[RAY_COUNT];
    double dis_max_temp[RAY_COUNT];
    lidar.scan(dis_min_temp, dis_max_temp);

    const bool apply_ver_scale = (line_id_ == 1 || line_id_ == 3 || line_id_ == 4);
    const double ver_scale = apply_ver_scale ? (1.0 / cos(lidar_ver_angle_[line_id_ - 1] / 180.0 * M_PI)) : 1.0;

    for (int i = 0; i < RANGE_BIN_COUNT; ++i) {
      const int src_idx = (i + RANGE_OFFSET) % RAY_COUNT;
      const double d_min = dis_min_temp[src_idx];
      const double d_max = dis_max_temp[src_idx];

      if (d_min <= INVALID_RANGE) {
        range_min_[i] = static_cast<float>(d_min * ver_scale);
      }
      if (d_max >= 0.0 && d_max <= INVALID_RANGE && range_max_[i] > d_max) {
        range_max_[i] = static_cast<float>(d_max * ver_scale);
      }
    }
  }
};
}  // namespace lidar
}  // namespace vanjee
