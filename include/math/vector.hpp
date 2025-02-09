#pragma once

#include "angle.hpp"

/**
 * @brief A vector in 2D space
 */
class Vec {
protected:
  Vec() : x(0), y(0){};

public:
  double x;
  double y;

  Vec(double x, double y) : x(x), y(y){};

  /**
   * @brief Get the magnitude of the vector
   *
   * @return double The magnitude
   */
  double magnitude();
  /**
   * @brief The angle of the vector
   *
   * @return Angle The angle
   */
  Angle angle();

  /**
   * @brief The angle between the point defined by this vector and another one
   *
   * @param vector The other one
   * @return Angle The angle
   */
  Angle angle_to(Vec vector);

  /**
   * @brief The distance between this vector and another one
   *
   * @param vector The other one
   * @return double The distance
   */
  double distance_to(Vec vector);
  /**
   * @brief The distance between this vector and another one, squared to save on
   * processing time
   *
   * @param vector The other one
   * @return double The distance
   */
  double distance_to_squared(Vec vector);

  /**
   * @brief Returns the normalized vector (same angle, magnitude of 1)
   *
   * @return Vec The normalized vecotr
   */
  Vec normalized();

  Vec operator+(const Vec &rhs);
  Vec operator-(const Vec &rhs);

  void operator+=(const Vec &rhs);
  void operator-=(const Vec &rhs);

  Vec operator*(const double &rhs);
  Vec operator/(const double &rhs);

  void operator*=(const double &rhs);
  void operator/=(const double &rhs);

  /**
   * @brief Calculate the dot product with another vector
   *
   * @param rhs The other vector
   * @return double The dot product
   */
  double dot(Vec &rhs);
};
