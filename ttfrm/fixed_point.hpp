// Copyright (c) 2021 Tim Perkins

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// “Software”), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to
// the following conditions:

// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
// BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
// ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
// CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef TTFRM_FIXED_POINT_HPP
#define TTFRM_FIXED_POINT_HPP

// #include <stddef.h>  // For size_t (and not std::size_t)
// #include <stdint.h>  // For uint64_t, etc (and not std::uint64_t, etc)

#include <cstddef>
#include <limits>
#include <type_traits>

namespace ttfrm {

template <typename Integer, std::size_t frac_width>
struct FixedPoint final {
  static_assert(!std::is_const<Integer>::value, "Integer type must not be const");
  static_assert(!std::is_volatile<Integer>::value, "Integer type must not be volatile");
  static_assert(!std::is_reference<Integer>::value, "Integer type must not be a reference");
  static_assert(std::is_integral<Integer>::value, "Integer type must be an integer type");

  FixedPoint() = default;
  FixedPoint(const FixedPoint&) = default;
  FixedPoint(FixedPoint&&) = default;

  static FixedPoint FromRaw(Integer value);
  static FixedPoint FromParts(Integer int_part, Integer frac_part);
  template <typename OtherInteger,
            typename = typename std::enable_if<std::is_integral<OtherInteger>::value>::type>
  static FixedPoint FromInteger(OtherInteger value);
  template <typename Float,
            typename = typename std::enable_if<std::is_floating_point<Float>::value>::type>
  static FixedPoint FromFloatingPoint(Float value);

  FixedPoint& operator=(const FixedPoint&) = default;
  FixedPoint& operator=(FixedPoint&&) = default;

  bool operator==(const FixedPoint& other_fxpt) const;
  bool operator!=(const FixedPoint& other_fxpt) const;

  bool operator<(const FixedPoint& other_fxpt) const;
  bool operator>(const FixedPoint& other_fxpt) const;
  bool operator<=(const FixedPoint& other_fxpt) const;
  bool operator>=(const FixedPoint& other_fxpt) const;

  FixedPoint& operator+=(const FixedPoint& other_fxpt);
  FixedPoint& operator-=(const FixedPoint& other_fxpt);
  FixedPoint& operator*=(const FixedPoint& other_fxpt);
  FixedPoint& operator/=(const FixedPoint& other_fxpt);

  FixedPoint operator+(const FixedPoint& other_fxpt) const;
  FixedPoint operator-(const FixedPoint& other_fxpt) const;
  FixedPoint operator*(const FixedPoint& other_fxpt) const;
  FixedPoint operator/(const FixedPoint& other_fxpt) const;

  Integer ExtractIntegerPart() const;
  Integer ExtractFractionalPart() const;

  template <typename Float = double,
            typename = typename std::enable_if<std::is_floating_point<Float>::value>::type>
  Float AsFloatingPoint() const;

  Integer raw_value = 0;

 private:
  explicit FixedPoint(Integer raw_value) : raw_value(raw_value) {}
};

template <typename Integer, std::size_t frac_width>
FixedPoint<Integer, frac_width> FixedPoint<Integer, frac_width>::FromRaw(Integer raw_value)
{
  return FixedPoint(raw_value);
}

template <typename Integer, std::size_t frac_width>
FixedPoint<Integer, frac_width> FixedPoint<Integer, frac_width>::FromParts(Integer int_part,
                                                                           Integer frac_part)
{
  return FixedPoint((int_part << frac_width) + frac_part);
}

template <typename Integer, std::size_t frac_width>
template <typename OtherInteger, typename>
FixedPoint<Integer, frac_width> FixedPoint<Integer, frac_width>::FromInteger(OtherInteger value)
{
  return FixedPoint(value << frac_width);
}

template <typename Integer, std::size_t frac_width>
template <typename Float, typename>
FixedPoint<Integer, frac_width> FixedPoint<Integer, frac_width>::FromFloatingPoint(Float value)
{
  // Use truncation to add the integer and fractional parts
  const Integer int_part = value;
  const Integer frac_part = (Integer(1) << frac_width) * Float(value - int_part);
  return FixedPoint((int_part << frac_width) + frac_part);
}

template <typename Integer, std::size_t frac_width>
bool FixedPoint<Integer, frac_width>::operator==(const FixedPoint& other_fxpt) const
{
  return raw_value == other_fxpt.raw_value;
}

template <typename Integer, std::size_t frac_width>
bool FixedPoint<Integer, frac_width>::operator!=(const FixedPoint& other_fxpt) const
{
  return !(*this == other_fxpt);
}

template <typename Integer, std::size_t frac_width>
bool FixedPoint<Integer, frac_width>::operator<(const FixedPoint& other_fxpt) const
{
  return raw_value < other_fxpt.raw_value;
}

template <typename Integer, std::size_t frac_width>
bool FixedPoint<Integer, frac_width>::operator>(const FixedPoint& other_fxpt) const
{
  return other_fxpt < *this;
}

template <typename Integer, std::size_t frac_width>
bool FixedPoint<Integer, frac_width>::operator<=(const FixedPoint& other_fxpt) const
{
  return !(other_fxpt < *this);
}

template <typename Integer, std::size_t frac_width>
bool FixedPoint<Integer, frac_width>::operator>=(const FixedPoint& other_fxpt) const
{
  return !(*this < other_fxpt);
}

template <typename Integer, std::size_t frac_width>
FixedPoint<Integer, frac_width>& FixedPoint<Integer, frac_width>::operator+=(
    const FixedPoint& other_fxpt)
{
  raw_value += other_fxpt.raw_value;
  return *this;
}

template <typename Integer, std::size_t frac_width>
FixedPoint<Integer, frac_width>& FixedPoint<Integer, frac_width>::operator-=(
    const FixedPoint& other_fxpt)
{
  raw_value -= other_fxpt.raw_value;
  return *this;
}

template <typename Integer, std::size_t frac_width>
FixedPoint<Integer, frac_width>& FixedPoint<Integer, frac_width>::operator*=(
    const FixedPoint& other_fxpt)
{
  constexpr Integer ROUNDING_MASK = (1 << frac_width) - 1;
  constexpr Integer ROUNDING_THRES = (1 << (frac_width - 1));
  raw_value *= other_fxpt.raw_value;
  raw_value += (raw_value & ROUNDING_MASK) < ROUNDING_THRES ? 0 : 1 << frac_width;
  raw_value >>= frac_width;
  return *this;
}

template <typename Integer, std::size_t frac_width>
FixedPoint<Integer, frac_width>& FixedPoint<Integer, frac_width>::operator/=(
    const FixedPoint& other_fxpt)
{
  // Do some additional rounding, this should be configurable. Or consider shifting both sides and
  // using the full frac_width bits to do the rounding, which is probably a better idea.
  constexpr size_t ROUNDING_BITS = 2;
  constexpr Integer ROUNDING_MASK = (1 << ROUNDING_BITS) - 1;
  constexpr Integer ROUNDING_THRES = (1 << (ROUNDING_BITS - 1));
  raw_value <<= frac_width + ROUNDING_BITS;
  raw_value /= other_fxpt.raw_value;
  raw_value += (raw_value & ROUNDING_MASK) < ROUNDING_THRES ? 0 : 1 << ROUNDING_BITS;
  raw_value >>= ROUNDING_BITS;
  return *this;
}

template <typename Integer, std::size_t frac_width>
FixedPoint<Integer, frac_width> FixedPoint<Integer, frac_width>::operator+(
    const FixedPoint& other_fxpt) const
{
  FixedPoint fxpt = *this;
  return fxpt += other_fxpt;
}

template <typename Integer, std::size_t frac_width>
FixedPoint<Integer, frac_width> FixedPoint<Integer, frac_width>::operator-(
    const FixedPoint& other_fxpt) const
{
  FixedPoint fxpt = *this;
  return fxpt -= other_fxpt;
}

template <typename Integer, std::size_t frac_width>
FixedPoint<Integer, frac_width> FixedPoint<Integer, frac_width>::operator*(
    const FixedPoint& other_fxpt) const
{
  FixedPoint fxpt = *this;
  return fxpt *= other_fxpt;
}

template <typename Integer, std::size_t frac_width>
FixedPoint<Integer, frac_width> FixedPoint<Integer, frac_width>::operator/(
    const FixedPoint& other_fxpt) const
{
  FixedPoint fxpt = *this;
  return fxpt /= other_fxpt;
}

template <typename Integer, std::size_t frac_width>
Integer FixedPoint<Integer, frac_width>::ExtractIntegerPart() const
{
  return raw_value >> frac_width;
}

template <typename Integer, std::size_t frac_width>
Integer FixedPoint<Integer, frac_width>::ExtractFractionalPart() const
{
  // Divide this by (2^frac_width) to get the decimal value
  return (~(~Integer(0) << frac_width) & raw_value);
}

template <typename Integer, std::size_t frac_width>
template <typename Float, typename>
Float FixedPoint<Integer, frac_width>::AsFloatingPoint() const
{
  return ExtractIntegerPart() + Float(ExtractFractionalPart()) / (1 << frac_width);
}

}  // namespace ttfrm

#endif  // TTFRM_FIXED_POINT_HPP
