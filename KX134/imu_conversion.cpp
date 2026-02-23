
#include "imu_conversion.hpp"


IMUConversion::IMUConversion(KX134_Range range)
    : _range(range),
      _scale(0.0f),
      _x_offset(0),
      _y_offset(0),
      _z_offset(0)
{
    updateScale();
}



void IMUConversion::setRange(KX134_Range range)
{
    _range = range;
    updateScale();
}

KX134_Range IMUConversion::getRange() const
{
    return _range;
}

void IMUConversion::setOffsets(int16_t x_off, int16_t y_off, int16_t z_off)
{
    _x_offset = x_off;
    _y_offset = y_off;
    _z_offset = z_off;
}

void IMUConversion::clearOffsets()
{
    _x_offset = 0;
    _y_offset = 0;
    _z_offset = 0;
}

IMU_Data IMUConversion::convert(const KX134_Raw& raw) const
{
    IMU_Data data;

    // Promote to 32-bit before subtracting to avoid overflow
    int32_t x_corr = static_cast<int32_t>(raw.x) - _x_offset;
    int32_t y_corr = static_cast<int32_t>(raw.y) - _y_offset;
    int32_t z_corr = static_cast<int32_t>(raw.z) - _z_offset;

    data.x_g = static_cast<float>(x_corr) * _scale;
    data.y_g = static_cast<float>(y_corr) * _scale;
    data.z_g = static_cast<float>(z_corr) * _scale;

    return data;
}

float IMUConversion::getScale() const
{
    return _scale;
}



void IMUConversion::updateScale()
{
    float full_scale = 0.0f;

    switch (_range)
    {
        case KX134_Range::RANGE_8G:
            full_scale = 8.0f;
            break;

        case KX134_Range::RANGE_16G:
            full_scale = 16.0f;
            break;

        case KX134_Range::RANGE_32G:
            full_scale = 32.0f;
            break;

        case KX134_Range::RANGE_64G:
            full_scale = 64.0f;
            break;

        default:
            // Safety fallback (should never happen)
            full_scale = 64.0f;
            break;
    }

    // 16-bit signed range: ±32768 counts
    _scale = full_scale / 32768.0f;
}