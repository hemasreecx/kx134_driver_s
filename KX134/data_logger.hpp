#ifndef DATA_LOGGER_HPP
#define DATA_LOGGER_HPP

#include <stdint.h>
#include <stddef.h>

class DataLogger
{
public:

    struct ImuSample
    {
        uint32_t timestamp_ms;  
        int16_t  x;
        int16_t  y;
        int16_t  z;
    };

    struct Stats
    {
        uint32_t total_samples;
        uint32_t total_flushes;
        uint32_t write_failures;
        uint32_t buffer_overflows;
    };

    // buffer_size should be >= 50
    explicit DataLogger(size_t buffer_size);

    bool init();

    // Called at 50 Hz
    bool addSample(uint32_t timestamp_ms,
                   int16_t x,
                   int16_t y,
                   int16_t z);

    // Called at 1 Hz
    bool flushToSD();

    Stats getStats() const;

private:

    bool writeBlockToSD(const ImuSample* data, size_t count);

    ImuSample* _buffer;
    size_t     _buffer_size;
    size_t     _index;

    Stats      _stats;
};

#endif