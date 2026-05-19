#ifndef __LOW_PASS_FILTER_H__
#define __LOW_PASS_FILTER_H__

#include "vector3d.h"
#include <stdint.h>

#define NANOS_TO_SECONDS 1e-9f

typedef struct{
    float timeConstantSecs;     // 时间常数 = 1 / (2πf)
    Vector3d filteredData;       // 滤波后的输出
    uint64_t lastTimestampNs;     // 上一次时间戳（纳秒）
    uint64_t numSamples;              // 样本计数
} LowPassFilter;

void lowPassFilter_init(LowPassFilter *filter, float cutoffFrequency);
void lowPassFilter_addSample(LowPassFilter *filter, const Vector3d *sampleData, uint64_t timestampNs);
void lowPassFilter_addWeightedSample(LowPassFilter *filter, const Vector3d *sampleData, uint64_t timestampNs, float weight);
const Vector3d *lowPassFilter_getFilteredData(const LowPassFilter *filter);
int lowPassFilter_getNumSamples(const LowPassFilter *filter);

#endif
