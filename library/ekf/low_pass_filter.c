#include "low_pass_filter.h"

void lowPassFilter_init(LowPassFilter *filter, float cutoffFrequency)
{
    if (cutoffFrequency <= 0.0f)
        cutoffFrequency = 1.0f; // 防止除0

    filter->timeConstantSecs = 1.0f / (2.0f * M_PI * cutoffFrequency);
    filter->filteredData.x = 0.0f;
    filter->filteredData.y = 0.0f;
    filter->filteredData.z = 0.0f;
    filter->lastTimestampNs = 0;
    filter->numSamples = 0;
}

void lowPassFilter_addSample(LowPassFilter *filter, const Vector3d *sampleData, uint64_t timestampNs)
{
    lowPassFilter_addWeightedSample(filter, sampleData, timestampNs, 1.0f);
}

void lowPassFilter_addWeightedSample(LowPassFilter *filter, const Vector3d *sampleData, uint64_t timestampNs, float weight)
{
    filter->numSamples++;

    if (filter->numSamples == 1)
    {
        // 第一个样本直接赋值
        Vector3d_set(&filter->filteredData, sampleData->x, sampleData->y, sampleData->z);
        filter->lastTimestampNs = timestampNs;
        return;
    }

    // 计算加权时间差 (单位: 秒)
    float weightedDeltaSecs = weight * (timestampNs - filter->lastTimestampNs) * NANOS_TO_SECONDS;

    // 计算滤波系数 alpha
    float alpha = weightedDeltaSecs / (filter->timeConstantSecs + weightedDeltaSecs);

    Vector3d scaledFiltered = filter->filteredData;
    Vector3d_scale(&scaledFiltered, 1.0f - alpha);
    Vector3d scaledSample = *sampleData;
    Vector3d_scale(&scaledSample, alpha);
    Vector3d_add(&scaledFiltered, &scaledSample, &filter->filteredData);

    // 更新时间戳
    filter->lastTimestampNs = timestampNs;
}

const Vector3d *lowPassFilter_getFilteredData(const LowPassFilter *filter)
{
    return &filter->filteredData;
}

int lowPassFilter_getNumSamples(const LowPassFilter *filter)
{
    return filter->numSamples;
}
