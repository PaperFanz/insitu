#ifndef insitu_plugins_Stats_HPP
#define insitu_plugins_Stats_HPP

#include <pluginlib/class_list_macros.h>
#include <insitu/filter.hpp>

namespace insitu_plugins {

class Stats : public insitu::Filter
{

public:
    Stats(void);

    const cv::Mat apply (void);

    bool hasSettingEditor(void)
    {
        return true;
    }

private:
    void filterInit(void);

    void onDelete(void);

    float readCPUPercent(void);

    float readMemPercent(void);

    float readDiskPercent(void);

    void drawtorect(cv::Mat& mat, cv::Rect target, const std::string& str,
                int face = cv::FONT_HERSHEY_PLAIN, int thickness = 1,
                cv::Scalar color = cv::Scalar(0, 0, 0, 255));

    /* variables for calculating cpu percent */
    float ptot = 0;
    float pidle = 0;
    float pcpu = 0;

}; // end class Stats

} // end namespace insitu_plugins

#endif // end insitu_plugins_Stats_HPP

