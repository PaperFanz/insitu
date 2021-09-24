#include <Stats/Stats.hpp>
#include <Stats/Stats_dialog.hpp>

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <sys/statvfs.h>
#include <thread>

using namespace std::chrono_literals;

namespace insitu_plugins
{
/*
    Filter Implementation
*/
Stats::Stats(void)
{
    // TODO instantiation code
}

void Stats::filterInit(void)
{
    settingsDialog = new StatsDialog(this);
    setSize(QSize(200, 120));
}

void Stats::onDelete(void)
{
}

void Stats::drawtorect(cv::Mat& mat, cv::Rect target, const std::string& str,
                       int face, int thickness, cv::Scalar color)
{
    cv::Size rect = cv::getTextSize(str, face, 1.0, thickness, 0);
    double scalex = (double)target.width / (double)rect.width;
    double scaley = (double)target.height / (double)rect.height;
    double scale = std::min(scalex, scaley);
    int marginx =
        scale == scalex ?
            0 :
            (int)((double)target.width * (scalex - scale) / scalex * 0.5);
    int marginy =
        scale == scaley ?
            0 :
            (int)((double)target.height * (scaley - scale) / scaley * 0.5);
    cv::putText(
        mat, str,
        cv::Point(target.x + marginx, target.y + target.height - marginy), face,
        scale, color, thickness, cv::LINE_AA, false);
}

const cv::Mat Stats::apply(void)
{
    std::this_thread::sleep_for(1s);
    cv::Scalar textColor = cv::Scalar(
        settings.get("r", 0).asInt(), settings.get("g", 0).asInt(),
        settings.get("b", 0).asInt(), settings.get("a", 255).asInt());
    cv::Scalar bgColor =
        cv::Scalar(settings.get("r", 0).asInt(), settings.get("g", 0).asInt(),
                   settings.get("b", 0).asInt(), 0);
    /*
        Create a transparent image to construct your overlay on
    */
    cv::Mat ret = cv::Mat(height(), width(), CV_8UC4, bgColor);

    std::vector<std::string> stats;

    if (settings.get("showCPU", true).asBool())
    {
        stats.push_back(
            "CPU: " + std::to_string(readCPUPercent()).substr(0, 5) + "%");
    }
    if (settings.get("showMem", true).asBool())
    {
        stats.push_back(
            "Mem: " + std::to_string(readMemPercent()).substr(0, 5) + "%");
    }
    if (settings.get("showDisk", true).asBool())
    {
        stats.push_back(
            "Disk: " + std::to_string(readDiskPercent()).substr(0, 5) + "%");
    }

    int h = 0, hdiv = height() / stats.size();
    for (const auto& str : stats)
    {
        drawtorect(ret, cv::Rect(0, h, ret.cols, hdiv), str,
                   cv::FONT_HERSHEY_PLAIN, 1, textColor);
        h += hdiv;
    }

    return ret;
}

float Stats::readCPUPercent(void)
{
    float res = 0.0;
    std::string cpu;
    std::ifstream stat;
    stat.open("/proc/stat", std::ios::in);

    if (stat.is_open())
    {
        std::getline(stat, cpu);
        std::string discard;
        float user, nice, system, idle, iowait, irq, softirq;
        std::istringstream(cpu) >> discard >> user >> nice >> system >> idle >>
            iowait >> irq >> softirq;
        float ctot = user + nice + system + idle + iowait + irq + softirq;
        float cidle = idle + iowait;
        float tdelta = ctot - ptot;
        float idelta = cidle - pidle;
        float used = tdelta - idelta;
        res = 100 * used / (tdelta + 1);
        if (res != 0)
            pcpu = res;
        else
            res = pcpu;
        ptot = ctot;
        pidle = cidle;
    }

    return res;
}

float Stats::readMemPercent(void)
{
    float res = 0.0;
    std::string mem;
    std::ifstream stat;
    stat.open("/proc/meminfo", std::ios::in);

    if (stat.is_open())
    {
        std::getline(stat, mem);
        std::string discard;
        float tot, free;
        std::istringstream(mem) >> discard >> tot;
        std::getline(stat, mem);
        std::istringstream(mem) >> discard >> free;
        res = 100 - (100 * free / tot);
    }

    return res;
}

float Stats::readDiskPercent(void)
{
    float res = 0.0;
    struct statvfs buf;

    if (statvfs("/", &buf) == 0)
    {
        float free = buf.f_bfree;
        float tot = buf.f_blocks;
        res = 100 - (100 * free / tot);
    }

    return res;
}

}    // end namespace insitu_plugins

PLUGINLIB_EXPORT_CLASS(insitu_plugins::Stats, insitu::Filter);
