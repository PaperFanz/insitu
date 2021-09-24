#ifndef{ 0 } _{ 1 } _HPP
#define{ 0 } _{ 1 } _HPP

#include <pluginlib/class_list_macros.h>
#include <insitu/filter.hpp>

namespace
{
0
}
{
    {
        class
        {
            1
        } : public insitu::Filter{ {

            public : { 1 }(void);

        const cv::Mat apply(void);

        bool hasSettingEditor(void)
        {
            {
                return true;
            }
        }

    private:
        void filterInit(void);

        void onDelete(void);
    }
};    // end class {1}
}
}    // end namespace {0}

#endif    // end {0}_{1}_HPP
