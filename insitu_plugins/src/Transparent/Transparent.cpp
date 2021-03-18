#include <Transparent/Transparent.hpp>
    #include <Transparent/Transparent_dialog.hpp>

    namespace insitu_plugins {

    /*
        Filter Implementation
    */
    Transparent::Transparent(void)
    {
        // TODO instantiation code
    }

    void Transparent::onInit(void)
    {
        settingsDialog = new TransparentDialog(this);

        // TODO initialization code
    }

    void Transparent::onDelete(void)
    {
        // TODO cleanup code
    }


    const cv::Mat Transparent::apply (void)
    {
        /*
            Create a transparent image to construct your overlay on
        */
        double alpha = settings.get("alpha", 0.5).asDouble();
        cv::Mat ret = cv::Mat(
            settings.get("height", 300).asInt(),
            settings.get("width", 300).asInt(),
            CV_8UC4,
            cv::Scalar(
                settings.get("red", 255).asInt(), 
                settings.get("green", 255).asInt(), 
                settings.get("blue", 255).asInt(), 
                (int)(alpha * 255)
            )
        );

        // TODO edit your overlay from user settings 
        // e.g. settings.get("key", defaultValue).asType()

        return ret;
    }

    } // end namespace insitu_plugins

    PLUGINLIB_EXPORT_CLASS(insitu_plugins::Transparent, insitu::Filter);
    