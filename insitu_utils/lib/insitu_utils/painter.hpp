#ifndef insitu_utils_PAINTER_HPP
#define insitu_utils_PAINTER_HPP

/* OpenCV includes */
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

namespace insitu_utils
{

namespace Color
{
    const cv::Scalar maroon(128,0,0);
    const cv::Scalar darkred(139,0,0);
    const cv::Scalar brown(165,42,42);
    const cv::Scalar firebrick(178,34,34);
    const cv::Scalar crimson(220,20,60);
    const cv::Scalar red(255,0,0);
    const cv::Scalar tomato(255,99,71);
    const cv::Scalar coral(255,127,80);
    const cv::Scalar indianred(205,92,92);
    const cv::Scalar lightcoral(240,128,128);
    const cv::Scalar darksalmon(233,150,122);
    const cv::Scalar salmon(250,128,114);
    const cv::Scalar lightsalmon(255,160,122);
    const cv::Scalar orangered(255,69,0);
    const cv::Scalar darkorange(255,140,0);
    const cv::Scalar orange(255,165,0);
    const cv::Scalar gold(255,215,0);
    const cv::Scalar darkgoldenrod(184,134,11);
    const cv::Scalar goldenrod(218,165,32);
    const cv::Scalar palegoldenrod(238,232,170);
    const cv::Scalar darkkhaki(189,183,107);
    const cv::Scalar khaki(240,230,140);
    const cv::Scalar olive(128,128,0);
    const cv::Scalar yellow(255,255,0);
    const cv::Scalar yellowgreen(154,205,50);
    const cv::Scalar darkolivegreen(85,107,47);
    const cv::Scalar olivedrab(107,142,35);
    const cv::Scalar lawngreen(124,252,0);
    const cv::Scalar chartreuse(127,255,0);
    const cv::Scalar greenyellow(173,255,47);
    const cv::Scalar darkgreen(0,100,0);
    const cv::Scalar green(0,128,0);
    const cv::Scalar forestgreen(34,139,34);
    const cv::Scalar lime(0,255,0);
    const cv::Scalar limegreen(50,205,50);
    const cv::Scalar lightgreen(144,238,144);
    const cv::Scalar palegreen(152,251,152);
    const cv::Scalar darkseagreen(143,188,143);
    const cv::Scalar mediumspringgreen(0,250,154);
    const cv::Scalar springgreen(0,255,127);
    const cv::Scalar seagreen(46,139,87);
    const cv::Scalar mediumaquamarine(102,205,170);
    const cv::Scalar mediumseagreen(60,179,113);
    const cv::Scalar lightseagreen(32,178,170);
    const cv::Scalar darkslategray(47,79,79);
    const cv::Scalar teal(0,128,128);
    const cv::Scalar darkcyan(0,139,139);
    const cv::Scalar aqua(0,255,255);
    const cv::Scalar cyan(0,255,255);
    const cv::Scalar lightcyan(224,255,255);
    const cv::Scalar darkturquoise(0,206,209);
    const cv::Scalar turquoise(64,224,208);
    const cv::Scalar mediumturquoise(72,209,204);
    const cv::Scalar paleturquoise(175,238,238);
    const cv::Scalar aquamarine(127,255,212);
    const cv::Scalar powderblue(176,224,230);
    const cv::Scalar cadetblue(95,158,160);
    const cv::Scalar steelblue(70,130,180);
    const cv::Scalar cornflowerblue(100,149,237);
    const cv::Scalar deepskyblue(0,191,255);
    const cv::Scalar dodgerblue(30,144,255);
    const cv::Scalar lightblue(173,216,230);
    const cv::Scalar skyblue(135,206,235);
    const cv::Scalar lightskyblue(135,206,250);
    const cv::Scalar midnightblue(25,25,112);
    const cv::Scalar navy(0,0,128);
    const cv::Scalar darkblue(0,0,139);
    const cv::Scalar mediumblue(0,0,205);
    const cv::Scalar blue(0,0,255);
    const cv::Scalar royalblue(65,105,225);
    const cv::Scalar blueviolet(138,43,226);
    const cv::Scalar indigo(75,0,130);
    const cv::Scalar darkslateblue(72,61,139);
    const cv::Scalar slateblue(106,90,205);
    const cv::Scalar mediumslateblue(123,104,238);
    const cv::Scalar mediumpurple(147,112,219);
    const cv::Scalar darkmagenta(139,0,139);
    const cv::Scalar darkviolet(148,0,211);
    const cv::Scalar darkorchid(153,50,204);
    const cv::Scalar mediumorchid(186,85,211);
    const cv::Scalar purple(128,0,128);
    const cv::Scalar thistle(216,191,216);
    const cv::Scalar plum(221,160,221);
    const cv::Scalar violet(238,130,238);
    const cv::Scalar fuchsia(255,0,255);
    const cv::Scalar orchid(218,112,214);
    const cv::Scalar mediumvioletred(199,21,133);
    const cv::Scalar palevioletred(219,112,147);
    const cv::Scalar deeppink(255,20,147);
    const cv::Scalar hotpink(255,105,180);
    const cv::Scalar lightpink(255,182,193);
    const cv::Scalar pink(255,192,203);
    const cv::Scalar antiquewhite(250,235,215);
    const cv::Scalar beige(245,245,220);
    const cv::Scalar bisque(255,228,196);
    const cv::Scalar blanchedalmond(255,235,205);
    const cv::Scalar wheat(245,222,179);
    const cv::Scalar cornsilk(255,248,220);
    const cv::Scalar lemonchiffon(255,250,205);
    const cv::Scalar lightgoldenrodyellow(250,250,210);
    const cv::Scalar lightyellow(255,255,224);
    const cv::Scalar saddlebrown(139,69,19);
    const cv::Scalar sienna(160,82,45);
    const cv::Scalar chocolate(210,105,30);
    const cv::Scalar peru(205,133,63);
    const cv::Scalar sandybrown(244,164,96);
    const cv::Scalar burlywood(222,184,135);
    const cv::Scalar tan(210,180,140);
    const cv::Scalar rosybrown(188,143,143);
    const cv::Scalar moccasin(255,228,181);
    const cv::Scalar navajowhite(255,222,173);
    const cv::Scalar peachpuff(255,218,185);
    const cv::Scalar mistyrose(255,228,225);
    const cv::Scalar lavenderblush(255,240,245);
    const cv::Scalar linen(250,240,230);
    const cv::Scalar oldlace(253,245,230);
    const cv::Scalar papayawhip(255,239,213);
    const cv::Scalar seashell(255,245,238);
    const cv::Scalar mintcream(245,255,250);
    const cv::Scalar slategray(112,128,144);
    const cv::Scalar lightslategray(119,136,153);
    const cv::Scalar lightsteelblue(176,196,222);
    const cv::Scalar lavender(230,230,250);
    const cv::Scalar floralwhite(255,250,240);
    const cv::Scalar aliceblue(240,248,255);
    const cv::Scalar ghostwhite(248,248,255);
    const cv::Scalar honeydew(240,255,240);
    const cv::Scalar ivory(255,255,240);
    const cv::Scalar azure(240,255,255);
    const cv::Scalar snow(255,250,250);
    const cv::Scalar black(0,0,0);
    const cv::Scalar dimgray(105,105,105);
    const cv::Scalar gray(128,128,128);
    const cv::Scalar darkgray(169,169,169);
    const cv::Scalar silver(192,192,192);
    const cv::Scalar lightgray(211,211,211);
    const cv::Scalar gainsboro(220,220,220);
    const cv::Scalar whitesmoke(245,245,245);
    const cv::Scalar white(255,255,255);
} /* namespace Color */

class Painter
{
public:
    static void drawtorect(cv::Mat& mat, cv::Rect target,
                           const std::string& str,
                           int face = cv::FONT_HERSHEY_PLAIN, int thickness = 1,
                           cv::Scalar color = Color::white);
};

}    // namespace insitu_utils

#endif /* insitu_utils_PAINTER_HPP */
