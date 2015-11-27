#ifndef VIEWERCOLORS_H_
#define VIEWERCOLORS_H_

namespace G2D
{
    struct ViewerColor
    {
        unsigned char r;
        unsigned char g;
        unsigned char b;
    };

    class ViewerColors
    {
    public:
        static constexpr ViewerColor Banana = { 227, 207, 87 };
        static constexpr ViewerColor Tomato = { 255, 99, 71 };
        static constexpr ViewerColor Red = { 255, 0, 0 };
    };
}

#endif
