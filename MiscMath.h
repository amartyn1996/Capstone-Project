#ifndef MiscMath_h
#define MiscMath_h

#define DEGREES_TO_RADIANS 0.01745329251 // PI / 180

class MiscMath {
	public:
		static void normalize(float &X, float &Y, float &Z);
		static float vecLength(float X, float Y, float Z);
};

#endif
