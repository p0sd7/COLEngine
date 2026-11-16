#ifndef BODY_H
#define BODY_H
#include "Math.h"

struct Body {
	unsigned int id = 0;

	Vec2 position;
	Vec2 velocity;
	Vec2 half_size = {10.0f, 10.0f};

	float inverse_mass = 1.0f;

};

#endif 