#ifndef MATH_H
#define MATH_H

struct Vec2 {
	float x = 0.0f;
	float y = 0.0f;

	Vec2 operator+(const Vec2& other) const {
		return { x + other.x, y + other.y };
	}

	Vec2 operator-(const Vec2& other) const {
		return { x - other.x, y - other.y };
	}

	Vec2 operator*(float scalar) const {
		return { x * scalar, y * scalar };
	}
	
	Vec2& operator+=(const Vec2& other) {
		x += other.x;
		y += other.y;
		return *this;
	}
};

inline float dot(const Vec2& a, const Vec2& b) {
	return a.x * b.x + a.y * b.y;
};

#endif