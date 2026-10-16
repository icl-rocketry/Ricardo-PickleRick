#pragma once

#include <vector>


class Controllable {
	public:
		virtual void control(std::vector<float> u) = 0;
		virtual bool isActive() const { return true; };
};