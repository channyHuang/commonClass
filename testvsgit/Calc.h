#pragma once
class Calc
{
public:
	Calc() {};
	~Calc() {};

	bool getState() { m_bState = !m_bState; return !m_bState; }

private:
	bool m_bState = false;
};

