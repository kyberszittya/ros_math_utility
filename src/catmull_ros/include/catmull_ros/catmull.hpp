#ifndef CATMULL_H
#define CATMULL_H

#include <limits>
#include <memory>

#include <iostream>

#include <tbb/concurrent_vector.h>

#include "vector3.hpp"
namespace catmull_ros
{
class ControlVertex
{
private:
	double t;
	Vector3 p;
	Vector3 v;

	std::shared_ptr<ControlVertex> prev;
	std::shared_ptr<ControlVertex> next;

	Vector3 a0;
	Vector3 a1; // velocity
	Vector3 a2;
	Vector3 a3;
public:
	ControlVertex(Vector3 p, double t): t(t), p(p)
	{
        		
	}

	ControlVertex(Vector3 p) : p(p)
	{

	}

	void initHermite()
	{
		double dtp1 = (next->getT() - t);
		a0 = p;
		a1 = v;
		a2 = (3.0*(next->getP() - p) / (dtp1*dtp1))
			- (next->getV() + 2.0*v) / (dtp1);
		a3 = (2.0*(p - next->getP()) / (dtp1*dtp1*dtp1)) 
			+ (next->getV() + v) / (dtp1*dtp1);
	}

	

	void initializeStart(std::shared_ptr<ControlVertex> prev,
		std::shared_ptr<ControlVertex> next)
	{
		this->prev = prev;
		this->next = next;
		this->t = 0.0;
	}
	

	void initialize(std::shared_ptr<ControlVertex> prev, 
		std::shared_ptr<ControlVertex> next)
	{
		this->prev = prev;
		this->next = next;
		double dx = p.x - prev->getP().x;
		double dy = p.y - prev->getP().y;
		this->t = sqrt(dx*dx + dy * dy)+prev->getT();
	}

	void initializeVelocity()
	{
		Vector3 pm1 = prev->getP();
		double tm1 = 0.0;
		double dxm1 = p.x - pm1.x;
		double dym1 = p.y - pm1.y;
		double t0 = sqrt(dxm1*dxm1 + dym1 * dym1);

		Vector3 pp1 = next->getP();
		double dxp1 = pp1.x - p.x;
		double dyp1 = pp1.y - p.y;
		double t1 = sqrt(dxp1*dxp1 + dyp1 * dyp1) + t0;

		double dtp1 = (t1 - t0);
		double dtm1 = (t0 - tm1);
		v = (((next->getP() - p) / dtp1) +
			((p - prev->getP()) / dtm1)) / 2.0;
	}

	void initHermiteClose()
	{
		Vector3 pp1 = next->getP();
		double dxp1 = pp1.x - p.x;
		double dyp1 = pp1.y - p.y;
		double t1 = sqrt(dxp1*dxp1 + dyp1 * dyp1) + t;
		double dtp1 = (t1 - t);
		a0 = p;
		a1 = v;
		a2 = (3.0*(next->getP() - p) / (dtp1*dtp1))
			- (next->getV() + 2.0*v) / (dtp1);
		a3 = (2.0*(p - next->getP()) / (dtp1*dtp1*dtp1))
			+ (next->getV() + v) / (dtp1*dtp1);
	}

	Vector3 ddhermite(double t0)
	{
		double dt = t0 - t;
		return 6.0*a3 *dt + 2.0*a2;
	}

	Vector3 dhermite(double t0)
	{
		double dt = t0 - t;
		return 3.0*a3 * (dt*dt) + 2.0*a2 * dt + a1;
	}

	Vector3 hermite(double t0)
	{
		double dt = t0 - t;
		return a3 * (dt*dt*dt) + a2 * dt*dt + a1 * dt + a0;
	}

	double getT()
	{
		return t;
	}

	Vector3 getP()
	{
		return p;
	}

	Vector3 getV()
	{
		return v;
	}

};

class CatmullSpline
{
private:
	tbb::concurrent_vector<std::shared_ptr<ControlVertex> > vertices;
	double min_t;
	double max_t;
	bool closed;
public:
	CatmullSpline():
		min_t(std::numeric_limits<double>::max()),
		max_t(std::numeric_limits<double>::min())
	{
		closed = false;
	}

	Vector3 r(double t) 
	{
		for (int i = 0; i < vertices.size()-1; i++)
		{
			if (vertices[i]->getT()<=t && vertices[i+1]->getT()>t) 
			{
				return vertices[i]->hermite(t);
			}
		}
		if (closed)
		{
			if (vertices[vertices.size() - 1]->getT() <= t && max_t > t)
			{
				return vertices[vertices.size() - 1]->hermite(t);
			}
		}
		
		Vector3 res;
		return res;
	}

	Vector3 ddr(double t)
	{
		for (int i = 0; i < vertices.size() - 1; i++)
		{
			if (vertices[i]->getT() <= t && vertices[i + 1]->getT()>t)
			{
				return vertices[i]->ddhermite(t);
			}
		}
		if (closed)
		{
			if (vertices[vertices.size() - 1]->getT() <= t && max_t > t)
			{
				return vertices[vertices.size() - 1]->ddhermite(t);
			}
		}

		Vector3 res;
		return res;
	}

	Vector3 dr(double t)
	{
		for (int i = 0; i < vertices.size() - 1; i++)
		{
			if (vertices[i]->getT() <= t && vertices[i + 1]->getT()>t)
			{
				return vertices[i]->dhermite(t);
			}
		}
		if (closed)
		{
			if (vertices[vertices.size() - 1]->getT() <= t && max_t > t)
			{
				return vertices[vertices.size() - 1]->dhermite(t);
			}
		}

		Vector3 res;
		return res;
	}

	void addControlVertex(Vector3 p)
	{
		std::shared_ptr<ControlVertex> cv =
			std::shared_ptr<ControlVertex>(new ControlVertex(p));
		vertices.push_back(cv);
		
	}

	void addControlVertex(Vector3 p, double t)
	{
		std::shared_ptr<ControlVertex> cv = 
			std::shared_ptr<ControlVertex>(new ControlVertex(p, t));
		vertices.push_back(cv);
	}

	std::shared_ptr<ControlVertex> getControlVertex(int i)
	{
		return vertices[i];
	}

	double getMinT()
	{
		return min_t;
	}

	double getMaxT()
	{
		return max_t;
	}

	void close()
	{
		closed = true;

		Vector3 p = vertices[vertices.size() - 1]->getP();

		Vector3 pp1 = vertices[0]->getP();
		double dxp1 = pp1.x - p.x;
		double dyp1 = pp1.y - p.y;
		double t1 = sqrt(dxp1*dxp1 + dyp1 * dyp1) + vertices[vertices.size() - 1]->getT();

		max_t = t1;
	}

	void constructClose()
	{
		
		vertices[0]->initializeStart(vertices[vertices.size() - 1],
			vertices[1]);
		for (int i = 1; i < vertices.size()-1; i++)
		{
			vertices[i]->initialize(vertices[i - 1], vertices[i + 1]);
		}
		vertices[vertices.size() - 1]->initialize(vertices[vertices.size() - 2], vertices[0]);
		close();

		for (int i = 0; i < vertices.size(); i++)
		{
			vertices[i]->initializeVelocity();
		}
		for (const auto& v : vertices)
		{
			double tmp = v->getT();
			if (tmp < min_t)
			{
				min_t = tmp;
			}
			if (tmp > max_t)
			{
				max_t = tmp;
			}
		}
		for (int i = 0; i < vertices.size()-1; i++)
		{
			vertices[i]->initHermite();
		}
		vertices[vertices.size() - 1]->initHermiteClose();
	}
};
}
#endif