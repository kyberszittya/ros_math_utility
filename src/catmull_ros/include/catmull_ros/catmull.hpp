/*
 * catmull.hpp
 * 
 * Header file for an ROS based general-purpose Catmull-Rom Spline
 * 
 * Hajdu Csaba (kyberszittya)
 */
#ifndef CATMULL_H
#define CATMULL_H

#include <limits>
#include <memory>

#include <iostream>

#include <tbb/concurrent_vector.h>

#include "vector3.hpp"
namespace catmull_ros
{
/**
 * catmull_ros::ControlVertex
 * 
 * Class to represent the control vertex of the Catmull-Rom spline 
 * 
 * Hajdu Csaba (kyberszittya)
*/
class ControlVertex
{
private:
	double t;
	Vector3 p;
	Vector3 v;

	std::shared_ptr<ControlVertex> prev;
	std::shared_ptr<ControlVertex> next;

	//Vector3 a0;
	//Vector3 a1; // velocity
	Vector3 a2;
	Vector3 a3;
public:
	ControlVertex(Vector3 p, double t=0.0): t(t), p(p)
	{
        		
	}


	/*
	Get zero-order Hermite coefficient (position)
	*/
	Vector3& A0()
	{
		return p;
	}

	/*
	Get first Hermite coefficient (velocity, linear component)
	*/
	Vector3& A1()
	{
		return v;
	}

	/*
	Get second Hermite coefficient (quadratic component)
	*/
	Vector3& A2()
	{
		return a2;
	}

	/*
	Get third Hermite coefficient (cubic component)
	*/
	Vector3& A3()
	{
		return a3;
	}

	/*
	Initialize Hermite according to set parameters
	*/
	void InitHermite()
	{
		if (next!=nullptr)
		{
			double dtp1 = (next->T() - t);			
			a2 = (3.0*(next->P() - p) / (dtp1*dtp1))
				- (next->V() + 2.0*v) / (dtp1);
			a3 = (2.0*(p - next->P()) / (dtp1*dtp1*dtp1)) 
				+ (next->V() + v) / (dtp1*dtp1);
		}
		
	}

	void InitializeLoop(std::shared_ptr<ControlVertex> prev,
		std::shared_ptr<ControlVertex> next)
	{
		this->prev = prev;
		this->next = next;
		this->t = 0.0;
	}	

	void Initialize(std::shared_ptr<ControlVertex> prev,
		std::shared_ptr<ControlVertex> next)
	{
		this->prev = prev;
		this->next = next;		
		double dx = p.coords[0] - prev->P().coords[0];
		double dy = p.coords[1] - prev->P().coords[1];
		this->t = sqrt(dx*dx + dy * dy)+prev->T();
	}

	void InitializeStart(std::shared_ptr<ControlVertex> next)
	{
		this->prev = nullptr;
		this->next = next;		
		this->t = 0;
	}

	void InitializeEnd(std::shared_ptr<ControlVertex> prev)
	{
		this->prev = prev;
		this->next = nullptr;		
		double dx = p.coords[0] - prev->P().coords[0];
		double dy = p.coords[1] - prev->P().coords[1];
		double dz = p.coords[2] - prev->P().coords[2];
		this->t = sqrt(dx*dx + dy * dy + dz * dz)+prev->T();
	}

	void InitializeVelocity()
	{
		if (prev!=nullptr)
		{
			Vector3 pm1 = prev->P();
			double tm1 = 0.0;
			double dxm1 = p.coords[0] - pm1.coords[0];
			double dym1 = p.coords[1] - pm1.coords[1];
			double t0 = sqrt(dxm1*dxm1 + dym1 * dym1);
			if (next==nullptr)
			{
				double dtm1 = (t0 - tm1);
				v = (p - prev->P()) / dtm1;
			}
			else
			{
				Vector3 pp1 = next->P();
				double dxp1 = pp1.coords[0] - p.coords[0];
				double dyp1 = pp1.coords[1] - p.coords[1];
				double t1 = sqrt(dxp1*dxp1 + dyp1 * dyp1) + t0;

				double dtp1 = (t1 - t0);
				double dtm1 = (t0 - tm1);
				v = (((next->P() - p) / dtp1) +
					((p - prev->P()) / dtm1)) / 2.0;
			}
			
		}
		else if (next!=nullptr) // This is the last point!
		{
			Vector3 pp1 = next->P();
			double dxp1 = pp1.coords[0] - p.coords[0];
			double dyp1 = pp1.coords[1] - p.coords[1];
			double t1 = sqrt(dxp1*dxp1 + dyp1 * dyp1);

			v = (next->P() - p) / t1;
		}
		else
		{
			v = Vector3();
		}
	}

	void InitHermiteClose()
	{
		Vector3 pp1 = next->P();
		double dxp1 = pp1.coords[0] - p.coords[0];
		double dyp1 = pp1.coords[1] - p.coords[1];
		double t1 = sqrt(dxp1*dxp1 + dyp1 * dyp1) + t;
		double dtp1 = (t1 - t);
		a2 = (3.0*(next->P() - p) / (dtp1*dtp1))
			- (next->V() + 2.0*v) / (dtp1);
		a3 = (2.0*(p - next->P()) / (dtp1*dtp1*dtp1))
			+ (next->V() + v) / (dtp1*dtp1);
	}

	Vector3 ddhermite(double t0)
	{
		double dt = t0 - t;
		return 6.0*a3 *dt + 2.0*a2;
	}

	Vector3 dhermite(double t0)
	{
		double dt = t0 - t;
		return 3.0*a3 * (dt*dt) + 2.0*a2 * dt + v;
	}

	Vector3 Hermite(double t0)
	{
		double dt = t0 - t;
		return a3 * (dt*dt*dt) + a2 * dt*dt + v * dt + p;
	}

	double T()
	{
		return t;
	}

	Vector3 P()
	{
		return p;
	}

	Vector3 V()
	{
		return v;
	}

};

/**
 * catmull_ros::CatmullSpline
 * 
 * Class to represent the Catmull-Rom Spline
 * Contains:
 * - Control vertices
 * - Zero order function to calculate position of the spline according to parameter t
 * - First-order derivative function to calculate tangential velocity at a given point of parameter
 * - Second-order derivative function to calculate tangential acceleration at a given point of parameter
 * 
 * 
 * */
class CatmullSpline
{
private:
	tbb::concurrent_vector<std::shared_ptr<ControlVertex> > vertices;
	double min_t;
	double max_t;
	bool closed;
public:
	/**
	Create our spline: the minimal and maximal t parameter
	shall be defined as supremum values
	*/
	CatmullSpline():
		min_t(std::numeric_limits<double>::max()),
		max_t(std::numeric_limits<double>::min())
	{
		closed = false;
	}

	/**
	Zero-order function of position according to parameter t
	*/
	Vector3 r(double t) 
	{
		for (int i = 0; i < vertices.size()-1; i++)
		{
			if (vertices[i]->T()<=t && vertices[i+1]->T()>t) 
			{
				return vertices[i]->Hermite(t);
			}
		}
		if (closed && vertices[vertices.size() - 1]->T() <= t && max_t > t)
		{
			return vertices[vertices.size() - 1]->Hermite(t);
		}
		else if (!closed && vertices[vertices.size() - 1]->T() == t)
		{
			return vertices[vertices.size() - 1]->Hermite(max_t);
		}
		
		Vector3 res;
		return res;
	}
	/**
	Second-order function of position according to parameter t
	*/
	Vector3 ddr(double t)
	{
		for (int i = 0; i < vertices.size() - 1; i++)
		{
			if (vertices[i]->T() <= t && vertices[i + 1]->T()>t)
			{
				return vertices[i]->ddhermite(t);
			}
		}
		if (closed &&
			vertices[vertices.size() - 1]->T() <= t && max_t > t)
		{
			return vertices[vertices.size() - 1]->ddhermite(t);
		}
		
		Vector3 res;
		return res;
	}

	/**
	First-order function of position according to parameter t
	*/
	Vector3 dr(double t)
	{
		for (int i = 0; i < vertices.size() - 1; i++)
		{
			if (vertices[i]->T() <= t && vertices[i + 1]->T()>t)
			{
				return vertices[i]->dhermite(t);
			}
		}
		if (closed
			&&	
			vertices[vertices.size() - 1]->T() <= t && max_t > t)
		{
				return vertices[vertices.size() - 1]->dhermite(t);
		}

		Vector3 res;
		return res;
	}

	/**
	Add control vertex to the list of control vertices
	This control vertex will be defined at the input position
	*/
	void AddControlVertex(Vector3 p)
	{
		std::shared_ptr<ControlVertex> cv =
			std::shared_ptr<ControlVertex>(new ControlVertex(p));
		vertices.push_back(cv);
		
	}

	/**
	Add control vertex to the list of control vertices with arbitrary parameter
	This control will be defined at the input position
	*/
	void AddControlVertex(Vector3 p, double t)
	{
		std::shared_ptr<ControlVertex> cv = 
			std::shared_ptr<ControlVertex>(new ControlVertex(p, t));
		vertices.push_back(cv);
	}

	/**
	Get the i-th control vertex
	*/
	std::shared_ptr<ControlVertex> GetControlVertex(int i)
	{
		return vertices[i];
	}

	double GetMinT()
	{
		return min_t;
	}

	double GetMaxT()
	{
		return max_t;
	}

	/**
	Close the Catmull-Rom spline with selecting 
	the first control vertex as the loop-end point
	*/
	void Close()
	{
		closed = true;

		Vector3 p = vertices[vertices.size() - 1]->P();

		Vector3 pp1 = vertices[0]->P();
		double dxp1 = pp1.coords[0] - p.coords[0];
		double dyp1 = pp1.coords[1] - p.coords[1];
		double t1 = sqrt(dxp1*dxp1 + dyp1 * dyp1) + vertices[vertices.size() - 1]->T();

		max_t = t1;
	}

	std::vector<Vector3> InterpolateLines(const int steps)
	{
		std::vector<Vector3> lines(steps);
		return lines;
	}

	void Construct()
	{
		if (vertices.size()!=1)
		{
			vertices[0]->InitializeStart(vertices[1]);
			for (int i = 1; i < vertices.size()-1; i++)
			{
				vertices[i]->Initialize(vertices[i - 1], vertices[i + 1]);
			}

			vertices[vertices.size()-1]->InitializeEnd(
				vertices[vertices.size()-2]);
			for (int i = 0; i < vertices.size(); i++)
			{
				vertices[i]->InitializeVelocity();
			}
			for (const auto& v : vertices)
			{
				double tmp = v->T();
				if (tmp < min_t)
				{
					min_t = tmp;
				}
				if (tmp > max_t)
				{
					max_t = tmp;
				}
			}
			for (int i = 0; i <= vertices.size()-1; i++)
			{
				vertices[i]->InitHermite();
			}
		}
		
	}

	
	void ConstructLoop()
	{
		
		vertices[0]->InitializeLoop(vertices[vertices.size() - 1],
			vertices[1]);
		for (int i = 1; i < vertices.size()-1; i++)
		{
			vertices[i]->Initialize(vertices[i - 1], vertices[i + 1]);
		}
		vertices[vertices.size() - 1]->Initialize(vertices[vertices.size() - 2], vertices[0]);
		Close();

		for (int i = 0; i < vertices.size(); i++)
		{
			vertices[i]->InitializeVelocity();
		}
		for (const auto& v : vertices)
		{
			double tmp = v->T();
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
			vertices[i]->InitHermite();
		}
		vertices[vertices.size() - 1]->InitHermiteClose();
	}
};
}
#endif