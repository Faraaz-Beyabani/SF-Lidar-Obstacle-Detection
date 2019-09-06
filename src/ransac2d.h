/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting
#pragma once

#ifndef RANSAC_H_
#define RANSAC_H_

#include <unordered_set>

template<typename PointT> 
inline std::unordered_set<int> RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	int num_pts = cloud->points.size();
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
  
	for(size_t i = 0; i < maxIterations; i++)
	{
		PointT pt1;
		PointT pt2;
		PointT pt3;

		std::unordered_set<int> inliersTemp;

		while(inliersTemp.size() < 3)
		{
			inliersTemp.insert(rand() % num_pts);
		}

		auto it = inliersTemp.begin();
		pt1 = cloud->points[*it];
		it++;
		pt2 = cloud->points[*it];
		it++;
		pt3 = cloud->points[*it];
      
		float x1 = pt1.x, x2 = pt2.x, x3 = pt3.x;
		float y1 = pt1.y, y2 = pt2.y, y3 = pt3.y;
		float z1 = pt1.z, z2 = pt2.z, z3 = pt3.z;
	
		// A is equivalent to i, the x component of the normal vector of the plane formed by pt1, pt2, and pt3
		// B is equivalent to j, the y component
		// C is equivalent to k, the z component
		double A = ((y2 - y1) * (z3 - z1)) - ((z2 - z1) * (y3 - y1));
		double B = ((z2 - z1) * (x3 - x1)) - ((x2 - x1) * (z3 - z1));
		double C = ((x2 - x1) * (y3 - y1)) - ((y2 - y1) * (x3 - x1));
		double D = -(A * x1 + B * y1 + C * z1);
		
		for(size_t j = 0; j < num_pts; j++)
		{
			if(inliersTemp.count(j) > 0) continue;

			auto other = cloud->points[j];
			auto x4 = other.x;
			auto y4 = other.y;
			auto z4 = other.z;
          
			double distance = fabs(A * x4 + B * y4 + C * z4 + D) / sqrt(A*A + B*B + C*C);

			if(distance <= distanceTol)
			{
				inliersTemp.insert(j);
			}
		}

		if(inliersTemp.size() > inliersResult.size())
		{
			inliersResult = inliersTemp;
		}
	}

	return inliersResult;
}

#endif
