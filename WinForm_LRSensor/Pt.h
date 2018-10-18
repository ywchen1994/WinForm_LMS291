#pragma once
#include<vector>
#include<math.h>
#include<vector>
#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"
using namespace std;
typedef unsigned int uint;


class Pt
{
public:
	Pt();
	Pt(double _x, double _y);
	Pt(double _x, double _y, double _range, double _theta);
	double x;
	double y;
	double range;
	double theta;
	double velcity;

	bool is_visited;
	int clusterId;
	bool isCore;
	vector<uint16_t>ArrivalPoints;
	cv::KalmanFilter KF;
	bool is_KF_init;
	void KF_initial();
};

int EuclidCluster(vector<Pt>& _vec, double eps);
int DBSCAN(std::vector<Pt>& _vec, double Eps, int MinPts);
void CorePointCluster(Pt &P, vector<Pt>&RawData, uint16_t clusterId);
inline double get_Distance(Pt P1, Pt P2);
vector<vector<Pt>>Cluster2List(vector<Pt>&XYcord, int NoObj);
static void FindClosePoint(vector<Pt>&NewPoints, vector<Pt>&oldPoints, double timeInterval, double Speed)
{
	double Ylim = 90.0f*(1000.0f / 36.0f)*timeInterval;
	/*double Xlim = 200 * timeInterval;
	Speed = Speed * 250 / 9;*/
	uint Nlength = NewPoints.size();
	uint Olength = oldPoints.size();
	vector<Pt> KF_oldPtPredict;
	for (uint16_t j = 0; j < Olength; j++)
	{
		cv::Mat prediction = oldPoints[j].KF.predict();
		KF_oldPtPredict.push_back(Pt(prediction.at<double>(0), prediction.at<double>(1)));
	}
	////
	for (uint16_t i = 0; i < Nlength; i++)
	{
		double minDistant = 8000;
		double distant = 0;
		uint minPointIndex = 0;
		for (uint16_t j = 0; j < Olength; j++)
		{
			distant = get_Distance(NewPoints[i], KF_oldPtPredict[j]);
			if ((distant < minDistant))
			{
				minPointIndex = j;
				minDistant = distant;
			}
		}
		if (minDistant <= Ylim)
		{
				cv::Mat  measurement = cv::Mat::zeros(2, 1, CV_64FC1);
				measurement.at<double>(0) = NewPoints[i].x;
				measurement.at<double>(1) = NewPoints[i].y;
				oldPoints[minPointIndex].KF.correct(measurement);
				NewPoints[i].x = KF_oldPtPredict[minPointIndex].x;
				NewPoints[i].y = KF_oldPtPredict[minPointIndex].y;
				NewPoints[i].KF = oldPoints[minPointIndex].KF;
				NewPoints[i].velcity = get_Distance(NewPoints[i], oldPoints[minPointIndex]) / timeInterval;
				if ((NewPoints[i].y - oldPoints[minPointIndex].y) < 0)
					NewPoints[i].velcity = -NewPoints[i].velcity;
		}
		else
		{
			NewPoints[i].KF_initial();
			NewPoints[i].velcity = Speed;
		}
		NewPoints[i].velcity = NewPoints[i].velcity*0.036;// cm/s Convert to km/hr 
	}

}