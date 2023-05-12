#ifndef WORLD_H
#define WORLD_H

#include "eigenIncludes.h"

// include elastic rod class
#include "elasticRod.h"

// include force classes
#include "elasticStretchingForce.h"
#include "elasticBendingForce.h"
#include "elasticTwistingForce.h"
#include "externalGravityForce.h"
#include "inertialForce.h"

// include external force
#include "dampingForce.h"

// include time stepper
#include "timeStepper.h"

// include input file and option
#include "setInput.h"


class world
{
public:
	world();
	world(setInput &m_inputData);
	~world();
	void setRodStepper();
	void updateTimeStep();
	int simulationRunning();
	int numPoints();
	double getScaledCoordinate(int i);
	double getCurrentTime();
	double getTotalTime();

	bool isRender();

	// file output
	void OpenFile(ofstream &outfile);
	void OpenFile1(ofstream &outfile);
	void CloseFile(ofstream &outfile);
	void CoutData(ofstream &outfile);

	void updateTimeStep_data();

	void CoutDataC(ofstream &outfile);

	Vector2d energy;
	MatrixXd vertices;

	MatrixXd vertices0;
	Vector3d getMaterialFrame(int i, int idx);
	MatrixXd frames;

	double kap;


private:

	// Physical parameters
	double RodLength;
	double helixradius, helixpitch;
	double rodRadius;
	int numVertices;
	double youngM;
	double Poisson;
	double shearM;
	double deltaTime;
	double totalTime;
	double density;
	Vector3d gVector;
	double viscosity;
	double kapB;


	// double EI;
	// double GJ;

	// double youngM;

	double v_constant;




	// Viscous drag coefficients
	double tol, stol;
	int maxIter; // maximum number of iterations
	double characteristicForce;
	double forceTol;

	// Geometr

	VectorXd theta;

	// Rod
	elasticRod *rod;

	// set up the time stepper
	timeStepper *stepper;
	double *totalForce;
	double currentTime;

	// declare the forces
	elasticStretchingForce *m_stretchForce;
	elasticBendingForce *m_bendingForce;
	elasticTwistingForce *m_twistingForce;
	inertialForce *m_inertialForce;
	externalGravityForce *m_gravityForce;
	dampingForce *m_dampingForce;


	int Nstep;
	int timeStep;
	int iter;

	void rodGeometry();
	void rodBoundaryCondition();

	void updateBoundary();

	void updateCons();

	void newtonMethod(bool &solved);

	void newtonMethodC(bool &solved);
	void calculateForce();

	bool render; // should the OpenGL rendering be included?
	bool saveData; // should data be written to a file?

	bool hold;


	vector<int> hold_for_tying;


	vector<int> contactZ;

	Vector3d temp;
	Vector3d temp1;
	Vector3d gravity;
	Vector3d inertial;
	Vector3d dampingF;

	int iter_c;

	double twist;


	double twist_time;

	double Error;

	void computeError();

	MatrixXd helixvertices;

	void preDefinedHelix();

	Matrix3d Rotation;
	Vector3d Trans;

	int step;

	double desiredTheta;

	double Error_c;

	double c1;
	double c2;
	double rate;


	double tau;
	double omega;

	double kap0;
	double tau0;
	double omega0;

	bool whenInstable;
	int reverseBoundary;

	bool move2goal(Vector3d goal0, Vector3d goal, double endT, int &step);

	double instableTime;
	int searchStep;

	double Error_c0;

	double threshold;
	double thresholdr;

	double dt;

	double recordData;

	int experiment;
	Vector3d m10;
	Vector3d m20;

};

#endif
