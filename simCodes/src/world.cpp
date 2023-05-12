#include "world.h"
#include <sstream>
#include <iomanip>

world::world()
{
	;
}

world::world(setInput &m_inputData)
{
	render = m_inputData.GetBoolOpt("render");				// boolean
	saveData = m_inputData.GetBoolOpt("saveData");			// boolean

	// Physical parameters
	RodLength = m_inputData.GetScalarOpt("RodLength");      // meter
  gVector = m_inputData.GetVecOpt("gVector");             // m/s^2
  maxIter = m_inputData.GetIntOpt("maxIter");             // maximum number of iterations
	rodRadius = m_inputData.GetScalarOpt("rodRadius");      // meter
	numVertices = m_inputData.GetIntOpt("numVertices");     // int_num
	youngM = m_inputData.GetScalarOpt("youngM");            // Bending stiffness
	Poisson = m_inputData.GetScalarOpt("Poisson");          // dimensionless
	deltaTime = m_inputData.GetScalarOpt("deltaTime");      // seconds
	tol = m_inputData.GetScalarOpt("tol");                  // small number like 10e-7
	stol = m_inputData.GetScalarOpt("stol");				// small number, e.g. 0.1%
	density = m_inputData.GetScalarOpt("density");          // kg/m^3
	viscosity = m_inputData.GetScalarOpt("viscosity");      // viscosity in Pa-s
  v_constant = m_inputData.GetScalarOpt("v_constant");
	c1 = m_inputData.GetScalarOpt("c1");
	c2 = m_inputData.GetScalarOpt("c2");

	kapB = m_inputData.GetScalarOpt("kapB");
	experiment = m_inputData.GetIntOpt("experiment");



	threshold  = 1.5;
	thresholdr  = 0.15;

	// v_constant = 0.1;


	shearM = youngM/(2.0*(1.0+Poisson));					// shear modulus

	// Viscous drag coefficients using Resistive Force Theory
	kap = 0;
	tau = 0;
	omega = 0;

	double dkap = v_constant;
	searchStep = 1;
	kap = kap0 + 1 * dkap * deltaTime;
	tau = tau0 + c1*dkap * deltaTime;
	omega = omega0 + c2* dkap* deltaTime ;

	kap0 = kap;
	tau0 = tau;
	omega0 = omega;

	frames = MatrixXd(numVertices-1, 3);

	Error_c0 = 0;

}


world::~world()
{
	;
}

bool world::isRender()
{
	return render;
}

void world::OpenFile(ofstream &outfile)
{
	if (saveData==false) return;

	int systemRet = system("mkdir datafiles"); //make the directory
	if(systemRet == -1)
	{
		cout << "Error in creating directory\n";
	}

	time_t current_time = time(0);

	// Open an input file named after the current time
	ostringstream name;
  name.precision(5);
	name << "datafiles/simData";
	name << "_c1_"<< c1;
	name << "_c2_"<< c2;
	name << "_Poisson_"<< Poisson;
	name << "_kapB_"<< kapB;
	name << "_exp_"<< experiment;
	name <<".txt";
	outfile.open(name.str().c_str());
	outfile.precision(10);
	// outfile << "# x [meter] y [meter] z [meter]\n";
}

void world::CoutDataC(ofstream &outfile)
{
	if (saveData==false)
		return;


	if (experiment == 0)
	{
			Vector3d node, node0;
			double kapM;

			node = rod->getVertex(numVertices-1);
			node0 = vertices0.row(numVertices/2 );


			outfile<< kap<<" "<< tau<<" "<<omega<<" "<< node(0)<<" "<< node(1) <<" " << node(2)<<" "<<
			node0(0)<<" "<< node0(1) <<" " << node0(2)<<" " << Error_c<<" "<<Error<<" "<<
				" "<<reverseBoundary<<endl;
	}
	else{

		double kap_resolution = kapB/1000.0;
		if (abs(kap - recordData) < kap_resolution)
		{
			return;
		}

		if (kap - recordData > 0)
		{
			recordData += kap_resolution;
		}
		else
		{
			recordData -= kap_resolution;
		}

		Vector3d node, node0;
		double kapM;
		if (numVertices%2 == 0) // even node
		{
			node0 = (vertices0.row(numVertices/2 - 1) + vertices0.row(numVertices/2))/2.0;
		}
		else{
			node0 = rod->getVertex(numVertices/2);
		}

		double scale = 0.51;
		node0 *= scale;

		node = scale * rod->getVertex(numVertices-1);

		Vector3d t = rod->tangent.row(numVertices-2);
	  Vector3d m1 = rod->m1.row(numVertices-2);
	  Vector3d m2 = rod->m2.row(numVertices-2);

	  Vector3d t0 = rod->getVertex(numVertices-1) - rod->getVertex(numVertices-2);
	  t0 = t0/t0.norm();

		if ( (t - t0).norm() > 1e-3)
		{
			cout <<t <<endl;
			cout << t0 << endl;
			exit(0);
		}

		Matrix3d Rt;
		Rt << -1, 0, 0,
	        0, -1, 0,
				  0, 0, 1;

		Matrix3d Rtheta;
		double alpha = M_PI;
	  Rtheta << cos(alpha), -sin(alpha), 0,
	            sin(alpha), cos(alpha), 0,
					  	0, 0, 1;
		Matrix3d R;
		// compute the true frame we need
	  double angle = rod->getTheta(numVertices-2);
		rod->rotateAxisAngle(m1, t, -angle);
		rod->rotateAxisAngle(m2, t, -angle);

		m10 = m1;
		m20 = m2;

		R << -m1(0), m2(0), -t(0),
	       -m1(1), m2(1), -t(1),
			   -m1(2), m2(2), -t(2);

	  R =  Rt * R;
	  R =  R * Rtheta; //Euler angle multiplication

		Vector3d t1 = R.col(2);
		Matrix3d Id3 = MatrixXd::Identity(3,3);

		node = Rt * node;
		node0 = Rt * node0;

		outfile<<kap<<" "<< tau<<" "<<omega<<" "<<node(0) << " " << node(1) <<" "<< node(2) <<" "
		<<node0(0)<<" "<<node0(1)<<" "<<node0(2)<<" "<< R(0, 0)<<" "<<R(1, 0)<<" "<<R(2, 0)<<" "
		<< R(0, 1) <<" "<<R(1, 1)<<" "<<R(2, 1)<<" "<<R(0, 2)<<" "<<R(1, 2)<<" "<<R(2, 2)<<" ";

		outfile<<Error_c<<
		" "<<rod->getTheta(numVertices-2)<<" "<<reverseBoundary<<endl;

	}
}

void world::CloseFile(ofstream &outfile)
{
	if (saveData==false)
		return;

	outfile.close();
}

void world::setRodStepper()
{
	// Set up geometry
	rodGeometry();

	// Create the rod
	rod = new elasticRod(vertices, vertices, density, rodRadius, deltaTime,
		youngM, shearM, RodLength, theta);

	// Find out the tolerance, e.g. how small is enough?
	characteristicForce = M_PI * pow(rodRadius ,4)/4.0 * youngM / pow(RodLength, 2);
	forceTol = tol * characteristicForce;

	// Set up boundary condition
	rodBoundaryCondition();

	// setup the rod so that all the relevant variables are populated
	rod->setup();
	// End of rod setup

	// set up the time stepper
	stepper = new timeStepper(*rod);
	totalForce = stepper->getForce();

	// declare the forces
	m_stretchForce = new elasticStretchingForce(*rod, *stepper);
	m_bendingForce = new elasticBendingForce(*rod, *stepper);
	m_twistingForce = new elasticTwistingForce(*rod, *stepper);
	m_inertialForce = new inertialForce(*rod, *stepper);
	m_gravityForce = new externalGravityForce(*rod, *stepper, gVector);
	m_dampingForce = new dampingForce(*rod, *stepper, viscosity);


	totalTime = 10000; // set a large value

	Nstep = totalTime/deltaTime;
	whenInstable = false;
	instableTime = 0;
	reverseBoundary = 0;

	rate = 1;


	// Allocate every thing to prepare for the first iteration
	rod->updateTimeStep();

	timeStep = 0;
	currentTime = 0.0;
	step = 1;
}

// Setup geometry
void world::rodGeometry()
{
	vertices = MatrixXd(numVertices, 3);
	vertices0 = MatrixXd(numVertices, 3);


	double delta_l = RodLength/(numVertices-1);

	for (int i = 0; i< numVertices; i++)
	{
		vertices(i, 0) = 0;
		vertices(i, 1) = 0;
		vertices(i, 2) = i*delta_l;
	}

	theta = VectorXd::Zero(numVertices - 1);

}


void world::rodBoundaryCondition()
{

	rod->setVertexBoundaryCondition(rod->getVertex(0),0);
    rod->setThetaBoundaryCondition(0,0);
    rod->setVertexBoundaryCondition(rod->getVertex(1),1);

	rod->setVertexBoundaryCondition(rod->getVertex(numVertices-1),numVertices-1);
	rod->setVertexBoundaryCondition(rod->getVertex(numVertices-2),numVertices-2);
    rod->setThetaBoundaryCondition(0,numVertices-2);
}


void world::preDefinedHelix()
{
	double a = helixradius;
  double b = helixpitch;

  double t = RodLength/sqrt(a*a+b*b);
  double dt = t/(numVertices-1);


  Vector3d P = Vector3d(0, a, 0);
  Vector3d T = Vector3d(b/sqrt(a*a+b*b), 0, a/sqrt(a*a+b*b));

  Vector3d T_base = Vector3d(0, 0, 1);

  Vector3d v = T.cross(T_base);
  double s = v.norm();
  double c = T.dot(T_base);

  Matrix3d Id3 = Matrix3d::Identity(3,3);
  if (s==0)
      Rotation = Id3;
  else
  {
      Matrix3d v_x;
      v_x<< 0, -v(2), v(1),
        	  v(2), 0, -v(0),
        	  -v(1),v(0), 0;
      Rotation = Id3  + v_x + v_x*v_x*(1-c)/pow(s,2);
  }

  Trans = -P;

	vertices0 = MatrixXd::Zero(numVertices, 3);

	for (int i = 0; i< numVertices; i++)
	{
		t = i * dt;
		Vector3d temp(b * t, a * cos(t), a * sin(t));

		temp = Rotation * temp + Trans;

		vertices0(i, 0) = temp(0);
		vertices0(i, 1) = temp(1);
		vertices0(i, 2) = temp(2);
	}

	// define frames;
	Vector3d d10(1, 0,  0);
	frames.row(0) = d10;
	Vector3d t0 = vertices0.row(1) - vertices0.row(0);
	t0 = t0/t0.norm();
	double domega = omega * (1+Poisson)/(numVertices-2);
	for (int i = 1; i< numVertices-1; i++)
	{
		Vector3d d1;
		Vector3d t1 = vertices0.row(i+1) - vertices0.row(i);
		t1 = t1/t1.norm();

		rod->parallelTansport(d10,t0,t1,d1);
		rod->rotateAxisAngle(d1,t1,domega);
		frames.row(i) = d1;
		t0 = t1;
		d10 = d1;
	}
}


void world::computeError()
{

  Error = 0;
	int middle = numVertices/2;
	// compute the prescribed middle curvature
	Vector3d t0 = vertices0.row(middle) - vertices0.row(middle-1);
	t0 = t0/t0.norm();
	Vector3d t1 = vertices0.row(middle+1) - vertices0.row(middle);
	t1 = t1/t1.norm();
	Vector3d kapV = 2.0 * t0.cross(t1) / (1.0+t0.dot(t1));
	kapV = RodLength/(numVertices-1)/kapV.norm() * kapV /kapV.norm();
	// compute the actual middle curvature
	Vector3d kapT = rod->kb.row(middle);
	kapT = RodLength/(numVertices-1)/kapT.norm() * kapT/kapT.norm();
	Vector3d tmp = vertices0.row(middle);
	tmp = tmp + kapV - rod->getVertex(middle) - kapT;
	Error = tmp.norm();

	Vector3d temp = rod->getVertex(numVertices/2);
	Vector3d temp1 =Vector3d(vertices0(numVertices/2,0),vertices0(numVertices/2,1),vertices0(numVertices/2,2));
	temp = temp - temp1;
	temp = temp;
	Error_c =  temp.norm() /helixradius;

}

void world::updateBoundary()
{
	// v_constant = 3.0/50.0 * M_PI * 0.1;
	//solve helixradius and helixpitch
	switch (reverseBoundary)
	{
		case 0:{
			rate = 1;
			break;
		}
		case 1:{
			rate = 0;
			break;
		}
		case 2:{
			rate = -1;
			break;
		}
	}
	double dkap = rate * v_constant/(sqrt(1+c1*c1+c2*c2));
	kap = kap0 +  dkap*deltaTime;
	tau = c1 * kap;
	omega = c2 * kap;
	double c = 1.0/(kap*kap + tau * tau);

	helixradius = c * kap;
	helixpitch =  c * tau ;

	preDefinedHelix();

	rod->setVertexBoundaryCondition(Vector3d(vertices0(numVertices-2, 0), vertices0(numVertices-2, 1), vertices0(numVertices-2, 2)), numVertices-2);
	rod->setVertexBoundaryCondition(Vector3d(vertices0(numVertices-1, 0), vertices0(numVertices-1, 1), vertices0(numVertices-1, 2)), numVertices-1);
	rod->setThetaBoundaryCondition(omega*(1+Poisson), numVertices-2);
}


void world::updateTimeStep()
{
	bool solved = false;
	iter_c = 0;

  while (!solved)
	{
		updateBoundary();
		rod->updateGuess();
		newtonMethod(solved);
		if (!solved)
		{
			deltaTime = 0.1 * deltaTime;
			rod->dt = deltaTime;
		}
		if (deltaTime < 1e-7)
		{
			break;
		}
	}
	dt = deltaTime;

	if (!solved) exit(0);
	rod->updateTimeStep();

	computeError();

	double kapS = kapB/2.0;

	if ( kap > kapS && reverseBoundary == 0)
	{
		if (Error_c > threshold || Error_c * helixradius > thresholdr)
		{
			reverseBoundary = 1;
			instableTime = currentTime;
		}
	}
	// cout << Error_c<<" "<< Error_c * helixradius << endl;
	if (reverseBoundary == 1 && currentTime > instableTime + 5)
	{
		reverseBoundary = 2;
	}

	if (kap <= 0 && reverseBoundary == 2)
	{
		currentTime = totalTime;
	}

	if (kap > 2 * kapB)
	{
		exit(0);
	}
	kap0 = kap;
	tau0 = tau;
	omega0 = omega;
	Error_c0 = Error_c;


	if (render) cout << "time: " << currentTime << " iter=" << iter <<" Error: "<<
	Error<<" Error_c: "<<Error_c<<" velocity: "<<rod->u.norm()<<" reverseBoundary: "<<reverseBoundary<<endl;


	currentTime += deltaTime;

	if (deltaTime < 1e-2 && iter < 4)
	{
		cout <<"changed time Step"<<endl;
		deltaTime = 10 * deltaTime;
		rod->dt = deltaTime;
	}

	timeStep++;

	if (solved == false)
	{
		currentTime = totalTime; // we are exiting
	}
}

void world::calculateForce()
{
	stepper->setZero();

	m_inertialForce->computeFi();
	m_stretchForce->computeFs();
	m_bendingForce->computeFb();
	m_twistingForce->computeFt();
	m_gravityForce->computeFg();
	m_dampingForce->computeFd();

	temp[0] =stepper->force[0]+stepper->force[4];
	temp[1] = stepper->force[1]+stepper->force[5];
	temp[2] = stepper->force[2]+stepper->force[6];


	temp1[0] = stepper->force[rod->ndof-3]+stepper->force[rod->ndof-7];
	temp1[1] = stepper->force[rod->ndof-2]+stepper->force[rod->ndof-6];
	temp1[2] = stepper->force[rod->ndof-1]+stepper->force[rod->ndof-5];

	// cout<<temp.norm()<<" "<<temp1.norm()<<" "<<inertial.norm()<<dampingF.norm()<<endl;
    // cout<< -temp - temp1 + inertial + dampingF<<endl;
}

void world::newtonMethod(bool &solved)
{
	double normf = forceTol * 10.0;
	double normf0 = 0;
	iter = 0;
	while (solved == false)
	{
		rod->prepareForIteration();
		stepper->setZero();

		// Compute the forces and the jacobians
		m_inertialForce->computeFi();
		m_inertialForce->computeJi();

		m_stretchForce->computeFs();
		m_stretchForce->computeJs();

		m_bendingForce->computeFb();
		m_bendingForce->computeJb();

		m_twistingForce->computeFt();
		m_twistingForce->computeJt();

		m_gravityForce->computeFg();
		m_gravityForce->computeJg();

		m_dampingForce->computeFd();
		m_dampingForce->computeJd();

		// stepper->monitor();

		// Compute norm of the force equations.
		normf = 0;
		for (int i=0; i < rod->uncons; i++)
		{
			normf += totalForce[i] * totalForce[i];
		}
		normf = sqrt(normf);
		// cout<<"iter: "<<iter<<"normf: "<<normf<<" "<<check->checker<<endl;
		if (iter == 0)
		{
			normf0 = normf;
		}

		if (normf <= forceTol )
		{
			solved = true;
		}
		else if(iter > 0 && normf <= forceTol)
		{
			solved = true;
		}

		if (iter > 3 && normf <= 10 * forceTol)
		{
			solved = true;
		}

		if (solved == false)
		{
			stepper->integrator(); // Solve equations of motion
			rod->updateNewtonX(totalForce); // new q = old q + Delta q
			iter++;
		}

		if (iter > maxIter)
		{
			cout << "Error. Could not converge. Exiting.\n";
			// solved = true;
			break;
		}
	}
	// cout<<"normf: "<<normf<<endl;

}

int world::simulationRunning()
{
	if (currentTime<totalTime)
		return 1;
	else
	{
		return -1;
	}
}

int world::numPoints()
{
	return rod->nv;
}

double world::getScaledCoordinate(int i)
{
	return rod->x[i] / RodLength;
}

double world::getCurrentTime()
{
	return currentTime;
}

double world::getTotalTime()
{
	return totalTime;
}


Vector3d world::getMaterialFrame(int i, int idx)
{
	if (idx == 0)
	{
		return Vector3d(rod->tangent(i, 0), rod->tangent(i, 1),rod->tangent(i, 2));
	}

	if (idx == 1)
	{
		return Vector3d(rod->m1(i, 0), rod->m1(i, 1),rod->m1(i, 2));
	}

	if (idx == 2)
	{
		return Vector3d(rod->m2(i, 0), rod->m2(i, 1),rod->m2(i, 2));
	}
	return Vector3d(0, 0, 0);
}
