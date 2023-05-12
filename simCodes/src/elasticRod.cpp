#include "elasticRod.h"

elasticRod::elasticRod(MatrixXd initialNodes, MatrixXd undeformed,
	double m_rho, double m_rodRadius, double m_dt,
	double m_youngM, double m_shearM, double m_rodLength, VectorXd m_theta)
{
	rodLength = m_rodLength;
	nodes = initialNodes;
	nodesUndeformed = undeformed;
	nv = nodes.rows();
	ne = nv - 1;
	ndof = 3*nv + ne;
	dt = m_dt;
	youngM = m_youngM;
	shearM = m_shearM;
	rho = m_rho;
	rodRadius = m_rodRadius;
	theta = m_theta;

    isConstrained = new int[ndof];
    for (int i=0; i < ndof; i++)
		isConstrained[i] = 0;

	x = VectorXd(ndof);
	for (int i=0; i < nv; i++)
	{
		x(4*i) = nodes(i, 0);
		x(4*i + 1) = nodes(i, 1);
		x(4*i + 2) = nodes(i, 2);
		if (i < nv - 1)
		{
			x(4*i + 3) = theta(i);
		}
	}
	x0 = x;

	u = VectorXd::Zero(ndof);
}

void elasticRod::setup()
{
	// compute the number of constrained and unconstrained dof
	ncons = 0;
    for (int i=0; i < ndof; i++)
    {
		if (isConstrained[i] > 0)
		{
			ncons ++;
		}
	}
	uncons = ndof - ncons;

	// Setup the map from free dofs to all dof
	unconstrainedMap = new int[uncons]; // maps xUncons to x
	fullToUnconsMap = new int[ndof];
	setupMap();

	crossSectionalArea = M_PI * rodRadius * rodRadius;

	d1 = MatrixXd::Zero(ne, 3);
	d2 = MatrixXd::Zero(ne, 3);
	d1_old = MatrixXd::Zero(ne, 3);
	d2_old = MatrixXd::Zero(ne, 3);
	m1 = MatrixXd::Zero(ne, 3);
	m2 = MatrixXd::Zero(ne, 3);
	refTwist = VectorXd(ne);

	// compute reference and voronoi lengths
	setReferenceLength();
	// set mass array
	setMass();
	// set tangent
	tangent = MatrixXd::Zero(ne, 3);
	computeTangent(x, tangent);
	// set reference directors
	computeSpaceParallel();
	// set material directors
	computeMaterialDirector();
	m1_b = m1;
	m2_b = m2;
	t_b = tangent;
	// compute natural curvature
	kb = MatrixXd::Zero(nv, 3);
	kappa = MatrixXd::Zero(nv, 2);
	computeKappa();
	kappaBar = MatrixXd::Zero(nv, 2);
	// compute undeformed twist
	undeformedTwist = VectorXd::Zero(ne);
	// calundeformedTwist();

	// Reference twist
	refTwist_old = VectorXd::Zero(ne);
	getRefTwist();
	// compute edge length
	edgeLen = VectorXd(ne);
	computeEdgeLen();
	// compute elastic stiffness
	computeElasticStiffness();

	// values at the beginning of time step
	x0 = x;
	d1_old = d1;
	d2_old = d2;
	tangent_old = tangent;
	refTwist_old = refTwist;

	TrueTwist = VectorXd::Zero(ne);

	return;
}

void elasticRod::calundeformedTwist()
{
	Vector3d m1_c = Vector3d(0, 1, 0);
	Vector3d t1 = Vector3d(1, 0, 0);
	// cout<<m1_c.norm()<<endl;
	Vector3d temp_m1, temp_m1_0;
	temp_m1_0 = m1_c;
	for (int i = 1; i< ne; i++)
	{
		Vector3d t2 = tangent.row(i);


		parallelTansport(m1_c, t2, t1, temp_m1);

		undeformedTwist(i) = -signedAngle(temp_m1_0, temp_m1, t1);

		temp_m1_0 = temp_m1;
	}
}


void elasticRod::updateMap()
{
	ncons = 0;
    for (int i=0; i < ndof; i++)
    {
		if (isConstrained[i] > 0)
		{
			ncons ++;
		}
	}
	uncons = ndof - ncons;

	delete [] unconstrainedMap;
	delete [] fullToUnconsMap;
	// Setup the map from free dofs to all dof
	unconstrainedMap = new int[uncons]; // maps xUncons to x
	fullToUnconsMap = new int[ndof];
	setupMap();
}

void elasticRod::freeVertexBoundaryCondition(int k)
{
	isConstrained[4*k] = 0;
	isConstrained[4*k + 1] = 0;
	isConstrained[4*k + 2] = 0;

}

// functions to handle boundary condition
void elasticRod::setVertexBoundaryCondition(Vector3d position, int k)
{
	isConstrained[4*k] = 1;
	isConstrained[4*k + 1] = 1;
	isConstrained[4*k + 2] = 1;
	// Store in the constrained dof vector
	x(4*k) = position(0);
	x(4*k + 1) = position(1);
	x(4*k + 2) = position(2);
}

void elasticRod::setVelocity(int k)
{
	for (int i = 0; i< 4; i++)
	{
		u(4*k+i) = 0;
	}
}


void elasticRod::setThetaBoundaryCondition(double desiredTheta, int k)
{
	isConstrained[4*k + 3] = 1;
	x(4*k + 3) = desiredTheta;
}

elasticRod::~elasticRod()
{
	delete isConstrained;
	delete unconstrainedMap;
	delete fullToUnconsMap;
}

int elasticRod::getIfConstrained(int k)
{
	return isConstrained[k];
}

void elasticRod::setMass()
{
	massArray = VectorXd::Zero(ndof);

	for (int i=0; i<nv; i++)
	{
		dm = rodLength * crossSectionalArea * rho / ne;

		if (i < nv-1)
		{
			massArray[4*i+3] = rodRadius * rodRadius * dm / 2.0;
		}
		if (i==0 || i==nv-1) dm = dm / 2.0;

		for (int k=0; k <3; k++)
		{
			massArray[4*i+k] = dm;
		}
	}
}

void elasticRod::setReferenceLength()
{
	refLen = VectorXd(ne);
	Vector3d dx;
	double delta_l = rodLength/ne;

	for (int i=0;i<ne;i++)
	{
		dx(0)=nodesUndeformed(i+1,0)-nodesUndeformed(i,0);
		dx(1)=nodesUndeformed(i+1,1)-nodesUndeformed(i,1);
		dx(2)=nodesUndeformed(i+1,2)-nodesUndeformed(i,2);
		refLen(i)=dx.norm();
		// refLen(i) = delta_l;
	}

	voronoiLen = VectorXd(nv);
	for (int i=0;i<nv;i++)
	{
		if (i==0)
			voronoiLen(i)=0.5*refLen(i);
		else if (i==nv-1)
			voronoiLen(i)=0.5*refLen(i-1);
		else
			voronoiLen(i)=0.5*(refLen(i-1)+refLen(i));
	}
}


Vector3d elasticRod::getVertex(int k)
{
	return Vector3d(x(4*k), x(4*k+1), x(4*k+2));
}

Vector3d elasticRod::getVelocity(int k)
{
	return (Vector3d(x(4*k), x(4*k+1), x(4*k+2)) -
			Vector3d(x0(4*k), x0(4*k+1), x0(4*k+2))) / dt;
}

double elasticRod::gettwist(int k)
{
	return ((x(4*k+3) - x0(4*k+3))/dt);
}

Vector3d elasticRod::getTangent(int k)
{
	return tangent.row(k);
}

double elasticRod::getTheta(int k)
{
	return x(4*k + 3);
}


double elasticRod::getTheta0(int k)
{
	return x0(4*k + 3);
}

void elasticRod::computeTimeParallel()
{
	// Use old versions of (d1, d2, tangent) to get new d1, d2
    Vector3d t0,t1, d1_vector;

    for (int i=0;i<ne;i++)
    {
		t0 = tangent_old.row(i);
		t1 = tangent.row(i);
        parallelTansport(d1_old.row(i), t0, t1, d1_vector);

        d1.row(i) = d1_vector;
        d2.row(i) = t1.cross(d1_vector);
    }
}

void elasticRod::computeTangent(const VectorXd &xLocal, MatrixXd &tangentLocal)
{
    for (int i=0; i<ne; i++)
    {
        tangentLocal.row(i) = xLocal.segment(4*(i+1),3) - xLocal.segment(4*i,3);
        tangentLocal.row(i) = tangentLocal.row(i)/(tangentLocal.row(i)).norm();
    }
}

void elasticRod::parallelTansport(const Vector3d &d1_1,const Vector3d &t1, const Vector3d &t2, Vector3d &d1_2)
{
	Vector3d b;
	Vector3d n1,n2;

	b=t1.cross(t2);

	if(b.norm()==0)
		d1_2=d1_1;
	else
	{
		b = b / b.norm();
		b = b - b.dot(t1) * t1;
		b = b / b.norm();
		b = b - b.dot(t1) * t2;
		b = b / b.norm();

		n1=t1.cross(b);
		n2=t2.cross(b);
		d1_2=d1_1.dot(t1)*t2+d1_1.dot(n1)*n2+d1_1.dot(b)*b;
		d1_2=d1_2-d1_2.dot(t2)*t2;
		d1_2=d1_2/d1_2.norm();
	}
}

void elasticRod::computeSpaceParallel()
{
	// This function is only called once
	Vector3d t0, t1, d1Tmp;
	Vector3d a, b, c, d;

	t0 = tangent.row(0);
	t1 << 0,0,-1;
	d1Tmp = t0.cross(t1);

	if (fabs(d1Tmp.norm()) < 1.0e-6)
	{
		t1 << 0,-1,0;
		d1Tmp=t0.cross(t1);
	}


	d1.row(0)=d1Tmp;
	d2.row(0)=t0.cross(d1Tmp);

	// cout<<t0<<endl;
	// cout<<d1.row(0)<<endl;
	// cout<<d2.row(0)<<endl;
	// exit(0);

	for (int i=0;i<ne-1;i++)
	{
		a=d1.row(i);
		b=tangent.row(i);
		c=tangent.row(i+1);
		parallelTansport(a, b, c, d);
		d1.row(i+1)=d;
	    d2.row(i+1)=c.cross( d );
	}
}

void elasticRod::computeMaterialDirector()
{
	// TODO: take out cs, ss out of the function and declare them as privates in elasticRod class
    double cs,ss;
    double angle;

    for (int i=0;i<ne;i++)
    {
		angle = x(4*i+3);
        cs=cos(angle);
        ss=sin(angle);
        m1.row(i) = cs*d1.row(i) + ss*d2.row(i);
        m2.row(i) =-ss*d1.row(i) + cs*d2.row(i);
    }

    // cout<<m1.row(0)<<endl;
    // cout<<m2.row(0)<<endl;
    // exit(0);
}

void elasticRod::computeKappa()
{
	// We know the tangent, m1, m2. Compute kappa using them
	Vector3d t0, t1;
	Vector3d m1e,m2e,m1f,m2f;

    for(int i=1; i<ne; i++)
    {
		    t0 = tangent.row(i-1);
        t1 = tangent.row(i);
        kb.row(i) = 2.0 * t0.cross(t1) / (1.0+t0.dot(t1));
    }

    for(int i=1; i<ne; i++)
    {
		m1e=m1.row(i-1);
        m2e=m2.row(i-1);
        m1f=m1.row(i);
        m2f=m2.row(i);
        kappa(i, 0)= 0.5 * (kb.row(i)).dot(m2e+m2f);
        kappa(i, 1)=-0.5 * (kb.row(i)).dot(m1e+m1f);
    }
}

void elasticRod::getRefTwist()
{
    Vector3d u0,u1,t0,t1,ut;
    double sgnAngle;

    // for (int i=1;i<ne;i++)
    // {
    //     u0=d1.row(i-1);
    //     u1=d1.row(i);
    //     t0=tangent.row(i-1);
    //     t1=tangent.row(i);

    //     parallelTansport(u0,t0,t1,ut);
    //     // rotateAxisAngle(ut,t1,refTwist_old(i));

    //     // sgnAngle = signedAngle(ut,u1,t1);
    //     sgnAngle = signedAngle(ut,u1,t1);
    //     refTwist(i) = sgnAngle;
    //     // refTwist(i) = refTwist_old(i) + sgnAngle;
    // }

    for (int i=1;i<ne;i++)
    {
        u0=d1.row(i-1);
        u1=d1.row(i);
        t0=tangent.row(i-1);
        t1=tangent.row(i);

        parallelTansport(u0,t0,t1,ut);
        rotateAxisAngle(ut,t1,refTwist_old(i));

        sgnAngle = signedAngle(ut,u1,t1);
        refTwist(i) = refTwist_old(i) + sgnAngle;
    }
}

double elasticRod::getTrueAngle(int n)
{
	double sgnAngle;

	sgnAngle = TrueTwist(n);

	return sgnAngle;

}

void elasticRod::updateTrueAngle()
{
	Vector3d t_0, t_f, m1_mid, m1_0, m1_new;
	for (int i = 0; i< ne; i++)
	{
		t_0 = t_b.row(i);
		t_f = tangent.row(i);
        m1_0 = m2_b.row(i);
		parallelTansport(m1_0, t_0, t_f, m1_mid);

	    m1_mid = m1_mid - m1_mid.dot(t_f)*t_f;
	    m1_mid = m1_mid/m1_mid.norm();

        double near_angle;
	    if (i==0)
	    {
	    	near_angle = TrueTwist(i);
	    }
	    else
	    {
	    	near_angle = TrueTwist(i-1);
	    }

	    rotateAxisAngle(m1_mid,t_f,near_angle);

	    m1_new = m2.row(i);

	    double delta_angle;
	    delta_angle = signedAngle(m1_mid, m1_new,t_f);
	    TrueTwist(i) = delta_angle + near_angle;
	}

    int_twist = VectorXd::Zero(ne);
	for (int i = 1; i< ne; i++)
	{
		int_twist(i) = x(4*i+3)-x(4*(i-1)+3) + refTwist(i) - undeformedTwist(i);
	}
}

void elasticRod::updateBoundary()
{

  double temp = 0;
	for (int i = 1; i<ne;i++)
	{
		temp = temp + refTwist(i) + x(4*i+3) - x(4*i-1);
	}

	x(4*(ne-1)+3) = x(4*(ne-1)+3) - temp;


}

void elasticRod::computeEdgeLen()
{
    for (int i=0; i<ne; i++)
    {
        edgeLen[i] = (x.segment(4*(i+1),3) - x.segment(4*i,3)).norm();
    }
}

double elasticRod::signedAngle(const Vector3d &u, const Vector3d &v, const Vector3d &n)
{
	//Compute the angle between two vectors
	Vector3d w=u.cross(v);
	double angle=atan2(w.norm(),u.dot(v));
	if (n.dot(w)<0)
		return -angle;
	else
		return angle;
}


void elasticRod::rotateAxisAngle(Vector3d &v,const Vector3d &z,const double &theta)
{
	//Compute the vector when it rotates along another vector into certain angle
	if (theta!=0) // if theta=0, v = v
	{
		double cs,ss;
		cs=cos(theta);
		ss=sin(theta);
		v=cs*v+ss*z.cross(v)+z.dot(v)*(1.0-cs)*z;
	}
}

void elasticRod::setupMap()
{
	int c = 0;
	for (int i=0; i < ndof; i++)
	{
		if (isConstrained[i] == 0)
		{
			unconstrainedMap[c] = i;
			fullToUnconsMap[i] = c;
			c++;
		}
	}
}

void elasticRod::printInfo()
{
	for (int i=0; i < nv; i++)
	{
		// std::cout << "x at i=" << i << " is "
		// 	<< x(4*i) << "," << x(4*i+1) << "," << x(4*i+2) << std::endl;
		std::cout << x(4*i) << "," << x(4*i+1) << "," << x(4*i+2) <<";" <<std::endl;
	}
	// for (int i=0; i < ne; i++)
	// {
	// 	std::cout << "ElasticRodClass i=" << i << " d1=" << d1.row(i) << " d2=" << d2.row(i) << std::endl;
	// }
}

void elasticRod::computeElasticStiffness()
{
	//shearM = youngM/(2.0*(1.0+Poisson));					// shear modulus
	//c =  GJ/EA = 2/(2*(1+Poisson)) = 1/(1+Poisson);

	EI = (youngM*M_PI*rodRadius*rodRadius*rodRadius*rodRadius)/4;
	EA =  100*youngM*M_PI*rodRadius*rodRadius;
	GJ = (shearM*M_PI*rodRadius*rodRadius*rodRadius*rodRadius)/2;

}

void elasticRod::prepareForIteration()
{
    computeTangent(x, tangent);
    computeTimeParallel();
    getRefTwist();
	computeMaterialDirector();
	computeEdgeLen();
    computeKappa();
}

void elasticRod::updateNewtonX(double *dx)
{
	for (int c=0; c < uncons; c++)
	{
		x[unconstrainedMap[c]] -= dx[c];
	}
}

void elasticRod::updateTimeStep()
{
	prepareForIteration();

	// compute velocity
	u = (x - x0) / dt;

	// update x
	x0 = x;

	// update reference directors
	d1_old = d1;
	d2_old = d2;

	// We do not need to update m1, m2. They can be determined from theta.
	tangent_old = tangent;
	refTwist_old = refTwist;
}

void elasticRod::updateGuess()
{
	for (int c=0; c < uncons; c++)
	{
		x[unconstrainedMap[c]] = x0[unconstrainedMap[c]] + u[unconstrainedMap[c]] * dt;
	}
}

Vector3d elasticRod::getRelativeVelocity(int i, int j, double sc, double tc)
{
	Vector3d relativeV;

	relativeV = (1 - sc) * getVelocity(i) + getVelocity(i+1)
	            - ((1 - tc) * getVelocity(j) + getVelocity(j+1));

	return relativeV;

    // i realtive j

}


Vector2d elasticRod::calenergy()
{
	double bending_e = 0;
	double twisting_e = 0;

    double delta_l = rodLength/ne;

	for(int i=0; i<nv; i++)
    {
    	Vector2d temp = kappa.row(i) - kappaBar.row(i);
    	bending_e = bending_e + 0.5 * EI * pow(temp.norm(),2)/delta_l;
        if (i < ne-1)
        {
        	double dtwist = refTwist(i) + x(4*(i+1)+3) - x(4*i+3);
        	twisting_e = twisting_e + 0.5 * GJ * pow(dtwist, 2)/delta_l;
        }
    }

    return Vector2d(bending_e, twisting_e);
}


double elasticRod::calTwist()
{
	twist = VectorXd::Zero(ne);

	for (int i = 1; i< ne; i++)
	{
		double deltam = x(4*i+3) - x(4*i-1);

		twist(i) = deltam + refTwist(i) - undeformedTwist(i);
	}

	return twist.sum();
}



Vector3d elasticRod::calStrain()
{
	twist = VectorXd::Zero(ne);

	for (int i = 1; i< ne; i++)
	{
		double deltam = x(4*i+3) - x(4*i-1);

		twist(i) = deltam + refTwist(i) - undeformedTwist(i);
	}

	// Kappa1 and Kappa2
	double kappa1 = 0; double kappa2 = 0;
  MatrixXd kappa_vec(ne, 3);
	for (int i = 0; i < ne; i++)
	{
		kappa1 += kappa(i, 0);
		kappa2 += kappa(i, 1);
	}


	return Vector3d(kappa1, kappa2, twist.sum());
}



Matrix3d elasticRod::obtainEnd()
{
	Matrix3d frame;

	frame<< tangent(ne-1, 0), tangent(ne-1, 1), tangent(ne-1, 2),
	        d1(ne-1,0), d1(ne-1, 1), d1(ne-1, 2),
	        d2(ne-1, 0), d2(ne-1, 1), d2(ne-1, 2);
	return frame;
}


void elasticRod::restore()
{
	x = x0;
}


void elasticRod::computeKappaNorm()
{

	vector <double> kappa_v;
	kappa_v.clear();
	minKappa = 0;
	maxKappa = 0;
	for (int i = 1; i < nv-1; i++ )
	{
		Vector2d temp(kappa(i, 0), kappa(i,1));
		kappa_v.push_back(temp.norm()/voronoiLen(i));
	}

	minKappa =  *min_element(kappa_v.begin(),kappa_v.end());
	maxKappa =  *max_element(kappa_v.begin(),kappa_v.end());

}

Vector3d elasticRod::computeEnergy()
{
	//compute energy
	// 0.5 * EA * (strain - starin^2) * ek; // ne
	// 0.5 * EI * (kappa - kappa0)^2 /lk
	// 0.5 * GJ * (tau - tau0)^2 /lk

	Vector3d result(0, 0, 0);
	for (int i = 0; i < ne; i++)
	{
		double epsX = edgeLen(i)/refLen(i) - 1.0;
		result(0) += 0.5 * EA * pow((epsX),2) * refLen(i);
	}

	for (int i = 1; i < ne; i++)
	{
		double kappaL = (kappa.row(i) - kappaBar.row(i)).norm();
		result(1) += 0.5 * EI * pow((kappaL),2) / voronoiLen(i);
	}

  VectorXd theta_f, theta_e, deltam;
	theta_f.setZero(ne);
	theta_e.setZero(ne);
	deltam.setZero(ne);

	for (int i=0; i <ne; i++)
	{
			theta_f(i) = x(4*i+3);
	}

	for (int i=0; i<ne; i++)
	{
			if (i==0)
					theta_e(i)=0;
			else
					theta_e(i)=theta_f(i-1);
	}

	deltam = theta_f-theta_e;

	for(int i=1; i < ne; i++)
	{
		result(2) += 0.5 *GJ *pow((deltam(i) + refTwist (i) - undeformedTwist(i)),2) / voronoiLen(i);
	}

	return result;
}
