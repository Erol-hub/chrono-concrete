// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Bahar Ayhan
// =============================================================================
//
//  Contact force model for discrete fresh concrete
//  Details about the model can be found from the paper of "Ramyar, Elham, and Gianluca Cusatis"
//  Ramyar, Elham, and Gianluca Cusatis. "Discrete Fresh Concrete Model for Simulation of Ordinary, 
//                                        Self-Consolidating, and Printable Concrete Flow." 
//											Journal of Engineering Mechanics 148.2 (2022): 04021142.
//
//			    Material Parameters
//		 	  	float ENm		: mortar to mortar and mortar to aggregate stiffness
//				float ENa		: aggregate to aggregate stiffness
//				float h			: thickness off mortar layer around an aggregate 		
//				float alpha		: parameter for compressive contact  
//				float beta		: parameter for tensile contact
//				float np 		: np=1 newtonian fluid, np<1 shear-thinning and np>1 shear-thickening 
//				float sgmTmax 	: tensile strength of mortar
//				float sgmTau0 	: shear yield stress
//				float kappa0  	: is a constant 
//				float eta_inf 	: mortar plastic viscosity
// =============================================================================

#include "ChSystemSMC.h"
#include "ChContactSMC.h"
#include "ChContactContainerSMC.h"

#include "chrono/collision/ChCollisionSystem.h"
#include "chrono/collision/ChCollisionSystemBullet.h" 
 

#include "chrono_thirdparty/rapidjson/prettywriter.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_thirdparty/filesystem/path.h"



using namespace chrono;




class MyStateVar {
    public:
    
    MyStateVar() {};
    MyStateVar(double mradA, double mradB, ChVector<> mstrain)
    : radA(mradA), radB(mradB), strain(mstrain)
    {};
    virtual ~MyStateVar() {}
    
    public:
	double step_time=0;
    float radA=1;
    float radB=1;
    ChVector<> strain={0,0,0};
    
};

std::map<std::string, MyStateVar> map_contact_info;




template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

// -----------------------------------------------------------------------------
// Class for overriding the default SMC contact force calculation
// -----------------------------------------------------------------------------
class MyContactForce : public ChSystemSMC::ChContactForceSMC {
  public:
    // Demonstration only.	
	
    virtual ChVector<> CalculateForce(
        const ChSystemSMC& sys,             ///< containing sys
        const ChVector<>& normal_dir,       ///< normal contact direction (expressed in global frame)
        const ChVector<>& p1,               ///< most penetrated point on obj1 (expressed in global frame)
        const ChVector<>& p2,               ///< most penetrated point on obj2 (expressed in global frame)
        const ChVector<>& vel1,             ///< velocity of contact point on obj1 (expressed in global frame)
        const ChVector<>& vel2,             ///< velocity of contact point on obj2 (expressed in global frame)
        const ChMaterialCompositeSMC& mat,  ///< composite material for contact pair
        double delta,                       ///< overlap in normal direction
        double eff_radius,                  ///< effective radius of curvature at contact
        double mass1,                       ///< mass of obj1
        double mass2,                        ///< mass of obj2
		ChContactable* objA,
		ChContactable* objB		
    )  const override {
		
		float mortar_layer=3;
		float ENm=4.0E-2;
		float ENa=10;
		float h=mortar_layer;		
		float alpha=0.25;
		float beta=0.5;
		float np=1.0;
		float sgmTmax=9.E-3;
		float sgmTau0=250E-6;
		float kappa0=100.;
		float eta_inf=10.E-6;
		float R1=5;
		float R2=5; 
		
		
		//
    	//Get state variables form the map defined globally
    	//    	
		auto bodyA=dynamic_cast<ChBody*>(objA);
		auto bodyB=dynamic_cast<ChBody*>(objB);
		
		
		std::vector<unsigned int> ID={bodyA->GetId(),bodyB->GetId()};
		std::string mykey;
		
		if (ID[0]>ID[1]){			
		       mykey=std::to_string(ID[0])+"_"+std::to_string(ID[1]);		      
		}else{
		       mykey=std::to_string(ID[1])+"_"+std::to_string(ID[0]);		       
		}
		//
    	// Get state variables relevant to this contact	
		//
    	ChVector<> statevar=map_contact_info[mykey].strain;
    		
		//
		// initialize force values
		//
		ChVector<> force=(0,0,0);		
		//
		// If delta>h, objects are saparated
		//	
		if (delta<0)
			return force;
		//
		// Get the dimension of the object
		//
		if (bodyA->GetCollisionModel()->GetShape(0)->GetType()==0){        
			R1=bodyA->GetCollisionModel()->GetShapeDimensions(0)[0];
			
		}
		if (bodyB->GetCollisionModel()->GetShape(0)->GetType()==0){        
			R2=bodyB->GetCollisionModel()->GetShapeDimensions(0)[0];			
		}
		
		if (bodyA->GetCollisionModel()->GetShape(0)->GetType()!=0 || bodyB->GetCollisionModel()->GetShape(0)->GetType()!=0){ 
			h=mortar_layer/2;
		}
		//
		// Calculate contact area
		//		
		
		// center to center distance between 2 objects		
		double Lij=R1+R2-delta;		
		double ai=(R1*R1-R2*R2+Lij*Lij)/Lij/2;
		double Rmin=std::min(R1, R2);
		double radius2=Rmin*Rmin-ai*ai;
		double contact_area=CH_C_PI*radius2;		
		
		//
		// modify penetration according to initial overlap
		//
		double delta_new=delta-h;
		//
		// Change material stiffness if aggregate to aggregate contact occurs
		//
		double E_eff=ENm;
		if (delta_new>=h){
			E_eff=ENa;
			double r=Rmin-h;
			contact_area=CH_C_PI*(r*r-(r-delta_new)*(r-delta_new));
		}
		
		if (contact_area<0)
			contact_area=CH_C_PI*eff_radius*eff_radius;
		//
		//	
        // Relative velocities at contact	
		//
		//				
        ChVector<> relvel = vel2 - vel1;
        double relvel_n_mag = relvel.Dot(normal_dir);
        ChVector<> relvel_n = relvel_n_mag * normal_dir;
        ChVector<> relvel_t = relvel - relvel_n;
        double relvel_t_mag = relvel_t.Length();		
		//
		// Calculate displacement increment in normal and tangential direction
		//
		double dT = sys.GetStep();
		double delta_t = relvel_t_mag * dT;
		double delta_n = relvel_n_mag * dT;			
        ChVector<> v_delta_t = relvel_t * dT;
		//
		// Calculate damping factor ( it is taken from Hooke type contact from multicore)
		//
		double eps = std::numeric_limits<double>::epsilon();
		double cr_eff=mat.cr_eff;
		double tmp_k = (16.0 / 15) * sqrt(eff_radius * E_eff);
        double char_vel = 1;
        double v2 = char_vel * char_vel;
        double loge = (cr_eff < eps) ? log(eps) : log(cr_eff);
        loge = (cr_eff > 1 - eps) ? log(1 - eps) : loge;
        double tmp_g = 1 + pow(CH_C_PI / loge, 2);
		double m_eff=mass1*mass2/(mass1+mass2);
        double kn = E_eff*contact_area; //tmp_k * pow(m_eff * v2 / tmp_k, 1.0 / 5);
        //double kt = kn;
        double gn = sqrt(4 * m_eff * kn / tmp_g);
        double gt = gn;
		//
		//
		//
		if (delta_new>0){
			//
			// Compressive contact;
			//
			
			// Calculate the strain and stress increment
			double depsN=delta_n/Lij;
			double depsT=delta_t/Lij;
			//double depsN=relvel_n_mag/Lij;
			//double depsT=relvel_t_mag/Lij;
			double dsgmN=0; double dsgmT=0;			
			if (delta_new<=h){
				dsgmN=E_eff*depsN;	
				dsgmT=0;			
			}else{
				dsgmN=E_eff*depsN;	
				dsgmT=E_eff*alpha*depsT;
			}			
			//
			//
			double sgmN=statevar[0]+dsgmN;
			double sgmT=statevar[1]+dsgmT;
			sgmT = std::min<double>(sgmT, mat.mu_eff * std::abs(sgmN));
			
			//
		    statevar[0]=sgmN; statevar[1]=sgmT;			
			map_contact_info[mykey].strain=statevar;
			map_contact_info[mykey].step_time=sys.GetChTime();
			//
			//	
			//
			double forceN=contact_area*sgmN-sgn(sgmN)*gn*relvel_n_mag;	/// first term is negative, second term should reduce absolute value so it is added.
			double forceT=contact_area*sgmT;
			//if (sgmT!=0) 
				forceT -= sgn(sgmT)*gt*relvel_t_mag;			
			//
			// Accumulate normal and tangential forces
			//
			force = forceN * normal_dir;		
			if (relvel_t_mag >= sys.GetSlipVelocityThreshold())
				force -= (forceT / relvel_t_mag) * relvel_t;			 
            			
			return -force;
			
		
		} else{
			//
			// Tensile contact
			//
			//Material parameters of viscous part which should be defined inside contactinfo
			double eta0=kappa0*eta_inf;
			double Deps0=sgmTau0/eta0;
			//std::cout<<"tension conatact condition"<<std::endl;
			//////////////////////////////////////////////////////////
			//
			// Calculate strain increments
			//
			//////////////////////////////////////////////////////////
			//double depsN=relvel_n_mag/Lij;			
			//double depsT=relvel_t_mag/Lij;
			double depsN=delta_n/Lij;
			double depsT=delta_t/Lij;
			double Deps=pow((beta*depsN*depsN+depsT*depsT),0.5);			
			//////////////////////////////////////////////////////////
			//
			// Calculate Stress from material stiffness
			//
			//////////////////////////////////////////////////////////
			double dsgmN_stiff=ENm*depsN;
			//
			// update state variables
			//
			double sgmN_stiff=statevar[2]+dsgmN_stiff;			
			sgmN_stiff=std::min<double>(sgmN_stiff, sgmTmax );			
			statevar[2]=sgmN_stiff;			
			map_contact_info[mykey].strain=statevar;
			map_contact_info[mykey].step_time=sys.GetChTime();
			//
			//////////////////////////////////////////////////////////
			//
			// Viscous stresses
			//		
			//////////////////////////////////////////////////////////
			//
			depsN=relvel_n_mag/Lij;
			depsT=relvel_t_mag/Lij;
			// calculate strain rates
			double vdepsN=depsN;
			double vdepsT=depsT;
			double vDeps=pow((beta*vdepsN*vdepsN+vdepsT*vdepsT),0.5);
			//
			double eta;
			if(Deps<=Deps0) {
				eta=eta0;
			}else{
				eta=eta_inf*pow(abs(vDeps),np-1.);
			}			
			
			double sgmN=beta*eta*vdepsN;
			double sgmT=eta*vdepsT;
			
			//////////////////////////////////////////////////////////
			//
			// Combine Viscous stresses and stiffness stresses and calculate forces
			//		
			//////////////////////////////////////////////////////////				
			//exit(0);
			double forceN = (sgmN+sgmN_stiff) * contact_area ;
			double forceT = sgmT * contact_area;
			//if( sgmN/(sgmN_stiff+1e-12)<0){
            //std::cout<<"delta_n  "<<delta_n<<" sgmN  "<<sgmN<<	" sgmN_stiff "<<sgmN_stiff<< "gn*relvel_n_mag" << gn*relvel_n_mag <<std::endl;
			//}
			//std::cout<<" gn: "<< gn << " relvel_n_mag "<<relvel_n_mag<<"\t";
			//std::cout<<"  sgmT * contact_area "<<sgmT * contact_area<<	" gt*relvel_t_mag "<<gt*relvel_t_mag<<std::endl;
			force = forceN * normal_dir;
			if (relvel_t_mag >= sys.GetSlipVelocityThreshold())
				force += (forceT / relvel_t_mag) * relvel_t;			
			
			return -force;
		}
        
    }
    		
	//collision::ChCollisionInfo *cinfo;
	
};


class MyContactContainer : public ChContactContainerSMC {
  public:
    MyContactContainer() {}
    ~MyContactContainer() {}
    // Traverse the list contactlist_6_6
    
    
    void IterOnContactList(double& current_time) {
        auto iter = contactlist_6_6.begin();
        int num_contact = 0;
        while (iter != contactlist_6_6.end()) {
            ChVector<> p1 = (*iter)->GetContactP1();
            ChVector<> p2 = (*iter)->GetContactP2();
            double CD = (*iter)->GetContactDistance();
            ChBody* bodyA = dynamic_cast<ChBody*>((*iter)->GetObjA());
			ChBody* bodyB = dynamic_cast<ChBody*>((*iter)->GetObjB());
	        ///
	        ///
	        ///
            //std::cout<<"objA: "<<bodyA->GetId();
			//std::cout<<"\t objB: "<< bodyB->GetId()<<std::endl;                   
			std::vector<unsigned int> ID={bodyA->GetId(),bodyB->GetId()};
			std::string mykey;
		    ///
			///
			if (ID[0]>ID[1]){
				
				   mykey=std::to_string(ID[0])+"_"+std::to_string(ID[1]);
				   //std::cout<<mykey<<std::endl;
			}else{
				   mykey=std::to_string(ID[1])+"_"+std::to_string(ID[0]);
				   //std::cout<<mykey<<std::endl;
			}
			
			if(map_contact_info[mykey].step_time!=current_time){
				std::cout<<"mykey  "<<mykey<<"\t";
				std::cout<<"map update time\t"<<map_contact_info[mykey].step_time<<"\t current time\t"<<current_time<<std::endl;
				map_contact_info.erase(mykey);
			}
			
            ///
			///
            num_contact++;
            ++iter;
        }
        
    }    
 
    
};


// -----------------------------------------------------------------------------
// Callback class for contact reporting
// -----------------------------------------------------------------------------
class ContactReporter : public ChContactContainer::ReportContactCallback {
  public:
    ContactReporter(FILE* fptr, std::shared_ptr<ChBody> objA, std::shared_ptr<ChBody> objB) : m_objA(objA), m_objB(objB), m_fptr(fptr) {}

  private:
    virtual bool OnReportContact(const ChVector<>& pA,
                                 const ChVector<>& pB,
                                 const ChMatrix33<>& plane_coord,
                                 const double& distance,
                                 const double& eff_radius,
                                 const ChVector<>& cforce,
                                 const ChVector<>& ctorque,
                                 ChContactable* modA,
                                 ChContactable* modB) override {
		//
		//
		auto bodyA=dynamic_cast<ChBody*>(modA);
		auto bodyB=dynamic_cast<ChBody*>(modB);
		ChVector<> pcA=bodyA->GetFrame_COG_to_abs().GetPos();
		ChVector<> pcB=bodyB->GetFrame_COG_to_abs().GetPos();
		
		//fprintf(m_fptr,"center of sphere A: %10.6f  %10.6f  %10.6f", pcA.x(), pcA.y(), pcA.z());
		//fprintf(m_fptr,"  center of sphere B: %10.6f  %10.6f  %10.6f", pcB.x(), pcB.y(), pcB.z());
		fprintf(m_fptr," %10.6f  %10.6f  %10.6f ", pcA.x(), pcA.y(), pcA.z());
		fprintf(m_fptr,"  %10.6f  %10.6f  %10.6f ", pcB.x(), pcB.y(), pcB.z());
		
		ChVector<> pcA0=bodyA->GetFrame_REF_to_abs().GetPos();
		ChVector<> pcB0=bodyB->GetFrame_REF_to_abs().GetPos();
		
		ChVector<> vcA=bodyA->GetFrame_COG_to_abs().GetPos_dt();
		ChVector<> vcB=bodyB->GetFrame_COG_to_abs().GetPos_dt();
		//fprintf(m_fptr,"  vel of sphere A: %10.6f  %10.6f  %10.6f", vcA.x(), vcA.y(), vcA.z());
		//fprintf(m_fptr,"  vel of sphere B: %10.6f  %10.6f  %10.6f", vcB.x(), vcB.y(), vcB.z());
		fprintf(m_fptr,"  %10.6f  %10.6f  %10.6f ", vcA.x(), vcA.y(), vcA.z());
		fprintf(m_fptr,"  %10.6f  %10.6f  %10.6f ", vcB.x(), vcB.y(), vcB.z());
		
		ChVector<> wcA=bodyA->GetFrame_COG_to_abs().GetWvel_loc();
		ChVector<> wcB=bodyB->GetFrame_COG_to_abs().GetWvel_loc();		
		//fprintf(m_fptr,"  angular vel of sphere A: %10.6f  %10.6f  %10.6f", wcA.x(), wcA.y(), wcA.z());
		//fprintf(m_fptr,"  angular vel of sphere B: %10.6f  %10.6f  %10.6f", wcB.x(), wcB.y(), wcB.z());
		fprintf(m_fptr,"  %10.6f  %10.6f  %10.6f ", wcA.x(), wcA.y(), wcA.z());
		fprintf(m_fptr,"  %10.6f  %10.6f  %10.6f ", wcB.x(), wcB.y(), wcB.z());
		//
		//
		
        // Check if contact involves objA
        if (modA == m_objA.get()) {
            //fprintf(m_fptr,"  A contact on sphere 1 at pos: %10.6f  %10.6f  %10.6f", pA.x(), pA.y(), pA.z());
			fprintf(m_fptr,"  %10.6f  %10.6f  %10.6f ", pA.x(), pA.y(), pA.z());
        } else if (modB == m_objA.get()) {
            //fprintf(m_fptr,"  B contact on sphere 1 at pos: %10.6f  %10.6f  %10.6f", pB.x(), pB.y(), pB.z());
			fprintf(m_fptr,"  %10.6f  %10.6f  %10.6f ", pB.x(), pB.y(), pB.z());
        }

        // Check if contact involves objB
        if (modA == m_objB.get()) {
            //fprintf(m_fptr,"  A contact on sphere 2 at pos: %10.6f  %10.6f  %10.6f", pA.x(), pA.y(), pA.z());
			fprintf(m_fptr,"  %10.6f  %10.6f  %10.6f ", pA.x(), pA.y(), pA.z());
        } else if (modB == m_objB.get()) {
            //fprintf(m_fptr,"  B contact on sphere 2 at pos: %10.6f  %10.6f  %10.6f", pB.x(), pB.y(), pB.z());
			fprintf(m_fptr,"  %10.6f  %10.6f  %10.6f ", pB.x(), pB.y(), pB.z());
        }
				
		
        const ChVector<>& nrm = plane_coord.Get_A_Xaxis();
        //fprintf(m_fptr,"  nrm: %10.6f, %10.6f  %10.6f", nrm.x(), nrm.y(), nrm.z());
        //fprintf(m_fptr,"  frc: %12.8e  %12.8e  %12.8e", cforce.x(), cforce.y(), cforce.z());		
        ////printf("  trq: %7.3f, %7.3f  %7.3f", ctorque.x(), ctorque.y(), ctorque.z());
        //fprintf(m_fptr,"  penetration: %12.8e   eff. radius: %7.3f\n", distance, eff_radius);
		
		fprintf(m_fptr,"  %10.6f  %10.6f  %10.6f ", nrm.x(), nrm.y(), nrm.z());
        fprintf(m_fptr,"  %12.8e  %12.8e  %12.8e ", cforce.x(), cforce.y(), cforce.z());        
        fprintf(m_fptr,"  %12.8e   %7.3f\n", distance, eff_radius);

        return true;
    }

    std::shared_ptr<ChBody> m_objA;
    std::shared_ptr<ChBody> m_objB;
	FILE *m_fptr;
};



 // Function to write particle positions and radii to a VTK file
void WriteParticlesVTK(ChSystemSMC& sys, const std::string& filename) {
     // Get the number of particles
     auto body_list= sys.Get_bodylist();
     int num_particles = body_list.size();

     // Create the VTK file and write the header
     std::ofstream vtk_file(filename);
     vtk_file << "# vtk DataFile Version 3.0\n";
     vtk_file << "vtk output\n";
     vtk_file << "ASCII\n";
     vtk_file << "DATASET UNSTRUCTURED_GRID\n";

     // Write the particle positions
     vtk_file << "POINTS " << num_particles << " float\n";
     for (int i = 0; i < num_particles; i++) {
         ChVector<float> pos = body_list[i]->GetPos();
         vtk_file << pos.x() << " " << pos.y() << " " << pos.z() << "\n";
     }

     // Write the particle IDs
     vtk_file << "\nCELLS " << num_particles << " " << num_particles * 2 << "\n";
     for (int i = 0; i < num_particles; i++) {
         vtk_file << "1 " << i << "\n";
     }

     // Write the cell types
     vtk_file << "\nCELL_TYPES " << num_particles << "\n";
     for (int i = 0; i < num_particles; i++) {
         vtk_file << "1\n";
     }

     // Write the particle radii
     vtk_file << "\nPOINT_DATA " << num_particles << "\n";
     vtk_file << "SCALARS radius float 1\n";
     vtk_file << "LOOKUP_TABLE default\n";
     for (int i = 0; i < num_particles; i++) {
         vtk_file << 5 << "\n";
     }

     // Write the particle velocities
     vtk_file << "\nVECTORS velocity float\n";
     for (int i = 0; i < num_particles; i++) {
         ChVector<float> vel = body_list[i]->GetPos_dt();;
         vtk_file << vel.x() << " " << vel.y() << " " << vel.z() << "\n";
     }

     // Close the file
     vtk_file.close();
 }


