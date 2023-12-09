#ifndef _DynamicObject_H_
#define _DynamicObject_H_

#include "GameObject.h"
#include <iostream>
#include <vector>

/*! \brief Brief description.
*  This physics dynamic object class is derived from the GameObject class, as a one type/class of game objects
*  It sets up parameters for physics computation and calls certain physics algorithms.
*  It returns the position and orientation of the object for the visualisation engine to display.
*  It is important to not include any graphics drawings in this class. This is the principle of the separation
*  of physics computation from graphics
*
*/

class DynamicObject : public GameObject
{
public:

	/** DynamicObject constructor
	*/
	DynamicObject();
	/** DynamicObject destructor
	*/
	~DynamicObject();

	/** Update function to override the base class function
	*   for physics computation
	*   This function is typically organized that it calls each physics computation procedures/algorithms
	*   in a sequential order, so that each computational algorithm provides required information for the
	*   next procedure.
	*   @param float deltaTs simulation time step length
	*/
	virtual void Update(float deltaTs);

	/** Add force that acts on the object to the total force for physics computation
	*  
	*   @param const glm::vec3 force 
	*/
	void AddForce(const glm::vec3 force) { _force += force; }

	void AddTorque(const glm::vec3 torque) { _torque += torque; }

	/** Clear all the forces on the object
	*
	*   @param const glm::vec3 force
	*/
	void ClearForces() { _force = glm::vec3(0.0f, 0.0f, 0.0f); }

	/** Collision detection and responses
	*
	*   
	*/
	void SetCollisionObject(std::vector<DynamicObject*> otherObjs) { _other_objects = otherObjs; };


	/** Set force for the object
	* @param glm::vec3 force a 3D vector for the force acting on the object
	*/
	void SetForce(const glm::vec3 force) { _force = force; }
	/** Set mass for the object
	* @param float mass a 3D vector for the mass of the object
	*/
	void SetMass(float mass) { _mass = mass; }
	/** Set a sphere bounding volume for the object
	* @param float r  the radius of the bounding sphere of the object
	*/
	void SetBoundingRadius(float r) { _bRadius = r; }

	/** Set position for the object
	* @param glm::vec3 pos a 3D vector for the position of the object
	*/
	void SetPosition(const glm::vec3 pos) { _position = pos; }
	/** Set velocity for the object
	* @param glm::vec3 vel a 3D vector for the velocity of the object
	*/
	void SetVelocity(const glm::vec3 vel) { _velocity = vel; }
	/** Set scale for the object
	* @param glm::vec3 vel a 3D vector for the scale of the object
	*/

	void SetAngularVelocity(const glm::vec3 vel) { _angular_velocity = vel; }

	void SetScale(const glm::vec3 scale) { _scale = scale; }

	void SetElasticity(float elast) { _elasticity = elast; }

	void SetDynamicMu(float dMu) { _dynamicMu = dMu; }

	void SetStaticMu(float sMu) { _staticMu = sMu; }

	/** Get the force acting on the object
	* @return a 3D vector
	*/
	const glm::vec3 GetForce() const { return _force; }
	/** Get the mass of the object
	* @return the result
	*/
	const float GetMass() const { return _mass; }
	/** Get the radius of the bounding sphere of the object
	* @return the result
	*/
	const float GetBoundingRadius() const { return _bRadius; }
	/** Get the position of the object
	* @return a 3D vector
	*/
	const glm::vec3 GetPosition() const { return _position; }
	/** Get the orientation of the object
	* @return a 4x4 matrix
	*/
	const glm::mat4 GetOrientation() const { return _orientation; }

	const glm::vec3 GetVelocity() const { return _velocity; }

	const float GetElasticity() const { return _elasticity; }

	const float GetDynmaicMu() const { return _dynamicMu; }

	const float GetStaticMu() const { return _staticMu; }

	const glm::vec3 GetAngularMomentum() const { return _angular_momentum; }

	const glm::vec3 GetAngularVelocity() { return _angular_velocity; }

	const glm::mat3 GetInverseInertiaTensor() const { return _inertia_tensor_inverse; }

	/** A boolean variable to control the start of the simulation This matrix is the camera's lens
	*/
	void StartSimulation(bool start);


private:

	void ComputeCollisionResponses(float deltaTs);
	/** Numerical integration function to compute the current velocity and the current position
	* based on the velocity and the position of the previous time step
	*   @param float deltaTs simulation time step length
	*/

	void AngularMotionUpdate(float deltaTs);

	/** Numerical integration: Euler's method
	**/
	void Euler(float deltaTs);

	/** Numerical integration: RK2 method
   **/
	void RungeKutta2(float deltaTs);

	/** Numerical integration: RK4 method
   **/
	void RungeKutta4(float deltaTs);

	/** Numerical integration: Verlet method
   **/
	void Verlet(float deltaTs);


	void ClearTorque() { _torque = glm::vec3(0.0f, 0.0f, 0.0f); }

	//void AddTorque(const glm::vec3 torque) { _torque += torque; }

	void ComputeInverseInertiaTensor();

	void ApplyImpluseResponses(DynamicObject* objA, DynamicObject* objB, float deltaTS);

	glm::vec3 ComputeFricitionForce(glm::vec3 relative_velocity, glm::vec3 contact_normal, glm::vec3 force_normal, float mu);

	/**Update the model matrix with the current position, orientation and scale
	*
	*/
	void UpdateModelMatrix();

	/** Set up physics parameters for computation
	*  Specific parameters are determined by the physics simulation
	*/
	/** The total force on the object
	*/
	glm::vec3 _force;
	/** Position of the object
	*/
	glm::vec3 _position;
	/** Velocity of the object
	*/
	glm::vec3 _velocity;
	/** Previous position of the object for Verlet integration
	*/
	glm::vec3 _previous_position;
	/** The mass of the object
	*/
	float _mass;
	/** Scale of the object
	*/
	/** The radius of a bounding sphere of the object
	*/
	float _bRadius;
	/** Scale of the object
	*/
	glm::vec3 _scale;
	/** Orientation of the object
	*/

	//Elasticity of the object
	float _elasticity;

	float _dynamicMu;

	float _staticMu;


	glm::mat4 _orientation;

	//The position of the center of mass for the object
	glm::vec3 _COM;

	//???
	glm::mat3x3 _R;

	//The speed of the roration
	glm::vec3 _torque;

	//Angular velocity of the object
	glm::vec3 _angular_velocity;

	//Angular momentum of the object
	glm::vec3 _angular_momentum;

	//Inverse inertia tensor
	glm::mat3x3 _inertia_tensor_inverse;

	//Inverse body intertia tensor 
	glm::mat3x3 _body_inertia_tensor_inverse;

	//Damping accumlated system energy
	float global_dampping;

	//Friction
	float d_mu;






	/** A boolean variable to control the start of the simulation 
	*/
	bool _start;
    
	std::vector<DynamicObject*> _other_objects;

};

#endif //!_DynamicObject_H_

