#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/rotate_vector.hpp>

#include "DynamicObject.h"
#include "Utility.h"


DynamicObject::DynamicObject()
{
	//Set initial values for physics parameters (no forces acting on object)
	_force = glm::vec3(0.0f, 0.0f, 0.0f);
	_velocity = glm::vec3(0.0f, 0.0f, 0.0f);
	_mass = 0.0f;
	_bRadius = 0.0f;
	_previous_position = glm::vec3(0.0f);

	//Set initital vlaues for parameters
	_scale = glm::vec3(1.0f, 1.0f, 1.0f);
	_start = false;

	//Initialise angular dynamic parameters
	_torque = glm::vec3(0.0f, 0.0f, 0.0f);
	_angular_velocity = glm::vec3(0.0f, 0.0f, 0.0f);
	_angular_momentum = glm::vec3(0.0f, 0.0f, 0.0f);

	//Set rotation matrix to identity matrix
	_R = glm::mat3(1.0f, 0.0f, 0.0f,
				0.0f, 1.0f, 0.0f,
				0.0f, 0.0f, 1.0f);

	global_dampping = 15.0f;
	d_mu = 0.6;
}


DynamicObject::~DynamicObject()
{
}


void DynamicObject::StartSimulation(bool start)
{
	_start = start;

	glm::mat3 body_inertia;

	if (this->GetType() == SPHERE)
	{
		//Compute shere's body inertia
		body_inertia = glm::mat3{
			(2.0f / 3.0f) * _mass * std::pow(_bRadius, 2), 0, 0,
			0, (2.0f / 3.0f) * _mass * std::pow(_bRadius, 2), 0,
			0, 0, (2.0f / 3.0f) * _mass * std::pow(_bRadius, 2)
		};
	}

	//Inverse body inertia
	_body_inertia_tensor_inverse = glm::inverse(body_inertia);
	//Compute inertia tensor inverse
	ComputeInverseInertiaTensor();
	//Computer angular velocity
	_angular_velocity = _inertia_tensor_inverse * _angular_momentum;
}


void DynamicObject::Update(float deltaTs)
{
	//Main function in physics simulation

	if (_start == true)
	{
		glm::vec3 grav = glm::vec3(0.0f, -9.8 * _mass, 0.0f);
		//Clear all forces
		ClearForces();
		ClearTorque();

		//Compute the Net Force
		AddForce(grav);

		//Compute the Net torque
		//glm::vec3 t = glm::cross(_position, grav);
		//AddTorque(t);
		//Check Lecture 8 Slide 23 before adding more torques

		//Compute collisions and responses
		ComputeCollisionResponses(deltaTs);

		//Integration (can be change to euler, k2, k4 or verlet)
		//Euler(deltaTs);
		//RungeKutta2(deltaTs);
		//RungeKutta4(deltaTs);
		Verlet(deltaTs);

	}
	//If shape is quad then update the postitons of the verts using Xi(t) = R(t)Xi + Xcom(t)

	//Update the model matrix
	UpdateModelMatrix();
}


void DynamicObject::AngularMotionUpdate(float deltaTs)
{
	//Angular motion update
	_angular_momentum += _torque * deltaTs;
	//Comput inverse inertia tensor
	ComputeInverseInertiaTensor();
	//Update angular velocity
	_angular_velocity = _inertia_tensor_inverse * (_angular_momentum);

	//Construct skew matrix omega star
	glm::mat3 omega_start = glm::mat3(
		0.0f, -_angular_velocity.z, _angular_velocity.y,
		_angular_velocity.z, 0.0f, -_angular_velocity.x,
		-_angular_velocity.y, _angular_velocity.x, 0.0f);

	//Update rotation matrix
	_R += omega_start * _R * deltaTs;
}


void DynamicObject::ApplyImpluseResponses(DynamicObject* objA, DynamicObject* objB, float deltaTs)
{
	glm::vec3 position_distance = objA->GetPosition() - objB->GetPosition();
	glm::vec3 n = glm::normalize(position_distance);
	float rA = objA->GetBoundingRadius();
	float rB = objB->GetBoundingRadius();
	float massA = objA->GetMass();
	float massB = objB->GetMass();
	glm::mat3 inertia_tensor_inverseA = objA->GetInverseInertiaTensor();
	glm::mat3 inertia_tensor_inverseB = objB->GetInverseInertiaTensor();
	glm::vec3 velocity_A = objA->GetVelocity() + glm::cross(objA->GetAngularVelocity(), rA * n);
	glm::vec3 velocity_B = objB->GetVelocity() + glm::cross(objB->GetAngularVelocity(), rB * n);
	glm::vec3 relative_velocity = velocity_A - velocity_B;

	float elasiticity = objA->GetElasticity() * objB->GetElasticity(); //Multiple restituations of the two objects
	float one_over_massA = 1.0f / massA;
	float one_over_massB = 1.0f / massB;
	float J_numerator = -(1.0f + elasiticity) * glm::dot(relative_velocity, n);
	glm::vec3 cross1 = glm::cross(rA * n, n); //(r1 cross n), r1 is a float so turn it into a vector by mutiply by n frist
	glm::vec3 cross2 = glm::cross(rB * n, n); //(r2 cross n), r2 is a float so turn it into a vector by mutiply by n frist
	cross1 = inertia_tensor_inverseA * glm::cross(cross1, rA * n); //I1_inverse * (r1 cross n)
	cross2 = inertia_tensor_inverseB * glm::cross(cross2, rB * n); //I2_inverse * (r2 cross n)
	float total_inverse_mass = one_over_massA + one_over_massB;
	float angular_effect = glm::dot((cross1 + cross2), n);
	float J = J_numerator / (total_inverse_mass + angular_effect);

	glm::vec3 collision_impulse_vector = J * n;

	
	//Dynamic Fricition
	glm::vec3 contact_pointA = n * rA;
	glm::vec3 contact_pointB = -n * rB;
	glm::vec3 contact_force = (velocity_A * massA) - (velocity_B * massB);
	float total_friction = objA->GetDynmaicMu() + objB->GetDynmaicMu();
	glm::vec3 friction_force = ComputeFricitionForce(relative_velocity, n, contact_force, total_friction);

	glm::vec3 relative_forward_velocity = relative_velocity - glm::dot(relative_velocity, n) * n;
	float tangent_length = glm::length(relative_forward_velocity);

	//if moving forward, add torque for roation
	if (tangent_length - glm::length(friction_force / (massA + massB)) * deltaTs > 0.09f)
	{
		//std::cout << tangent_length << std::endl;
		objA->AddForce(friction_force);
		objB->AddForce(friction_force);
		glm::vec3 torque_armA = rA * n;
		glm::vec3 torque_armB = rB * n;
		glm::vec3 torqueA = glm::cross(torque_armA, friction_force);
		glm::vec3 torqueB = glm::cross(torque_armB, friction_force);
		//torqueA -= objA->GetAngularMomentum() * global_dampping;
		//torqueB -= objB->GetAngularMomentum() * global_dampping;
		objA->AddTorque(torqueA);
		objB->AddTorque(torqueB);
		std::cout << tangent_length - glm::length(friction_force / (massA + massB)) * deltaTs << std::endl;
	}
	
	//Object1
	glm::vec3 velocity = objA->GetVelocity() + collision_impulse_vector * one_over_massA;
	glm::vec3 angular_velocity = objA->GetAngularVelocity() + inertia_tensor_inverseA * glm::cross(rA * n, collision_impulse_vector);
	objA->SetVelocity(velocity);
	objA->SetAngularVelocity(angular_velocity);

	//Object2
	velocity = objB->GetVelocity() - collision_impulse_vector * one_over_massB;
	angular_velocity = objB->GetAngularVelocity() - inertia_tensor_inverseB * glm::cross(rB * n, collision_impulse_vector);
	objB->SetVelocity(velocity);
	objB->SetAngularVelocity(angular_velocity);
	
}


glm::vec3 DynamicObject::ComputeFricitionForce(glm::vec3 relative_velocity, glm::vec3 contact_normal, glm::vec3 force_normal, float mu)
{
	glm::vec3 friction_force;
	glm::vec3 forward_relative_velocity = relative_velocity - glm::dot(relative_velocity, contact_normal) * contact_normal;
	float tangent_length = glm::length(forward_relative_velocity);
	if (tangent_length > 1e-6f) //0.0000001 (1 millionth)
	{
		//Get normalized vector as the direction of travel
		glm::vec3 forward_direction = glm::normalize(forward_relative_velocity);
		//Friction direction is in the opposite direction of travel
		glm::vec3 friction_direction = -forward_direction;
		friction_force = friction_direction * mu * glm::length(force_normal); 

		return friction_force;
	}
	else return glm::vec3(0);
	
}


void DynamicObject::ComputeInverseInertiaTensor()
{
	_inertia_tensor_inverse = _R * _body_inertia_tensor_inverse * glm::transpose(_R);
}


void DynamicObject::Euler(float deltaTs)
{
	float oneOverMass = 1 / _mass;
	_velocity += (_force * oneOverMass) * deltaTs;
	_position += _velocity * deltaTs;

	AngularMotionUpdate(deltaTs);
}


void DynamicObject::RungeKutta2(float deltaTs)
{
	glm::vec3 force;
	glm::vec3 acceleration;
	glm::vec3 k0;
	glm::vec3 k1;

	//Evaluate once at t0
	force = _force;
	acceleration = force / _mass;
	k0 = deltaTs * acceleration;

	//Evaluation once at t0 + deltaT/2.0 using half of k0
	force = _force + k0 / 2.0f;
	acceleration = force / _mass;
	k1 = deltaTs * acceleration;

	//Evaluation once at t0 + deltaT usig k1
	_velocity += k1;
	_position += _velocity * deltaTs;

	AngularMotionUpdate(deltaTs);
}


void DynamicObject::RungeKutta4(float deltaTs)
{
	glm::vec3 force;
	glm::vec3 acceleration;
	glm::vec3 k0;
	glm::vec3 k1;
	glm::vec3 k2;
	glm::vec3 k3;

	//Evaluation once at t0 to find k0
	force = _force;
	acceleration = force / _mass;
	k0 = deltaTs * acceleration;

	//Evaluation twice t0 + dleatT/2.0 using hald of k0 and half of k1
	force = _force + k0 / 2.0f;
	acceleration = force / _mass;
	k1 = deltaTs * acceleration;

	force = _force + k1 / 2.0f;
	acceleration = force / _mass;
	k2 = deltaTs * acceleration;

	//Evaluate once at t0 + deltaT using k2
	force = _force + k2;
	acceleration = force / _mass;
	k3 = deltaTs * acceleration;

	//Evaluate t0 + deltaT using weighted sum of k0, k1, k2, and k3
	_velocity += (k0 + 2.0f * k1 + 2.0f * k2 + k3) / 6.0f;
	//Update position
	_position += _velocity * deltaTs;

	AngularMotionUpdate(deltaTs);
}


void DynamicObject::Verlet(float deltaTs)
{
	glm::vec3 acceleration = _force / _mass;
	_previous_position = _position - _velocity * deltaTs + 0.5f * acceleration * deltaTs * deltaTs;
	_position = -_previous_position + 2.0f * _position + acceleration * deltaTs * deltaTs;
	_velocity = (_position - _previous_position) / (2.0f * deltaTs);
	_velocity += acceleration * deltaTs;

	AngularMotionUpdate(deltaTs);
}


void DynamicObject::UpdateModelMatrix()
{
	//Update the model matrix with the current position, orientation and scale
	//convert 3x3 matrix to 4x4 matrix
	glm::mat4 model_rotation = glm::mat4(_R);

	_modelMatrix = glm::translate(glm::mat4(1), _position);
	_modelMatrix = _modelMatrix * model_rotation;
	_modelMatrix = glm::scale(_modelMatrix, _scale);
	_invModelMatrix = glm::inverse(_modelMatrix);
}


void DynamicObject::ComputeCollisionResponses(float deltaTs)
{
	float elasticity = this->GetElasticity();
	glm::vec3 n = glm::vec3(0.0f, 1.0f, 0.0f);
	glm::vec3 c0 = _position;
	glm::vec3 q = glm::vec3(0.0f, 0.0f, 0.0f);
	glm::vec3 c1 = _position + _velocity * deltaTs;
	glm::vec3 ci(0);
	float r = GetBoundingRadius();

	//Sphere to Plane collison (floor)
	bool collision = PFG::MovingSphereToPlaneCollision(n, c0, c1, q, r, ci);
	if (collision)
	{
		//Contact Force
		glm::vec3 plane_velocity = glm::vec3(0.0f, 0.0f, 0.0f);
		float collision_impulse = -(1 + elasticity) * glm::dot(_velocity - plane_velocity, n) / (1.0f / _mass);
		glm::vec3 collision_impulse_vector = collision_impulse * n;
		_velocity += collision_impulse_vector / _mass;
		glm::vec3 contact_force = glm::vec3(0.0f, 9.8f * _mass, 0.0f);
		AddForce(contact_force);

		//Compute Torque at contact point (friction)
		glm::vec3 relative_velocity = _velocity - plane_velocity;
		glm::vec3 friction_force = ComputeFricitionForce(relative_velocity, n, contact_force, d_mu);
		AddForce(friction_force);
		glm::vec3 torque_arm = r * n;
		glm::vec3 torque = glm::cross(torque_arm, friction_force);
		glm::vec3 torqueCopy(torque);
		glm::vec3 relative_forward_velocity = relative_velocity - glm::dot(relative_velocity, n) * n;
		
		//Adds global damming to roation 
		torque -= this->GetAngularMomentum() * global_dampping;
		AddTorque(torque);
		
		//if moving forward, add torque
		if (glm::length(relative_forward_velocity) - glm::length(friction_force / _mass) * deltaTs > 0.0f) //Change relative velocity to forward relative velocity
		{
			AddTorque(torqueCopy);
		}
		
	}


	//Checks there is another object
	for (int i = 0; i < _other_objects.size(); i++)
	{
		DynamicObject* _other = dynamic_cast<DynamicObject*>(_other_objects[i]);
		if (!_other)
		{
			std::cout << "Missing collison object (_other not set)" << std::endl;
			return;
		}

		//Sphere to sphere collision
		if (this->GetType() == SPHERE && _other->GetType() == SPHERE && _other != this)
		{

			glm::vec3 position_distance = _other->GetPosition() - _position;
			n = glm::normalize(position_distance);
			float r1 = _bRadius;
			float r2 = _other->GetBoundingRadius();
			float distance = glm::length(position_distance);

			if (distance <= r1 + r2)
			{
				ApplyImpluseResponses(this, _other, deltaTs);

			}
		}
	}
}

