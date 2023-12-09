#include "Scene.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <string>


/*! \brief Brief description.
*  Scene class is a container for loading all the game objects in your simulation or your game.
*
*/
Scene::Scene()
{
	// Set up your scene here......
	// Set a camera
	_camera = new Camera();
	// Don't start simulation yet
	_simulation_start = false;

	// Position of the light, in world-space
	_lightPosition = glm::vec3(10, 10, 0);

	// Create a game level object
	_level = new GameObject();

	fall = false;
	

	// Create the material for the game object- level
	Material* modelMaterial = new Material();
	// Shaders are now in files
	modelMaterial->LoadShaders("assets/shaders/VertShader.txt", "assets/shaders/FragShader.txt");
	// You can set some simple material properties, these values are passed to the shader
	// This colour modulates the texture colour
	modelMaterial->SetDiffuseColour(glm::vec3(0.8, 0.8, 0.8));
	// The material currently supports one texture
	// This is multiplied by all the light components (ambient, diffuse, specular)
	// Note that the diffuse colour set with the line above will be multiplied by the texture colour
	// If you want just the texture colour, use modelMaterial->SetDiffuseColour( glm::vec3(1,1,1) );
	modelMaterial->SetTexture("assets/textures/diffuse.bmp");
	// Need to tell the material the light's position
	// If you change the light's position you need to call this again
	modelMaterial->SetLightPosition(_lightPosition);
	// Tell the level object to use this material
	_level->SetMaterial(modelMaterial);

	// The mesh is the geometry for the object
	Mesh *groundMesh = new Mesh();
	// Load from OBJ file. This must have triangulated geometry
	groundMesh->LoadOBJ("assets/models/woodfloor.obj");
	// Tell the game object to use this mesh
	_level->SetMesh(groundMesh);
	_level->SetPosition(0.0f, 0.0f, 0.0f);
	_level->SetRotation(3.141590f, 0.0f, 0.0f);


	// Create the material for the game object- level
	Material* objectMaterial1 = new Material();
	Material* objectMaterial2 = new Material();
	Material* objectMaterial3 = new Material();
	Material* objectMaterial4 = new Material();
	// Shaders are now in files
	objectMaterial1->LoadShaders("assets/shaders/VertShader.txt", "assets/shaders/FragShader.txt");
	objectMaterial2->LoadShaders("assets/shaders/VertShader.txt", "assets/shaders/FragShader.txt");
	objectMaterial3->LoadShaders("assets/shaders/VertShader.txt", "assets/shaders/FragShader.txt");
	objectMaterial4->LoadShaders("assets/shaders/VertShader.txt", "assets/shaders/FragShader.txt");
	// You can set some simple material properties, these values are passed to the shader
	// This colour modulates the texture colour
	objectMaterial1->SetDiffuseColour(glm::vec3(1.0, 0.0, 0.0));
	objectMaterial2->SetDiffuseColour(glm::vec3(0.0, 0.0, 1.0));
	objectMaterial3->SetDiffuseColour(glm::vec3(0.0, 1.0, 0.0));
	objectMaterial4->SetDiffuseColour(glm::vec3(1.0, 1.0, 0.0));
	// The material currently supports one texture
	// This is multiplied by all the light components (ambient, diffuse, specular)
	// Note that the diffuse colour set with the line above will be multiplied by the texture colour
	// If you want just the texture colour, use modelMaterial->SetDiffuseColour( glm::vec3(1,1,1) );
	objectMaterial1->SetTexture("assets/textures/default.bmp");
	objectMaterial2->SetTexture("assets/textures/default.bmp");
	objectMaterial3->SetTexture("assets/textures/default.bmp");
	objectMaterial4->SetTexture("assets/textures/default.bmp");
	// Need to tell the material the light's position
	// If you change the light's position you need to call this again
	objectMaterial1->SetLightPosition(_lightPosition);
	objectMaterial2->SetLightPosition(_lightPosition);
	objectMaterial3->SetLightPosition(_lightPosition);
	objectMaterial4->SetLightPosition(_lightPosition);
	// Tell the level object to use this material

	// Set the geometry for the object
	Mesh *modelMesh = new Mesh();
	// Load from OBJ file. This must have triangulated geometry
	modelMesh->LoadOBJ("assets/models/sphere.obj");
	// Tell the game object to use this mesh


	//Float values to get from text file
	float posX, posY, posZ, velX, velY, velZ, mass, elastic, staticMu, dynamicMu, radius, scaleX, scaleY, scaleZ;

	//Creats a list of 4 dynamic objects with values fom text files
	for (int i = 0; i < 4; i++)
	{
		physicsObjects.push_back(new DynamicObject());

		physicsObjects[i]->SetMesh(modelMesh);
		physicsObjects[i]->SetType(SPHERE);

		ifstream fin("physics_object" + to_string(i + 1) + ".txt");
		posX = posY = posZ = velX = velY = velZ = mass = elastic = staticMu = dynamicMu = radius = scaleX = scaleY = scaleZ = NULL;

		while (fin >> posX >> posY >> posZ >> velX >> velY >> velZ >> mass >>
			elastic >> staticMu >> dynamicMu >> radius >> scaleX >> scaleY >> scaleZ)
		{
			physicsObjects[i]->SetPosition(glm::vec3(posX, posY, posZ));
			physicsObjects[i]->SetVelocity(glm::vec3(velX, velY, velZ));
			physicsObjects[i]->SetMass(mass);
			physicsObjects[i]->SetElasticity(elastic);
			physicsObjects[i]->SetStaticMu(staticMu);
			physicsObjects[i]->SetDynamicMu(dynamicMu);
			physicsObjects[i]->SetBoundingRadius(radius);
			physicsObjects[i]->SetScale(glm::vec3(scaleX, scaleY, scaleZ));
		}
	}

	physicsObjects[0]->SetMaterial(objectMaterial1);
	physicsObjects[1]->SetMaterial(objectMaterial2);
	physicsObjects[2]->SetMaterial(objectMaterial3);
	physicsObjects[3]->SetMaterial(objectMaterial4);
	
	
}

Scene::~Scene()
{
	// You should neatly clean everything up here
	delete &physicsObjects;
	delete _level;
	delete _camera;
}

void Scene::Update(float deltaTs, Input* input)
{
	if (input->cmd_x)
	{
		_simulation_start = true;
	}
	if (_simulation_start)
	{
		physicsObjects[0]->StartSimulation(_simulation_start);
		physicsObjects[1]->StartSimulation(_simulation_start);
		physicsObjects[2]->StartSimulation(_simulation_start);
		physicsObjects[3]->StartSimulation(_simulation_start);

		physicsObjects[0]->SetCollisionObject(physicsObjects);
		physicsObjects[1]->SetCollisionObject(physicsObjects);
		physicsObjects[2]->SetCollisionObject(physicsObjects);
		physicsObjects[3]->SetCollisionObject(physicsObjects);
	}
	
	physicsObjects[0]->Update(deltaTs);
	physicsObjects[1]->Update(deltaTs);
	physicsObjects[2]->Update(deltaTs);
	physicsObjects[3]->Update(deltaTs);
	_level->Update(deltaTs);
	_camera->Update(input);

	_viewMatrix = _camera->GetView();
	_projMatrix = _camera->GetProj();
														
}

void Scene::Draw()
{
	// Draw objects, giving the camera's position and projection
	physicsObjects[0]->Draw(_viewMatrix, _projMatrix);
	physicsObjects[1]->Draw(_viewMatrix, _projMatrix);
	physicsObjects[2]->Draw(_viewMatrix, _projMatrix);
	physicsObjects[3]->Draw(_viewMatrix, _projMatrix);
	_level->Draw(_viewMatrix, _projMatrix);

}


