#include <imgui\imgui.h>
#include <imgui\imgui_impl_sdl_gl3.h>
#include <glm/glm.hpp>
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <glm/gtc/quaternion.hpp>
#include <deque>

#define GRAVEDAD -9.81f
float impulsoInicial = 250.f;

using v3 = glm::vec3;
using m4 = glm::mat4;
using m3 = glm::mat3;

struct Force {
	v3 puntAp;
	v3 power;
};

namespace Cube {
	extern void setupCube();
	extern void cleanupCube();
	extern void updateCube(const glm::mat4& transform);
	extern void drawCube();
	extern const float halfW;
	
	glm::quat orientation;
	glm::quat orientationO;
	v3 pos;
	v3 posO;
	m3 iBody;
	m3 impulso;
	v3 vel;
	v3 velO;
	v3 velA;
	v3 velAO;
	v3 linM;
	v3 angM;
	std::deque<Force> sumF;
	v3 torque;
	float mass;

	extern glm::vec3 verts[];
	v3 boxNorms[6]{

	};
}

namespace UV {
	float currentTime=0;
}

float eCo=0.5;
float fCo=0.5;
#define NPARTICLES 1
#define MASS 1.f//F
struct particle {
	v3 P;
	v3 Po;
	v3 V;
	v3 Vo;
	v3 F;
};

//PLANOS
const glm::vec3 planes[6] = {
	{ 0, 1, 0 },
	{ 0, -1, 0 },
	{ 1, 0, 0 },
	{ -1, 0, 0 },
	{ 0, 0, 1 },
	{ 0, 0, -1 }
};

const glm::vec3 planePoint[6]{
	{ 0, 0, 0 }, //BOT
	{ 0, 10, 0 }, //TOP
	{ -5, 5, 0 }, //LEFT
	{ 5, 5, 0 }, //RIGHT
	{ 0, 5, -5 }, //BACK
	{ 0, 5, 5 }	//FRONT
};

//COLISIONES PLANO
float planeD(v3 normal, v3 puntoP) {
	return -glm::dot(normal, puntoP);
}

bool hasCollided(v3 location, v3 locationO, v3 normal, float d) {
	return ((glm::dot(normal, locationO) + d)*(glm::dot(normal, location) + d) <= 0);
}

std::pair<bool, int> isOutOfCube() {
	for (int i = 0; i < 6; ++i) {
		for (int j = 0; j < 8; ++j) {
			if (hasCollided(Cube::pos + Cube::verts[j], Cube::posO + Cube::verts[j], planes[i], planeD(planes[i], planePoint[i])))
				return std::make_pair(true, j);
		}
	}
	return std::make_pair(false, -1);
}

void rebote(particle &particula, glm::vec3 normal, glm::vec3 planeSpot) {
	normal = glm::normalize(normal);
	float d = planeD(normal, planeSpot);

	particula.Po -= (1 + eCo) * (glm::dot(normal, particula.Po) + d)*normal;

	particula.P -= (1 + eCo) * (glm::dot(normal, particula.P) + d)*normal;

	particula.V = particula.V - (1 + eCo) * glm::dot(normal, particula.V)*normal;

	glm::vec3 velocidadNormal = glm::dot(normal, particula.Vo)*normal;
	glm::vec3 velocidadTangencial = particula.Vo - velocidadNormal;

	particula.V -= fCo*velocidadTangencial;
}

float getRandBetweenFloats(float x, float y) {
	float random = ((float)rand()) / (float)RAND_MAX;
	float diff = y - x;
	float r = random * diff;
	return x + r;
}

bool show_test_window = false;

void GUI() {
	bool show = true;
	ImGui::Begin("Physics Parameters", &show, 0);

	// Do your GUI code here....
	{	
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);//FrameRate
		
	}
	// .........................
	
	ImGui::End();

	// Example code -- ImGui test window. Most of the sample code is in ImGui::ShowTestWindow()
	if(show_test_window) {
		ImGui::SetNextWindowPos(ImVec2(650, 20), ImGuiSetCond_FirstUseEver);
		ImGui::ShowTestWindow(&show_test_window);
	}
}

void applyForce(Force f) {
	Cube::sumF.push_back(f);
}

void bolzano(double dt, float obj) {

	bolzano(dt / 2, obj);
}

void myUpdateCube(float dt){
	Cube::sumF.push_back({ Cube::pos, {0, GRAVEDAD, 0} });

	for (std::deque<Force>::iterator i = Cube::sumF.begin(); i != Cube::sumF.end(); ++i) {
		Cube::linM += dt*i->power;
	}
	Cube::velO = Cube::vel;
	Cube::vel = Cube::linM / Cube::mass;
	Cube::posO = Cube::pos;
	Cube::pos = Cube::pos + Cube::vel * dt;	

	Cube::angM = dt*Cube::torque;

	glm::mat3 or = glm::mat3_cast(Cube::orientation);
	Cube::impulso = or*(glm::inverse(Cube::iBody))*glm::transpose(or);

	Cube::velAO = Cube::velA;
	Cube::velA = Cube::impulso*Cube::angM;

	Cube::orientationO = Cube::orientation;
	Cube::orientation += dt*(0.5f * glm::quat(0.f, Cube::velA)* Cube::orientation);
	Cube::orientation = glm::normalize(Cube::orientation);

	Cube::sumF.clear();
	Cube::sumF.shrink_to_fit();

	for (int i = 0; i < 8; ++i) {
		Cube::verts[i];
	}
	std::pair<bool, int> myPair = isOutOfCube();
	if(myPair.first){
		Cube::vel = Cube::velO;
		Cube::velA = Cube::velAO;
		Cube::pos = Cube::posO;
		Cube::orientation = Cube::orientationO;

		switch (myPair.second) {//Los cases son los mismos 2 a 2, porque los que nos importa es la coordenada, en función del plano (top y bot, y), (left y right, x), (front y back, z)
		case 0:
		case 1:
			bolzano(dt / 2, planes[myPair.second].y);
			break;
		case 2:
		case 3:
			bolzano(dt / 2, planes[myPair.second].x);
			break;
		case 4:
		case 5:
			bolzano(dt / 2, planes[myPair.second].z);
			break;
		}
	}
}

void PhysicsInit() {
	srand((unsigned)time(NULL));
	Cube::mass = 1.f;

	Cube::linM = { 0, 0, 0 };
	Cube::angM = { 0, 0, 0 };

	Cube::vel = { 0, 0, 0 };
	getRandBetweenFloats(-900, 900);
	Cube::pos = { getRandBetweenFloats(-5, 5), getRandBetweenFloats(0, 10), getRandBetweenFloats(-5, 5) }; //LOS RANDOM VAN MAL
	Cube::iBody = {	1.f/12.f*Cube::mass*(glm::pow(Cube::halfW*2, 2) + glm::pow(Cube::halfW * 2, 2)), 0, 0, //CAMBIAR NOMBRES
							0, 1.f / 12.f * Cube::mass*(glm::pow(Cube::halfW * 2, 2) + glm::pow(Cube::halfW * 2, 2)), 0,
							0, 0, 1.f / 12.f * Cube::mass*(glm::pow(Cube::halfW * 2, 2) + glm::pow(Cube::halfW * 2, 2))};
	

	applyForce({ Cube::pos + v3{ 0.065, 0.065, 0.065 } ,{ 0, 300, 0 } });
	applyForce({ Cube::pos ,{ 0, 30000000, 0 } });
	
	Cube::torque = glm::cross(Cube::sumF.begin()->puntAp - Cube::pos, Cube::sumF.begin()->power);

	std::cout << "torque: " << std::endl;
	std::cout << Cube::torque.x << " " << Cube::torque.y << " " << Cube::torque.z << " " << std::endl;
	Cube::sumF.clear();
	applyForce({ Cube::pos ,{ 0, 123.456789, 0 } });

	Cube::setupCube();
}

void PhysicsUpdate(float dt) {
	myUpdateCube(dt);
	m4 pos = {	1, 0, 0, 0,
				0, 1, 0, 0,
				0, 0, 1, 0,
				Cube::pos.x, Cube::pos.y, Cube::pos.z, 1 };

	Cube::updateCube(pos*glm::mat4_cast(Cube::orientation));
	Cube::drawCube();
	
	UV::currentTime += dt;
}

void PhysicsCleanup() {
	Cube::cleanupCube();
}