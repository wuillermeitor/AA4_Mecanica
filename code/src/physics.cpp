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
	v3 pos;
	m3 iBody;
	m3 impulso;
	v3 vel;
	v3 velA;
	v3 linM;
	v3 angM;
	std::deque<Force> sumF;
	v3 torque;
	float mass;
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
	{ 0, 0, 0 },
	{ 0, 10, 0 },
	{ -5, 5, 0 },
	{ 5, 5, 0 },
	{ 0, 5, -5 },
	{ 0, 5, 5 }
};

//COLISIONES PLANO
float planeD(glm::vec3 normal, glm::vec3 puntoP) {
	return -glm::dot(normal, puntoP);
}

bool hasCollided(particle particula, glm::vec3 normal, float d) {
	return ((glm::dot(normal, particula.Po) + d)*(glm::dot(normal, particula.P) + d) <= 0);
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

void myUpdateCube(float dt){
	Cube::sumF.push_back({ Cube::pos, {0, GRAVEDAD, 0} });

	for (std::deque<Force>::iterator i = Cube::sumF.begin(); i != Cube::sumF.end(); ++i) {
		Cube::linM += dt*i->power;
		Cube::torque = glm::cross(glm::abs(i->puntAp - Cube::pos),i->power);
	}
	
	Cube::vel = Cube::linM / Cube::mass;
	Cube::pos = Cube::pos + Cube::vel * dt;	

	Cube::angM += dt*Cube::torque;

	glm::mat3 or = glm::mat3_cast(Cube::orientation);
	Cube::impulso = or*(glm::inverse(Cube::iBody))*glm::transpose(or);

	Cube::velA = Cube::impulso*Cube::angM;

	Cube::orientation += dt*(0.5f * glm::quat(0.f, Cube::velA)* Cube::orientation);
	Cube::orientation = glm::normalize(Cube::orientation);

	//m4 tempVelA();
	//Cube::iBody += dt*(Cube::iBody*Cube::velA);//FALLA ESTO, REESCRIBIR LA VELOCIDAD ANGULAR COMO UNA MATRIX

	Cube::sumF.clear();
	Cube::sumF.shrink_to_fit();
}

void PhysicsInit() {
	srand((unsigned)time(NULL));
	Cube::mass = 1.f;

	Cube::linM = { 0, 0, 0 };
	Cube::angM = { 0, 0, 0 };

	Cube::vel = { 0, 0, 0 };
	getRandBetweenFloats(-900, 900);
	Cube::pos = { getRandBetweenFloats(-5, 5), getRandBetweenFloats(0, 10), getRandBetweenFloats(-5, 5) }; //LOS RANDOM VAN MAL
	std::cout << Cube::pos.x << " " << Cube::pos.y << " " << Cube::pos.z << std::endl;
	Cube::iBody = {	1.f/12.f*Cube::mass*(glm::pow(Cube::halfW*2, 2) + glm::pow(Cube::halfW * 2, 2)), 0, 0, //CAMBIAR NOMBRES
							0, 1.f / 12.f * Cube::mass*(glm::pow(Cube::halfW * 2, 2) + glm::pow(Cube::halfW * 2, 2)), 0,
							0, 0, 1.f / 12.f * Cube::mass*(glm::pow(Cube::halfW * 2, 2) + glm::pow(Cube::halfW * 2, 2))};
	//Cube::vel = { 0, 0, 0 };
	//Cube::velO = { 0, 0, 0 };


	applyForce({ Cube::pos + v3{0, 0.25, 0} ,{ 150, 400, 200 } });
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