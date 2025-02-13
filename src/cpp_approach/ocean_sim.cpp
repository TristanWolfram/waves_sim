#include "ocean_sim.hpp"
#include "tinyxml2.h"
#include <iostream>
#include <Stonefish/entities/statics/Plane.h>
#include <Stonefish/entities/solids/Sphere.h>
#include <Stonefish/sensors/vision/ColorCamera.h>
#include <Stonefish/entities/solids/Box.h>
#include <Stonefish/core/Robot.h>
#include <Stonefish/core/GeneralRobot.h>
#include <Stonefish/sensors/scalar/IMU.h>
#include <Stonefish/core/NED.h>
#include <Stonefish/sensors/scalar/RotaryEncoder.h>
#include <Stonefish/actuators/Servo.h>
#include <Stonefish/core/ScenarioParser.h>
#include <Stonefish/entities/forcefields/Uniform.h>
#include <Stonefish/entities/forcefields/Jet.h>
#include <Stonefish/entities/statics/Obstacle.h>

MySimulationManager::MySimulationManager(sf::Scalar stepsPerSecond) : SimulationManager(stepsPerSecond)
{
}

void MySimulationManager::BuildScenario()
{

    sf::ScenarioParser parser(this);
    parser.Parse("../testing.scn");
    parser.SaveLog("materials.log");

    // Physical materials
    // CreateMaterial("Aluminium", 2710.0, 0.7);
    // CreateMaterial("Steel", 7891.0, 0.9);
    // CreateMaterial("Stone", 2700.0, 0.5);
    // CreateMaterial("robot", 1000.0, 0.5);
    // CreateMaterial("Neutral", 900.0, 0.5);
    // CreateMaterial("Container", 500.0, 0.8);
    // CreateMaterial("Boat", 450.0, 0.8);
    // SetMaterialsInteraction("Aluminium", "Aluminium", 0.7, 0.5);
    // SetMaterialsInteraction("Steel", "Steel", 0.4, 0.2);
    // SetMaterialsInteraction("Aluminium", "Steel", 0.6, 0.4);
    // SetMaterialsInteraction("Neutral", "Steel", 0.9, 0.9);

    //Graphical materials (looks)
    // CreateLook("gray", sf::Color::Gray(0.5f), 0.3f, 0.2f);
    // CreateLook("red", sf::Color::RGB(1.f,0.f,0.f), 0.1f, 0.f);
    // CreateLook("Yellow", sf::Color::RGB(1.0f, 0.9f, 0.0f), 0.3f);
    // CreateLook("Gray", sf::Color::Gray(0.1f), 0.4f, 0.5f);
    // CreateLook("Green", sf::Color::RGB(0.0f, 0.2f, 0.0f), 0.9f);
    // CreateLook("Invisible", sf::Color::RGB(1.0f, 1.0f, 1.0f), 0.2f, 0.0f); // Assuming alpha is the last parameter
    // CreateLook("Black", sf::Color::RGB(0.0f, 0.0f, 0.0f), 0.2f);
    // CreateLook("White", sf::Color::RGB(1.0f, 1.0f, 1.0f), 0.2f);
    // CreateLook("Brown", sf::Color::RGB(0.5f, 0.3f, 0.0f), 0.2f);
    // CreateLook("Pink", sf::Color::RGB(1.0f, 0.078f, 0.576f), 0.8f);
    CreateLook("SimpleIsland", sf::Color::Gray(1.0f), 0.2f, 0.f, 0.f, "../../data/textures/IslandTexture.jpg");
    CreateLook("Container", sf::Color::Gray(1.0f), 0.2f, 0.f, 0.f, "../../data/textures/container.jpg");
    CreateLook("Fisherboat", sf::Color::Gray(1.0f), 0.2f, 0.f, 0.f, "../../data/textures/fisher_boat.jpg");

    getNED()->Init(63.446827, 10.421906, 0.0);

    // // OCEAN
    // getMaterialManager()->CreateFluid("OceanWater", 1031.0, 0.002, 1.33);
    // EnableOcean(0.0, getMaterialManager()->getFluid("OceanWater"));
    // getOcean()->setWaterType(0.2);
    // getOcean()->AddVelocityField(new sf::Uniform(sf::Vector3(1.0, 0.0, 0.0)));
    // getOcean()->AddVelocityField(new sf::Jet(sf::Vector3(0.0, 0.0, 3.0), sf::Vector3(0.0, 1.0, 0.0), 0.2, 2.0));

    // // ATMOSPHERE
    // getAtmosphere()->SetupSunPosition(20.0, 10.0);
    // getAtmosphere()->AddVelocityField(new sf::Uniform(sf::Vector3(1.0, 0.0, 0.0)));
    // getAtmosphere()->AddVelocityField(new sf::Jet(sf::Vector3(0.0, 0.0, 3.0), sf::Vector3(0.0, 1.0, 0.0), 0.2, 2.0));

    //Create environment
    // sf::Plane* plane = new sf::Plane("Ground", 10000.0, "Steel", "gray");
    // AddStaticEntity(plane, sf::I4());

    //Create object

    sf::BodyPhysicsMode surfacemode = sf::BodyPhysicsMode::SURFACE;
    sf::BodyPhysicsSettings settings;
    settings.mode = surfacemode;

    sf::Obstacle* ball = new sf::Obstacle("Ball", 0.5, sf::I4(), "Aluminium", "Yellow");
    AddStaticEntity(ball, sf::Transform(sf::Quaternion(0.0, 0.0, 0.0), sf::Vector3(0.0, 0.0, -5.0)));

    sf::Obstacle* rocks = new sf::Obstacle("Rocks", "../../data/obstacles/rocks.obj", 1.0, sf::I4(), "../../data/obstacles/rocks_phys.obj", 1.0, sf::I4(), true, "Rock", "Gray");
    AddStaticEntity(rocks, sf::Transform(sf::Quaternion(0.0, 0.0, 0.0), sf::Vector3(0.0, 0.0, 10.0)));

    // sf::Sphere* sph = new sf::Sphere("Sphere", settings, 0.1, sf::I4(), "Aluminium", "red");
    // AddSolidEntity(sph, sf::Transform(sf::IQ(), sf::Vector3(0.0,0.0,-1.0)));
}