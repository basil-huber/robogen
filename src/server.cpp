/*
 * @(#) server.cpp   1.0   Mar 5, 2013
 *
 * Andrea Maesani (andrea.maesani@epfl.ch)
 *
 * The ROBOGEN Framework
 * Copyright © 2012-2013 Andrea Maesani
 *
 * Laboratory of Intelligent Systems, EPFL
 *
 * This file is part of the ROBOGEN Framework.
 *
 * The ROBOGEN Framework is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License (GPL)
 * as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @(#) $Id$
 */
#include <iostream>

#include "config/ConfigurationReader.h"
#include "config/RobogenConfig.h"
#include "scenario/Scenario.h"
#include "scenario/ScenarioFactory.h"
#include "utils/network/ProtobufPacket.h"
#include "utils/network/TcpSocket.h"
#include "utils/RobogenCollision.h"
#include "utils/RobogenUtils.h"
#include "viewer/KeyboardHandler.h"
#include "Models.h"
#include "RenderModels.h"
#include "Robogen.h"
#include "Robot.h"
#include "robogen.pb.h"

using namespace robogen;

// ODE World
dWorldID odeWorld;

// Container for collisions
dJointGroupID odeContactGroup;

bool interrupted;

int main(int argc, char* argv[]) {

	interrupted = false;

	if (argc != 2) {
		std::cerr << "Please, provide server port." << std::endl;
		return EXIT_FAILURE;
	}

	// Parameters: <PORT>
	int port = std::atoi(argv[1]);

	TcpSocket socket;
	bool rc = socket.create(port);
	if (!rc) {
		std::cerr << "Cannot listen for incoming connections on port " << port
				<< std::endl;
		return EXIT_FAILURE;
	}

	while (!interrupted) {

		// Wait for client to connect
		std::cout << "Waiting for clients..." << std::endl;

		rc = socket.accept();

		if (rc) {

			std::cout << "Client connected..." << std::endl;

			while (true) {

				// ---------------------------------------
				// Simulator initialization
				// ---------------------------------------

				dInitODE();

				// Create ODE world
				odeWorld = dWorldCreate();

				// Set gravity [mm/s]
				dWorldSetGravity(odeWorld, 0, 0, -9.81);

				dWorldSetERP(odeWorld, 0.4);
				dWorldSetCFM(odeWorld, 0.2);

				// Create collision world
				dSpaceID odeSpace = dSimpleSpaceCreate(0);

				// Create contact group
				odeContactGroup = dJointGroupCreate(0);

				// ---------------------------------------
				// Decode solution
				// ---------------------------------------

				ProtobufPacket<robogenMessage::Robot> packet;

				// 1) Read packet header
				std::vector<unsigned char> headerBuffer;
				socket.read(headerBuffer,
						ProtobufPacket<robogenMessage::Robot>::HEADER_SIZE);
				unsigned int packetSize = packet.decodeHeader(headerBuffer);

				// 2) Read packet size
				std::vector<unsigned char> payloadBuffer;
				socket.read(payloadBuffer, packetSize);
				packet.decodePayload(payloadBuffer);

				// ---------------------------------------
				//  Decode configuration file
				// ---------------------------------------

				boost::shared_ptr<RobogenConfig> configuration =
						ConfigurationReader::parseConfigurationFile(
								packet.getMessage()->configuration());
				if (configuration == NULL) {
					std::cout
							<< "Problems parsing the configuration file. Quit."
							<< std::endl;
					return EXIT_FAILURE;
				}

				// ---------------------------------------
				// Setup environment
				// ---------------------------------------

				boost::shared_ptr<Scenario> scenario =
						ScenarioFactory::createScenario(configuration);
				if (scenario == NULL) {
					return EXIT_FAILURE;
				}

				// ---------------------------------------
				// Generate Robot
				// ---------------------------------------
				boost::shared_ptr<Robot> robot(new Robot(odeWorld, odeSpace));
				if (!robot->init(*packet.getMessage().get())) {
					std::cout << "Problems decoding the robot. Quit."
							<< std::endl;
					return EXIT_FAILURE;
				}

				// Register sensors
				std::vector<boost::shared_ptr<Sensor> > sensors =
						robot->getSensors();
				std::vector<boost::shared_ptr<TouchSensor> > touchSensors;
				for (unsigned int i = 0; i < sensors.size(); ++i) {
					if (boost::dynamic_pointer_cast<TouchSensor>(sensors[i])) {
						touchSensors.push_back(
								boost::dynamic_pointer_cast<TouchSensor>(
										sensors[i]));
					}
				}

				// Register robot motors
				std::vector<boost::shared_ptr<Motor> > motors =
						robot->getMotors();

				std::cout << "S: " << sensors.size() << std::endl;
				std::cout << "M: " << motors.size() << std::endl;

				// Register brain and body parts
				boost::shared_ptr<NeuralNetwork> neuralNetwork =
						robot->getBrain();
				std::vector<boost::shared_ptr<Model> > bodyParts =
						robot->getBodyParts();

				// Initialize scenario
				if (!scenario->init(odeWorld, odeSpace, robot)) {
					std::cout << "Cannot initialize scenario. Quit."
							<< std::endl;
					return EXIT_FAILURE;
				}

				// Setup environment
				boost::shared_ptr<Environment> env = scenario->getEnvironment();

				if (!scenario->setupSimulation()) {
					std::cout << "Cannot setup scenario. Quit." << std::endl;
					return EXIT_FAILURE;
				}

				// ---------------------------------------
				// OSG Main Loop
				// ---------------------------------------

				int count = 0;
				double deltaSecs = 0;
				double t = 0;
				double lastLightSensorUpdateT = 0;
				osg::Timer_t prevTime = osg::Timer::instance()->tick();
				while (t < configuration->getSimulationTime()) {

					double step = configuration->getTimeStepLength();
					const osg::Timer_t now = osg::Timer::instance()->tick();
					deltaSecs += osg::Timer::instance()->delta_s(prevTime, now);
					prevTime = now;

					while (deltaSecs > step) {

						deltaSecs -= step;
						t += step;

						if ((count++) % 100 == 0) {
							std::cout << "Step!" << count << std::endl;
						}

						// Prepare touch sensors for collision detection
						for (unsigned int i = 0; i < touchSensors.size(); ++i) {
							touchSensors[i]->reset();
						}

						// Collision detection
						dSpaceCollide(odeSpace, 0, odeCollisionCallback);

						// Step the world by one timestep
						dWorldStep(odeWorld, step);

						// Empty contact groups used for collisions handling
						dJointGroupEmpty(odeContactGroup);

						float networkInput[MAX_INPUT_NEURONS];
						float networkOutputs[MAX_OUTPUT_NEURONS];

						// Feed neural network
						for (unsigned int i = 0; i < bodyParts.size(); ++i) {
							if (boost::dynamic_pointer_cast<PerceptiveComponent>(
									bodyParts[i])) {
								boost::dynamic_pointer_cast<PerceptiveComponent>(
										bodyParts[i])->updateSensors(env);
							}
						}

						bool updateLightSensors = false;
						if (t - lastLightSensorUpdateT
								> LightSensor::DEFAULT_SENSOR_UPDATE_TIMESTEP) {
							updateLightSensors = true;
							lastLightSensorUpdateT = t;
						}
						for (unsigned int i = 0; i < sensors.size(); ++i) {

							if (boost::dynamic_pointer_cast<TouchSensor>(
									sensors[i])) {
								networkInput[i] = boost::dynamic_pointer_cast<
										TouchSensor>(sensors[i])->read();
							} else if (boost::dynamic_pointer_cast<LightSensor>(
									sensors[i])) {

								// Light sensors are updated with a different frequency than the simulation timestep
								networkInput[i] = boost::dynamic_pointer_cast<
										LightSensor>(sensors[i])->read(
										env->getLightSources(),
										env->getAmbientLight(),
										updateLightSensors);

							} else if (boost::dynamic_pointer_cast<SimpleSensor>(
									sensors[i])) {
								networkInput[i] = boost::dynamic_pointer_cast<
										SimpleSensor>(sensors[i])->read();
							}
						}

						// Step the neural network
						::step(neuralNetwork.get());

						// Fetch the neural network ouputs
						::fetch(neuralNetwork.get(), &networkOutputs[0]);

						// Send control to motors
						for (unsigned int i = 0; i < motors.size(); ++i) {
							if (boost::dynamic_pointer_cast<ServoMotor>(
									motors[i])) {

								boost::shared_ptr<ServoMotor> motor =
										boost::dynamic_pointer_cast<ServoMotor>(
												motors[i]);

								if (motor->isVelocityDriven()) {
									motor->setVelocity(networkOutputs[i]);
								} else {
									motor->setPosition(
											osg::inDegrees(networkOutputs[i]));
								}
							}
						}

						if (!scenario->afterSimulationStep()) {
							std::cout
									<< "Cannot execute scenario after simulation step. Quit."
									<< std::endl;
							return EXIT_FAILURE;
						}

					}

				}

				if (!scenario->endSimulation()) {
					std::cout << "Cannot complete scenario. Quit." << std::endl;
					return EXIT_FAILURE;
				}

				// ---------------------------------------
				// Compute fitness
				// ---------------------------------------

				double fitness = scenario->getFitness();
				std::cout << "Fitness for the current solution: " << fitness
						<< std::endl;

				// ---------------------------------------
				// Simulator finalization
				// ---------------------------------------

				// Destroy ODE space
				dSpaceDestroy(odeSpace);

				// Destroy ODE world
				dWorldDestroy(odeWorld);

				// Destroy the ODE engine
				dCloseODE();

				// ---------------------------------------
				// Test with Pradeep
				// ---------------------------------------
				boost::shared_ptr<robogenMessage::EvaluationResult> evalResultPacket(
						new robogenMessage::EvaluationResult());
				evalResultPacket->set_fitness(fitness);
				evalResultPacket->set_id(packet.getMessage()->id());
				ProtobufPacket<robogenMessage::EvaluationResult> evalResult;
				evalResult.setMessage(evalResultPacket);

				std::vector<unsigned char> sendBuffer;
				evalResult.forge(sendBuffer);

				socket.write(sendBuffer);

				std::cout << "Visualization terminated" << std::endl;

			}

		} else {
			std::cerr << "Cannot connect to client. Exiting." << std::endl;
			socket.close();
			return EXIT_FAILURE;
		}

	}

	return EXIT_SUCCESS;
}
