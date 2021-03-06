#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"

//#include <iostream>
//using namespace std;

#ifdef __PX4_NUTTX
#include <systemlib/param/param.h>
#endif

void QuadControl::Init()
{
  BaseController::Init();

  // variables needed for integral control
  integratedAltitudeError = 0;
    
#ifndef __PX4_NUTTX
  // Load params from simulator parameter system
  ParamsHandle config = SimpleConfig::GetInstance();
   
  // Load parameters (default to 0)
  kpPosXY = config->Get(_config+".kpPosXY", 0);
  kpPosZ = config->Get(_config + ".kpPosZ", 0);
  KiPosZ = config->Get(_config + ".KiPosZ", 0);
     
  kpVelXY = config->Get(_config + ".kpVelXY", 0);
  kpVelZ = config->Get(_config + ".kpVelZ", 0);

  kpBank = config->Get(_config + ".kpBank", 0);
  kpYaw = config->Get(_config + ".kpYaw", 0);

  kpPQR = config->Get(_config + ".kpPQR", V3F());

  maxDescentRate = config->Get(_config + ".maxDescentRate", 100);
  maxAscentRate = config->Get(_config + ".maxAscentRate", 100);
  maxSpeedXY = config->Get(_config + ".maxSpeedXY", 100);
  maxAccelXY = config->Get(_config + ".maxHorizAccel", 100);

  maxTiltAngle = config->Get(_config + ".maxTiltAngle", 100);

  minMotorThrust = config->Get(_config + ".minMotorThrust", 0);
  maxMotorThrust = config->Get(_config + ".maxMotorThrust", 100);
#else
  // load params from PX4 parameter system
  //TODO
  param_get(param_find("MC_PITCH_P"), &Kp_bank);
  param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}

VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
  // Convert a desired 3-axis moment and collective thrust command to 
  //   individual motor thrust commands
  // INPUTS: 
  //   collThrustCmd: desired collective thrust [N]
  //   momentCmd: desired rotation moment about each axis [N m]
  // OUTPUT:
  //   set class member variable cmd (class variable for graphing) where
  //   cmd.desiredThrustsN[0..3]: motor commands, in [N]

  // HINTS: 
  // - you can access parts of momentCmd via e.g. momentCmd.x
  // You'll need the arm length parameter L, and the drag/thrust ratio kappa

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  
  //cmd.desiredThrustsN[0] = mass * 9.81f / 4.f; // front left    //clockwise -           // rotor 1 -- F1
  //cmd.desiredThrustsN[1] = mass * 9.81f / 4.f; // front right   //counter clockwise +   // rotor 2 -- F2
  //cmd.desiredThrustsN[2] = mass * 9.81f / 4.f; // rear left     //counter clockwise +   // rotor 4 -- F4
  //cmd.desiredThrustsN[3] = mass * 9.81f / 4.f; // rear right    //clockwise -           // rotor 3 -- F3
  float l = L / sqrt(2.f);
  float p_bar = momentCmd.x / l;     //p_bar is the net force apply on the x axis
  float q_bar = momentCmd.y / l;     //q_bar is the net force apply on the y axis
  float r_bar = momentCmd.z / kappa; //r_bar is the net force from the sum up the moment of the 4 rotors divided by kappa                                     
  float c_bar = collThrustCmd;       //c_bar is the net lift force

  //Calculation:
  // p_bar = momentCmd.x/l      =    F1 - F2 - F3 + F4
  // q_bar = momentCmd.y/l      =    F1 + F2 - F3 - F4
  // r_bar = momentCmd.z/kappa  =   -F1 + F2 - F3 + F4
  // c_bar = collThrustCmd      =    F1 + F2 + F3 + F4

  // p_bar + q_bar - r_bar + c_bar =  (F1 - F2 - F3 + F4) + (F1 + F2 - F3 - F4) - (-F1 + F2 - F3 + F4) + (F1 + F2 + F3 + F4) = 4*F1
  //-p_bar + q_bar + r_bar + c_bar = -(F1 - F2 - F3 + F4) + (F1 + F2 - F3 - F4) + (-F1 + F2 - F3 + F4) + (F1 + F2 + F3 + F4) = 4*F2
  // p_bar - q_bar + r_bar + c_bar =  (F1 - F2 - F3 + F4) - (F1 + F2 - F3 - F4) + (-F1 + F2 - F3 + F4) + (F1 + F2 + F3 + F4) = 4*F4
  //-p_bar - q_bar - r_bar + c_bar = -(F1 - F2 - F3 + F4) - (F1 + F2 - F3 - F4) - (-F1 + F2 - F3 + F4) + (F1 + F2 + F3 + F4) = 4*F3

  cmd.desiredThrustsN[0] = ( p_bar + q_bar - r_bar + c_bar) / 4.f; // front left    //clockwise -           // rotor 1 -- F1
  cmd.desiredThrustsN[1] = (-p_bar + q_bar + r_bar + c_bar) / 4.f; // front right   //counter clockwise +   // rotor 2 -- F2
  cmd.desiredThrustsN[2] = ( p_bar - q_bar + r_bar + c_bar) / 4.f; // rear left     //counter clockwise +   // rotor 4 -- F4
  cmd.desiredThrustsN[3] = (-p_bar - q_bar - r_bar + c_bar) / 4.f; // rear right    //clockwise -           // rotor 3 -- F3
  
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return cmd;
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
  // Calculate a desired 3-axis moment given a desired and current body rate
  // INPUTS: 
  //   pqrCmd: desired body rates [rad/s]
  //   pqr: current or estimated body rates [rad/s]
  // OUTPUT:
  //   return a V3F containing the desired moments for each of the 3 axes

  // HINTS: 
  //  - you can use V3Fs just like scalars: V3F a(1,1,1), b(2,3,4), c; c=a-b;
  //  - you'll need parameters for moments of inertia Ixx, Iyy, Izz
  //  - you'll also need the gain parameter kpPQR (it's a V3F)

  V3F momentCmd;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  
  V3F I;    //Here I is a 3 elements vector only, not inertia matrix
  V3F pqrErr = pqrCmd - pqr;    //In P controller, we can treat  (pqrCmd - pqr) as the (phiCmd - phi) in (t)
  //cout << "pqrErr is " << pqrErr.x << " " << pqrErr.y << " " << pqrErr.z << " " << endl;
  I.x = Ixx;
  I.y = Iyy;
  I.z = Izz;
  
  // kpPQR * pqrErr will get the acceleration of phi,  I * phi will provide the moment.
  momentCmd = I * kpPQR * pqrErr; // P controller, 
  
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return momentCmd;
}

// returns a desired roll and pitch rate 
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
  // Calculate a desired pitch and roll angle rates based on a desired global
  //   lateral acceleration, the current attitude of the quad, and desired
  //   collective thrust command
  // INPUTS: 
  //   accelCmd: desired acceleration in global XY coordinates [m/s2]
  //   attitude: current or estimated attitude of the vehicle
  //   collThrustCmd: desired collective thrust of the quad [N]
  // OUTPUT:
  //   return a V3F containing the desired pitch and roll rates. The Z
  //     element of the V3F should be left at its default value (0)

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the roll/pitch gain kpBank
  //  - collThrustCmd is a force in Newtons! You'll likely want to convert it to acceleration first

  V3F pqrCmd;
  Mat3x3F R = attitude.RotationMatrix_IwrtB();

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  //accelCmd - from Altitude Control
  //attitude - from simulator. In real world, it generae from gyroscope
  //collThrustCmd - from Altitude Control
  float c_cmd = - collThrustCmd/mass;   // negative sign to adjust the sign of NED using in AltitudeControl
  
  float b_x_cmd = CONSTRAIN(accelCmd.x / c_cmd, -maxTiltAngle, maxTiltAngle); 
  float b_x_actl = R(0, 2);
  float b_x_err = b_x_cmd - b_x_actl;  //we treat p control time constant t is 1, (i.e. 1 unit time), so t is ignore. 
                                       //similar to acc = new_vel - old_vel in 1 unit time.
  float b_x_p_term = b_x_err * kpBank;

  
  float b_y_cmd = CONSTRAIN(accelCmd.y / c_cmd, -maxTiltAngle, maxTiltAngle);  
  float b_y_actl = R(1, 2);
  float b_y_err = b_y_cmd - b_y_actl;   //compare their command tilt angle and actual tilt angle
  float b_y_p_term = b_y_err * kpBank;

  //By equation: [[p_c] [q_c]] = ([[R21 -R11] [R22 -R12]] * [[b_dot_x_c] [b_dot_y_c]]) / R33
  pqrCmd.x = (R(1, 0) * b_x_p_term - R(0, 0) * b_y_p_term) / R(2, 2);
  pqrCmd.y = (R(1, 1) * b_x_p_term - R(0, 1) * b_y_p_term) / R(2, 2);
  
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return pqrCmd;
}

float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
  // Calculate desired quad thrust based on altitude setpoint, actual altitude,
  //   vertical velocity setpoint, actual vertical velocity, and a vertical 
  //   acceleration feed-forward command
  // INPUTS: 
  //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
  //   posZ, velZ: current vertical position and velocity in NED [m]
  //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
  //   dt: the time step of the measurements [seconds]
  // OUTPUT:
  //   return a collective thrust command in [N]

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the gain parameters kpPosZ and kpVelZ
  //  - maxAscentRate and maxDescentRate are maximum vertical speeds. Note they're both >=0!
  //  - make sure to return a force, not an acceleration
  //  - remember that for an upright quad in NED, thrust should be HIGHER if the desired Z acceleration is LOWER

  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float thrust = 0;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  
  float posZErr = posZCmd - posZ;
  float velZErr = velZCmd - velZ;
  integratedAltitudeError += posZErr * dt;  // summation (Sigma) of a constant (c) from 0 to n step with step size 1 = 1 * nc. If the step size is dt, then it is dt*nc

  float p_term = posZErr * kpPosZ;
  float i_term = integratedAltitudeError * KiPosZ;    //to compensate the unexpected extra weight of a drone in scenario 4 of the red drone.
                                                      //to compensate the shift of gravity center
                                                      //integral controller to minor the gap created by unexpected change in weight.
  float d_term = velZErr * kpVelZ;

  //float u_1_bar = p_term + d_term + accelZCmd;            //Feed Forward Added               //for scenario 3
  float u_1_bar = p_term + i_term + d_term + accelZCmd; //Feed Forward Added               //for scenario 4  
  
  float b_z = R(2, 2);

  float acc = (u_1_bar - CONST_GRAVITY) / b_z;  //from the course linear equation

  thrust = -mass * CONSTRAIN(acc, -maxAscentRate / dt, maxAscentRate / dt);
  
  
  /////////////////////////////// END STUDENT CODE ////////////////////////////
  
  return thrust;
}

// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF)
{
  // Calculate a desired horizontal acceleration based on 
  //  desired lateral position/velocity/acceleration and current pose
  // INPUTS: 
  //   posCmd: desired position, in NED [m]
  //   velCmd: desired velocity, in NED [m/s]
  //   pos: current position, NED [m]
  //   vel: current velocity, NED [m/s]
  //   accelCmdFF: feed-forward acceleration, NED [m/s2]
  // OUTPUT:
  //   return a V3F with desired horizontal accelerations. 
  //     the Z component should be 0
  // HINTS: 
  //  - use the gain parameters kpPosXY and kpVelXY
  //  - make sure you limit the maximum horizontal velocity and acceleration
  //    to maxSpeedXY and maxAccelXY

  // make sure we don't have any incoming z-component
  accelCmdFF.z = 0;
  velCmd.z = 0;
  posCmd.z = pos.z;

  // we initialize the returned desired acceleration to the feed-forward value.
  // Make sure to _add_, not simply replace, the result of your controller
  // to this variable
  V3F accelCmd = accelCmdFF;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  
  V3F kpPos;
  kpPos.x = kpPosXY;
  kpPos.y = kpPosXY;
  kpPos.z = 0.f;

  V3F kpVel;
  kpVel.x = kpVelXY;
  kpVel.y = kpVelXY;
  kpVel.z = 0.f;

  if (velCmd.mag() > maxSpeedXY)
      velCmd = velCmd.norm() * maxSpeedXY;

  accelCmd = (posCmd - pos) * kpPos + (velCmd - vel) * kpVel + accelCmdFF;  // added feed forward acceleration 

  if (accelCmd.mag() > maxAccelXY)
      accelCmd = accelCmd.norm() * maxAccelXY;
   
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return accelCmd;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw)
{
  // Calculate a desired yaw rate to control yaw to yawCmd
  // INPUTS: 
  //   yawCmd: commanded yaw [rad]
  //   yaw: current yaw [rad]
  // OUTPUT:
  //   return a desired yaw rate [rad/s]
  // HINTS: 
  //  - use fmodf(foo,b) to unwrap a radian angle measure float foo to range [0,b]. 
  //  - use the yaw control gain parameter kpYaw

  float yawRateCmd=0;
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
   
  if (yawCmd > 0)                           //as per requirement, settle the wrap around issue
      yawCmd = fmodf(yawCmd, 2.f * F_PI);
  else
      yawCmd = -fmodf(-yawCmd, 2.f * F_PI);

  float yawErr = yawCmd - yaw;

  if (yawErr > F_PI)                       //settle the r tuning direction issue
      yawErr = yawErr - 2.F * F_PI;
  if (yawErr < -F_PI)
      yawErr = yawErr + 2.F * F_PI;

  yawRateCmd = yawErr * kpYaw;
  
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return yawRateCmd;

}

VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
  curTrajPoint = GetNextTrajectoryPoint(simTime);

  float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z, dt);

  // reserve some thrust margin for angle control
  float thrustMargin = .1f*(maxMotorThrust - minMotorThrust);
  collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust+ thrustMargin)*4.f, (maxMotorThrust-thrustMargin)*4.f);
  
  V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);
  
  V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
  desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());

  V3F desMoment = BodyRateControl(desOmega, estOmega);

  return GenerateMotorCommands(collThrustCmd, desMoment);
}
