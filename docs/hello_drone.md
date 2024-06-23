# Hello Drone

## How does Hello Drone work?

Hello Drone uses the RPC client to connect to the RPC server that is automatically started by the Colosseum. 
The RPC server routes all the commands to a class that implements [MultirotorApiBase](https://github.com/CodexLabsLLC/Colosseum/tree/main/AirLib//include/vehicles/multirotor/api/MultirotorApiBase.hpp). In essence, MultirotorApiBase defines our abstract interface for getting data from the quadrotor and sending back commands. We currently have concrete implementation for MultirotorApiBase for MavLink based vehicles. The implementation for DJI drone platforms, specifically Matrice, is in works.
