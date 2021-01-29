## Simulation environment
To start the simulation, run the **pythonServer.py** script.

You can access the simulation in a web browser at http://localhost:4000.

### Folder overview
- namedModels: Neural Network models created by the scripts in [Learning](/Learning) and used by the simulation.
- WebsiteFiles: All resources for the simulation website
  - This folder includes a submodule. Be aware to pull this manually or add submodules to your clone request.
- pythonServer.py: Simple server script
- SimModel2D.py: Helper script to handle model loading
- (logs): Folder created by the simulation to store recordings. Ignored by git.

### Installation guidelines:
The code was tested on Windows 10 and Linux Ubuntu 18.04 builds with
 - python3.6 or python3.8, including the packages
   - python-socketio==4.6.0  !
   - python-engineio==3.13.2 !
   - tensorflow==2.3 or 2.4,
 - zero or one tensorflow accessible GPU,
 - and firefox, edge, or chrome as browsers for the simulation.
