# cfsd-logic-recorder
This microservice provides the recorder handling for Lynx.  
The microservice starts and stops the recorder and lables the recorded data correctly based on mission and time of recording.

### Build
AMD64: docker build -f Dockerfile.amd64 -t chalmersfsd/cfsd-logic-recorder:v0.0.1 .

### Run
See included docker-compose file.

### Requirements
The path to the start and stop script (recorder-up.sh and recorder-down.sh) has to be provided when running the container (see yml file) and the scripts have to be present on the host.  
The yml file for the actual recording has to be present in the same directory as the scripts.