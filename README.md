# cfsd-cognition-velocity-planner
This microservice generates speed requests for Lynx. By analyzing the curvature of the local path the microservice calculates the longitudinal speed of the vehicle and outputs a groundSpeedRequest for the longitudinal control microservice.  
This version includes a graphical visualization of the microservice results. 

### Build
AMD64:
```
docker build -f Dockerfile.Viewer.amd64 -t chalmersfsd/cfsd-cognition-velocity-control:<tag> .
```

### Run
Simply run the docker-compose.yml file or the following command:  
```
docker run --rm -ti --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix chalmersfsd/cfsd-cognition-velocity-planner:<tag> /usr/bin/velocity --cid=219 --ayLimit=5.0 --velocityLimit=10.0 --decelerationLimit=5.0
```

### Dependencies
 - Eigen
 - Pangolin (for visualization)