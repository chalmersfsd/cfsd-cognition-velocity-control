# cfsd-cognition-velocity-planner
This microservice generates speed requests for Lynx. By analyzing the curvature of the local path the microservice calculates the longitudinal speed of the vehicle and outputs a groundSpeedRequest for the longitudinal control microservice.  

### Build
This microservice can be built with or without graphical visualization of the results:  
AMD64 without viewer: 
```
docker build -f Dockerfile.amd64 -t chalmersfsd/cfsd-cognition-velocity-control:<tag> .
```
AMD64 with viewer: 
```
docker build -f Dockerfile.Viewer.amd64 -t chalmersfsd/cfsd-cognition-velocity-control:<tag> .
```

### Run
Without viewer:  
```
docker run --rm -ti --net=host chalmersfsd/cfsd-cognition-velocity-planner:<tag> /usr/bin/velocity --cid=219 --ayLimit=5.0 --velocityLimit=10.0 --decelerationLimit=5.0
```

With viewer:  
```
docker run --rm -ti --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix chalmersfsd/cfsd-cognition-velocity-planner:<tag> /usr/bin/velocity --cid=219 --ayLimit=5.0 --velocityLimit=10.0 --decelerationLimit=5.0
```


### Dependencies
 - Eigen
 - Pangolin (for visualization)