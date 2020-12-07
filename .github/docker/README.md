# How to log on to dockerhub:

# Check what images there are:

    docker images

# How to build docker images

    docker build --tag berndpfrommer/multicam_calibration_xenial:1.0 -f Dockerfile.xenial .

# testing images:

run docker image:

    docker run -it berndpfrommer/xenial_ros:1
	
If you want to run with a persistent storage mounted:

    docker volume create docker_home
    docker run --name test --mount source=docker_home,target=/app -it berndpfrommer/xenial_ros:1
    docker container rm test

# How to push docker images:

    docker push berndpfrommer/xenial_ros:1

    

