# Polaris Test
This project contains a Dockerfile to allow you to run the POLARIS_GEM_e2 simulator in gazebo by using a container with pre-built packages and dependencies. 
It also contains a test ros package to allow you to test the project with an output of success or failure based on the cross_track_error values resulted from the simulation
Finally a Jenkins file is setting up the pipeline on a Jenkins service to allow a developer to build the docker image, create a container, execute the simulation, run the test and get the results by email.

## Implemntation

### Dockerfile
For the dockerfile I used multiple layers and mounting points to allow caching from local inline cache from docker buildkit. By using this you can speed up the building times significantly.
This dockerfile pulls a basic ros image, installs basic dependencies then project dependencies and clones the POLARIS project in home folder. Then it uses rosdep to install additional dependencies and builds the project.
It sources ros bash file and opens a bash prompt for the user when the container starts.


### Jenkins File
The jenkins file uses a docker in docker kubernetes pod that had a docker daemon inside the container to allow you to run docker commands.

It uses multiple stages and they can visualised using the blue ocean plugin. 
First it clones this repository that contains the ros test package and then it uses the Dockerfile to build a new image based on the instructions of the docker file. This process takes around 10 minutes to finish. If the developer wants to speed up the process they can uncomment the pulling from dockerhub which takes 1.5 minutes. (I can add this as argument in Jenkins)
Then it creates a container by mounting the local workspace with all the files needed to execute the rostest inside the docker container. This project is mounted inside the ros workspace. 
Then the next stage executes the rostest which launches the simulation in headless mode and runs the test for 30 seconds. Depending on the output it will either push the freshly built image to dockerhub and notify via email with a success email. If it fails it wont push the image as something went wrong and it will send a failure email. 
I have commented out the archiving of test_results as I was running out of time.


### test_cross_error and modified script for waypoints
I had to modify the pure_pursuit_sim.py file to allow it to publish to `/cross_track_error` rostopic so it can be read by the test script. The test script `test_cross_error.py` subscribes to this topic and then waits for 30 seconds while adding to a list the error values. After the 30 seconds it will parse the list and check if any of the values are above 1.0. If yes then the test fails.

## How to run
In order to run the dockerfile you can either build it first locally or pull it from docker hub.

### Building it from Dockerfile
To build it from docker file first clone this project and cd to root of the project folder. 
Then run this: `DOCKER_BUILDKIT=1 docker build --build-arg BUILDKIT_INLINE_CACHE=1 --tag polaris_sim`
This will build the Dockerfile into a docker image with name polaris_sim:latest

To pull the image directly from dockerhub: `docker pull fotiosp/polaris_test:latest`
This will pull the latest built image from dockerhub and store it locally
Then retag it like this: `docker tag fotiosp/polaris_test:latest polaris_test:latest`

To open a container and run the simulation you can use this command:
`docker run -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix polaris_test:latest`

This will open a container with a bash prompt ready to run the simulation. As we didnt pass a --name arge as soon as we exit the container will be removed.

If using WSL2 in windows run this to enable gui output:
`docker run -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /mnt/wslg:/mnt/wslg --device /dev/dxg polaris_test:latest`
