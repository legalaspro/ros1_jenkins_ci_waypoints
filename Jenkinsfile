pipeline {
    agent any

    environment {
        DOCKER_IMAGE = 'legalaspro/tortoisebot-noetic-waypoints-ci'
        GAZEBO_STARTUP_TIMEOUT = '60'  
    }

    stages {
        stage('Build Docker Image') {
            steps {
                echo 'Building Docker image...'
                dir('ros1_jenkins_ci_waypoints') {
                    sh 'docker build -t ${DOCKER_IMAGE}:latest .'
                }
            }
        }

        stage('Run Tests (Headless)') {
            steps {
                echo 'Starting Gazebo in headless mode and running tests...'
                sh '''
                    docker run --rm \
                        ${DOCKER_IMAGE}:latest \
                        /bin/bash -c "
                            set -e
                            source /opt/ros/noetic/setup.bash
                            source /catkin_ws/devel/setup.bash

                            # Launch Gazebo in headless mode (no GUI, no X11 required)
                            echo 'Launching Gazebo in headless mode...'
                            roslaunch tortoisebot_gazebo tortoisebot_playground.launch gui:=false headless:=true &
                            GAZEBO_PID=\$!

                            # Wait for required topics using rostopic with timeout
                            echo 'Waiting for Gazebo simulation to be ready...'
                            timeout ${GAZEBO_STARTUP_TIMEOUT} rostopic echo -n 1 /odom > /dev/null 2>&1 || \
                                { echo 'ERROR: Timeout waiting for /odom topic'; exit 1; }
                            echo 'Simulation ready - /odom topic available'

                            # Run the rostest
                            echo 'Running waypoints test...'
                            rostest tortoisebot_waypoints waypoints_test.test --reuse-master
                            TEST_RESULT=\$?

                            # Cleanup (container auto-removes with --rm, this ensures graceful ROS shutdown)
                            echo 'Cleaning up...'
                            kill -INT \$GAZEBO_PID 2>/dev/null || true
                            wait \$GAZEBO_PID 2>/dev/null || true

                            exit \$TEST_RESULT
                        "
                '''
            }
        }
    }

    post {
        success {
            echo 'Pipeline completed successfully!'
        }
        failure {
            echo 'Pipeline failed!'
        }
    }
}