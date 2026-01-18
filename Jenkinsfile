pipeline {
  agent any

  environment {
    DOCKER_IMAGE = 'legalaspro/tortoisebot-noetic-waypoints-ci'
    GAZEBO_STARTUP_TIMEOUT = '60'
  }

  stages {
    stage('Compute Tag') {
      steps {
        script {
          env.IMAGE_TAG = (env.GIT_COMMIT ? env.GIT_COMMIT.take(7) : 'local')
        }
        echo "Using image tag: ${IMAGE_TAG}"
      }
    }

    stage('Build Docker Image') {
      steps {
        echo "Building Docker image: ${DOCKER_IMAGE}:${IMAGE_TAG}"
        sh 'docker build -t ${DOCKER_IMAGE}:${IMAGE_TAG} .'
      }
    }

    stage('Run Tests (Headless)') {
      steps {
        echo "Running tests in: ${DOCKER_IMAGE}:${IMAGE_TAG}"
        sh '''
          docker run --rm \
            -e GAZEBO_STARTUP_TIMEOUT=${GAZEBO_STARTUP_TIMEOUT} \
            ${DOCKER_IMAGE}:${IMAGE_TAG} bash -lc '
              set -e
              source /opt/ros/noetic/setup.bash
              source /catkin_ws/devel/setup.bash

              export ROS_LOG_DIR=/tmp/roslog
              mkdir -p "$ROS_LOG_DIR"

              echo "Launching Gazebo (headless)..."
              roslaunch tortoisebot_gazebo tortoisebot_playground.launch gui:=false headless:=true >/dev/null 2>&1 &

              echo "Waiting for /odom (timeout: ${GAZEBO_STARTUP_TIMEOUT}s)..."
              timeout "${GAZEBO_STARTUP_TIMEOUT}" bash -lc "
                until rostopic echo -n 1 /odom >/dev/null 2>&1; do
                  sleep 1
                done
              " || { echo "ERROR: Timeout waiting for /odom"; exit 1; }

              echo "Simulation ready (/odom is publishing)."
              echo "Running waypoints rostest..."
              rostest tortoisebot_waypoints waypoints_test.test --reuse-master
            '
        '''
      }
    }
  }

  post {
    always {
      sh 'docker image prune -f >/dev/null 2>&1 || true'
    }
    success { echo 'Pipeline completed successfully!' }
    failure { echo 'Pipeline failed!' }
  }
}
