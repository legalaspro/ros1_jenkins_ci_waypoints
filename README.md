# ROS1 Jenkins CI Waypoints

A ROS1 project demonstrating CI/CD with Jenkins for the TortoiseBot waypoints navigation package.

## Project Structure

```
├── src/                    # ROS packages
│   ├── tortoisebot_description/
│   ├── tortoisebot_gazebo/
│   └── tortoisebot_waypoints/
├── Jenkinsfile             # CI pipeline definition
├── Dockerfile              # Build environment
└── jenkins-infra/          # Jenkins setup scripts and docs
```

## CI/CD Setup

See [`jenkins-infra/README.md`](jenkins-infra/README.md) for Jenkins setup and quick access instructions.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
